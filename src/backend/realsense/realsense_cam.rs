/// Configure and stream a 435i sensor.

/// Notice that the streaming configuration changes based on the USB speed of the sensor.
/// If one attempts to set a streaming configuration that is too much for the current USB
/// speed, RealSense will return with an error. However, that error is non-descript and will
/// not help identify the underlying problem, i.e. the bandwidth of the connection.
use anyhow::{Result, bail, ensure};
use realsense_rust;
use realsense_rust::frame::{ColorFrame, PointsFrame};
use realsense_rust::pipeline::ActivePipeline;
use realsense_rust::{
    config::Config,
    context::Context,
    frame::DepthFrame,
    kind::{Rs2CameraInfo, Rs2Format, Rs2ProductLine, Rs2StreamKind},
    pipeline::InactivePipeline,
};

use std::process::{Command, Stdio};
use std::{collections::HashSet, convert::TryFrom, time::Duration};

use crate::backend::realsense::rs2_process_frame::{FrameProc, FrameProcBlock, create_color_image};

///The realsense camera
pub struct RealsenseCam {
    pub cam_no: usize,
    pipeline: ActivePipeline,
    pcl_block: FrameProcBlock<PointsFrame>,
}

impl RealsenseCam {
    ///Connect and Initialise a camera
    /// When using multiple cameras, cam_no is used to specify which one to connect to
    pub fn initialise_raw(cam_no: usize) -> Result<Self, anyhow::Error> {
        let (pipeline, config) = Self::setup(cam_no)?;

        Ok(Self {
            cam_no,
            //Start the pipeline in the camera object
            //(This is done to get around the pipeline being consumed
            pipeline: pipeline.start(Some(config))?,
            pcl_block: FrameProcBlock::make_raw_points_block(1)?,
        })
    }

    fn setup(cam_no: usize) -> Result<(InactivePipeline, Config), anyhow::Error> {
        // Check for depth or color-compatible devices.
        let mut queried_devices = HashSet::new();
        queried_devices.insert(Rs2ProductLine::D400);
        let context = Context::new()?;
        let devices = context.query_devices(queried_devices);
        ensure!(!devices.is_empty(), "No devices found");

        //If the provided camera number is too high - bail
        if cam_no > devices.len() {
            bail!("Invalid camera selection");
        }

        /*
        println!("devs: {:?}", devices);
        println!(
            "Selected: {:?}",
            devices[cam_no].info(Rs2CameraInfo::SerialNumber)
        );
        */

        //Try and create the inactive pipeline
        let pipeline = InactivePipeline::try_from(&Context::new()?)?;

        //Create a new config
        let mut config = Config::new();

        //Only enable the  depth stream (not bothered about gyro/infra/colour atm)
        config
            .enable_device_from_serial(devices[cam_no].info(Rs2CameraInfo::SerialNumber).unwrap())?
            .disable_all_streams()?
            //.enable_all_streams()?;
            //Height of 0 indicates to realsense that it should select the most appropriate height itself
            .enable_stream(Rs2StreamKind::Depth, None, 1280, 0, Rs2Format::Z16, 30)?
            .enable_stream(Rs2StreamKind::Color, None, 1280, 0, Rs2Format::Rgb8, 30)?;

        Ok((pipeline, config))
    }

    ///Return a raw xyz pointcloud from the camera
    pub fn get_depth_pnts(&mut self, max_range: f32) -> Result<Vec<[f32; 3]>> {
        //Wait for a frame to arrive
        let frame = self.pipeline.wait(None)?;
        //Extract the depth frame
        let mut depth_frame = frame.frames_of_type::<DepthFrame>();

        //Check the depth frame isnt empty
        if depth_frame.is_empty() {
            bail!("No depth frame to read!");
        }

        //Add the frame to the queue to extract the point cloud
        self.pcl_block
            .queue(depth_frame.pop().unwrap())
            .expect("FAILED TO QUEUE DEPTH FRAME");

        //Wait a maximum of 1000ms for the frame to be processed
        let point_frame: PointsFrame = self.pcl_block.wait(Duration::from_millis(1000))?;

        //Turn the point frame into a vector of floats
        let mut points: Vec<[f32; 3]> = vec![];

        for vertex in point_frame.vertices().iter() {
            let vertex = vertex.xyz;
            if vertex[2] == 0.0 || vertex[2] > max_range {
                continue;
            }
            points.push(vertex)
        }

        Ok(points)
    }

    ///Get a colour image from the realsense camera
    pub fn get_image(&mut self, filename: &str) -> Result<(), anyhow::Error> {
        //Wait for a frame to arrive
        let frame = self.pipeline.wait(None)?;
        //Extract the first colour frame
        let color_frame = frame.frames_of_type::<ColorFrame>();

        //Check the depth frame isnt empty
        if color_frame.is_empty() {
            bail!("No color frame to read!");
        }

        //Process the image
        let image = create_color_image(color_frame.get(0).unwrap())?;

        //Save the image
        let fp = format!("appdata\\{}.png", filename);
        image.save(fp)?;

        println!("Image saved!");

        Ok(())
    }

    ///Return all aruco tags visible to the camera
    pub fn get_aruco_tags(&mut self) -> Result<Vec<(usize, [(i32, i32); 4])>, anyhow::Error> {
        //Take an image and save it in the application data
        let tag_img_fp = format!("aruco_detect_{}", self.cam_no);
        self.get_image(&tag_img_fp)?;

        //Run the python script - only works on windows machines
        let py_cmd = Command::new(
            //,
            "cmd",
        )
        .args([
            "/C",
            &format!(
                "py src\\aruco_detection\\detect_aruco_id.py appdata\\{}.png",
                tag_img_fp
            ),
        ])
        .stdout(Stdio::piped())
        .output()
        .unwrap();

        //Get the output string from the python file and split it line by line
        let out_string = String::from_utf8(py_cmd.stdout).unwrap();
        let line_split: Vec<&str> = out_string.split("\n").collect();

        //Get the number of IDs detected
        let no_of_ids: usize = line_split[1]
            .trim()
            .replace("ID_COUNT:", "")
            .parse()
            .unwrap();

        println!("Number of IDs detected: {}", no_of_ids);

        if no_of_ids == 0 {
            bail!("No ids detected")
        }

        let mut id_info: Vec<(usize, [(i32, i32); 4])> = vec![];

        //Extract the id information
        let lines_per_id = 4;
        for i in 0..no_of_ids {
            let start_line = i + 2 + (i * lines_per_id);
            println!("{}", line_split[start_line]);

            //Get the id number
            let id_no: usize = line_split[start_line]
                .trim()
                .replace("MARK:[[", "")
                .replace("]]", "")
                .parse()
                .unwrap();

            //Get the corner information
            let mut corners: [(i32, i32); 4] = [(0, 0), (0, 0), (0, 0), (0, 0)];
            for j in 1..=4 {
                let corner_str: &str = line_split[start_line + j];

                let corner_split = corner_str
                    .trim()
                    .replace("CORN:", "")
                    .replace("[", "")
                    .replace("]", "");

                let corner_split: Vec<&str> = corner_split.split(".").collect();

                corners[j - 1] = (
                    corner_split[0].trim().parse().unwrap(),
                    corner_split[1].trim().parse().unwrap(),
                );
            }
            id_info.push((id_no, corners));
        }

        Ok(id_info)
    }
}
