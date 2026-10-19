use zed_sdk::{Camera,  Mat, OpenOptions, Resolution, ResolutionPreset, Rgba8, RuntimeParameters, Point3Color};
use std::{fs::File, io::Write};
use std::io::BufWriter;
use anyhow::Error;
use image::{RgbaImage};

use crate::data_types::{pointcloud::PointCloud, intrinsic_info::IntrinsicInfo};


pub struct ZedCamera{
    camera : Camera,
    resolution : Resolution,
    runtime : RuntimeParameters,
}


impl ZedCamera{

    ///Connect to a zed camera and store the required info
    pub fn connect(id : i32) -> Result<ZedCamera, zed_sdk::Error>{

        //Set the options to match those of the realsense camera
        let options = OpenOptions::default()
        .camera_device_id(id)
        .resolution(ResolutionPreset::Hd1080)
        .depth_mode(zed_sdk::DepthMode::Quality)
        .coordinate_unit(zed_sdk::Unit::Meter)
        .coordinate_system(zed_sdk::CoordinateSystem::RightHandedYUp)
        .depth_range_m(Option::from(0.0), 2.0)
        .fps(30);

        let mut camera = Camera::open(options)?;
        let resolution = camera.resolution()?;
        let runtime = RuntimeParameters::default();

        // The first few frames after open can be empty, so grab a handful just to clear it
        for _ in 0..10 {
            camera.grab(&runtime)?;
        }

        Ok(
            ZedCamera {camera, resolution, runtime }
        )
    }

    ///Capture an image from the then save it to the desired filepath
    /// Left determines which lens the image is grabbed from
    pub fn save_image(&mut self, fp : String, left : bool) -> Result<String, anyhow::Error>{
        //Prep an empty frame
        let mut frame = Mat::<Rgba8>::new_cpu(self.resolution)?;

        //Grab the latest frame
        self.camera.grab(&self.runtime)?;

        //Extract the correct lens side
        if left{
            self.camera.retrieve_left(&mut frame)?;
        }else{
            self.camera.retrieve_right(&mut frame)?;
        }

        //Create an immutable view of the colour frame
        let frame_view = frame.view()?;

        let mut image = RgbaImage::from_raw(frame_view.width() as u32, frame_view.height() as u32, frame_view.as_padded_bytes().to_vec()).unwrap();

        //Swap zed BGRA to RGBA - this is a bug with zed_sdk
        for px in image.pixels_mut() {
            px.0.swap(0, 2);
        }     
        
        let fp = format!("{}.png", fp);

        image.save(&fp)?;

        Ok(fp)

    }


    //Report the intrinsic information of the connect ZED camera
    //Left determines if it is from the left or the right lens
    pub fn get_intrinsics(&self, left : bool) ->Result<IntrinsicInfo, anyhow::Error>{
        let cali_params = self.camera.calibration_parameters(false)?;

        let cali_params = if left{
            cali_params.left_cam
        }else{
            cali_params.right_cam
        };       


        Ok( IntrinsicInfo::create(
                cali_params.fx,
                cali_params.fy,
                cali_params.cx,
                cali_params.cy,
                0.0
        ))
    }


    pub fn get_pointcloud(&mut self) -> Result<PointCloud, anyhow::Error>{
        //Prep an empty frame
        let mut frame = Mat::<Point3Color>::new_cpu(self.resolution)?;

        //Grab the latest frame
        self.camera.grab(&self.runtime)?;

        //Get the point map - zed_sdk crate cant access retrieve_xyz...
        self.camera.retrieve_xyzrgba(&mut frame)?;

        //view the point set
        let view = frame.view()?;

        //Convert to the rustgeomapping pointtype
        let vertices : Vec<[f32; 3]> = view.rows()
        .flatten() //collect every row
        .filter_map(|pnt| { //Check every point to make sure it is valid
                            let v = [pnt.x, pnt.y, pnt.z];
                            v.iter().all(|c| c.is_finite()).then_some(v) //Only keep if valid
                        })
        .collect();


        PointCloud::create_from_iter(vertices)
    }

}