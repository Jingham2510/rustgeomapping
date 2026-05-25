///A set of tools around the creation and manipulation of point cloud objects
use anyhow::bail;
use chrono::{DateTime, Utc};
use nalgebra::{Matrix4, Vector4};
use std::fs::File;
use std::io::{BufRead, BufReader, Write};

///A point cloud
pub struct PointCloud {
    ///Vector of the points stored in xyz format
    points: Vec<[f32; 3]>,
    ///The number of points present
    no_of_points: usize,
    ///The timestamp of when the pointcloud was captured (relative to when the camera started)
    rel_timestamp: f64,
    ///The timestamp of when the pointcloud was captured (relative to the epoch)
    global_timestamp: DateTime<Utc>,
    ///The name of the file the pointcloud was loaded from (if loaded)
    filename: Option<String>,
}

impl PointCloud {
    ///Create a pointcloud from a list of points
    pub fn create_from_list(pnts: Vec<[f32; 3]>, timestamp: f64) -> Self {
        //Calculate the number of points
        let no_of_points = pnts.len();

        Self {
            points: pnts,
            no_of_points,
            rel_timestamp: timestamp,
            global_timestamp: Utc::now(),
            filename: None,
        }
    }

    ///Create a pointcloud from a list of vertices
    pub fn create_from_iter(vertex: &[[f32; 3]], timestamp: f64) -> Result<Self, anyhow::Error> {
        let mut points: Vec<[f32; 3]> = vec![];
        let mut no_of_points = 0;

        for vertex in vertex.iter() {
            //check if the point is valid - if not ignore it
            if vertex[2] == 0.0 || vertex[2] > 2.0 {
                continue;
            }

            points.push([vertex[0] as f32, vertex[1] as f32, vertex[2] as f32]);
            no_of_points += 1;
        }

        Ok(Self {
            points,
            no_of_points,
            rel_timestamp: timestamp,
            global_timestamp: Utc::now(),
            filename: None,
        })
    }

    ///Create a pointcloud from a loaded file
    pub fn create_from_file(filepath: String) -> Result<Self, anyhow::Error> {
        //Create the filepath
        let fpath;
        if let Some(fp) = filepath.strip_suffix(".txt") {
            fpath = fp.to_string();
        } else {
            bail!("Failed to create filepath")
        }

        //Open the file and create a buffer to read the lines
        let file = File::open(filepath.clone())?;
        let mut line_reader = BufReader::new(file);

        //Read the first line to get the date
        let timestamp_str: &mut String = &mut "".to_string();
        line_reader.read_line(timestamp_str)?;

        //Convert the string to the datetime f64
        let global_timestamp: DateTime<Utc> = timestamp_str.parse()?;

        //Setup the empty points
        let mut points = vec![];

        for line in line_reader.lines() {
            //Create empty point
            let mut pnt: [f32; 3] = [f32::NAN, f32::NAN, f32::NAN];

            //Delimit the line based on commas
            for (cnt, token) in line?.split(",").enumerate() {
                pnt[cnt] = token.parse()?;
            }

            points.push(pnt);
        }

        let no_of_points = points.len();

        Ok(Self {
            points,
            no_of_points,
            //Relative timestamp is -1.0 because there is no reference start time
            rel_timestamp: -1.0,
            global_timestamp,
            filename: Some(fpath),
        })
    }

    ///Get the points in the point cloud without destroying the cloud
    pub fn points(&self) -> Vec<[f32; 3]> {
        self.points.clone()
    }

    ///Get the points in the point cloud whilst consuming the cloud
    pub fn consume_points(self) -> Vec<[f32; 3]> {
        self.points
    }

    pub fn timestamp(&self) -> DateTime<Utc> {
        self.global_timestamp
    }

    ///Get the number of points in the cloud
    pub fn size(&self) -> usize {
        self.no_of_points
    }

    pub fn filename(&self) -> Option<String> {
        self.filename.clone()
    }

    ///Print all points in the point cloud
    pub fn print_points(&self) {
        for pnt in self.points.iter() {
            println!("{:?}", pnt);
        }
    }

    ///Calculate the xy bounds of the pointcloud (rectangular)
    ///Assumes z is the height
    pub fn get_bounds(&self) -> [f32; 4] {
        //Predefine the values we are interested in
        let mut x_min: f32 = 9999.0;
        let mut y_min: f32 = 9999.0;
        let mut x_max: f32 = -9999.0;
        let mut y_max: f32 = -9999.0;

        //Check each point to if it escapes the set bounds
        //If points are sorted beforehand, no need! but sorting might take a while - and how do you sort?
        for pnt in self.points.iter() {
            //Check x-bounds
            if pnt[0] < x_min {
                x_min = pnt[0];
            } else if pnt[0] > x_max {
                x_max = pnt[0];
            }

            //Check y-bounds
            if pnt[1] < y_min {
                y_min = pnt[1];
            } else if pnt[1] > y_max {
                y_max = pnt[1];
            }
        }

        //Return the bounding coordinates of the rectangle
        [x_min, x_max, y_min, y_max]
    }

    ///Calculate the xy plane area of the pcl
    pub fn get_xy_area(&self) -> f32 {
        let bounds = self.get_bounds();

        ((bounds[1] - bounds[0]) * (bounds[3] - bounds[2])).abs()
    }

    ///Rotate a pointcloud using Euler angles
    ///Can cheat with the mat multiplication here, we know the predefined sizes already
    ///Yaw - Z
    ///Pitch - Y
    ///Roll - X
    pub fn rotate(&mut self, yaw: f32, pitch: f32, roll: f32) {
        //Precalc the trig
        let yaw_c = yaw.cos();
        let yaw_s = yaw.sin();

        let pitch_c = pitch.cos();
        let pitch_s = pitch.sin();

        let roll_c = roll.cos();
        let roll_s = roll.sin();

        //Create the transform matrix rows
        let x_rot = [
            yaw_c * pitch_c,
            ((yaw_c * pitch_s * roll_s) - (yaw_s * roll_c)),
            ((yaw_c * pitch_s * roll_c) + (yaw_s * roll_s)),
        ];

        let y_rot = [
            yaw_s * pitch_c,
            (yaw_s * pitch_s * roll_s) + (yaw_c * roll_c),
            (yaw_s * pitch_s * roll_c) - (yaw_c * roll_s),
        ];
        let z_rot = [-pitch_s, pitch_c * roll_s, pitch_c * roll_c];

        //Iterate through every point in the pointcloud
        for pnt in self.points.iter_mut() {
            //store the original points
            let og_x = pnt[0];
            let og_y = pnt[1];
            let og_z = pnt[2];

            //Rotate by multiplying the vector by the transform matrix
            pnt[0] = (x_rot[0] * og_x) + (x_rot[1] * og_y) + (x_rot[2] * og_z);
            pnt[1] = (y_rot[0] * og_x) + (y_rot[1] * og_y) + (y_rot[2] * og_z);
            pnt[2] = (z_rot[0] * og_x) + (z_rot[1] * og_y) + (z_rot[2] * og_z);
        }
    }

    ///Translate a pointcloud with xyz coords
    pub fn translate(&mut self, x: f32, y: f32, z: f32) {
        //Iterate through every point
        for pnt in self.points.iter_mut() {
            //Transform the points using addition
            pnt[0] += x;
            pnt[1] += y;
            pnt[2] += z;
        }
    }

    ///Transform a pointcloud by using a homogenous matrix
    pub fn transform_with(&mut self, tmat: &Matrix4<f32>) {
        //Transform each point
        for pnt in self.points.iter_mut() {
            //Pad a 0 for vlaid matrix multiplication
            let temp_pnt = Vector4::new(pnt[0], pnt[1], pnt[2], 1.0);

            //Multiple the matrix and the temp point
            let temp_pnt = tmat * temp_pnt;

            pnt[0] = temp_pnt[0];
            pnt[1] = temp_pnt[1];
            pnt[2] = temp_pnt[2];
        }
    }

    ///Scale every point by the same value
    pub fn scale_even(&mut self, scale_val: f32) {
        //Iterate through every point
        for pnt in self.points.iter_mut() {
            //Transform the points using addition
            pnt[0] *= scale_val;
            pnt[1] *= scale_val;
            pnt[2] *= scale_val;
        }
    }

    ///Filter a pointcloud by specifying the valid bounds
    ///Inclusive of points that lie on the boundary
    pub fn crop(&mut self, min_x: f32, max_x: f32, min_y: f32, max_y: f32, min_z: f32, max_z: f32) {
        let pnts = self.points();

        let mut new_pnts: Vec<[f32; 3]> = vec![];

        for pnt in pnts {
            //Drop all the points that sit outside the bounds
            if pnt[0] < min_x
                || pnt[0] > max_x
                || pnt[1] < min_y
                || pnt[1] > max_y
                || pnt[2] < min_z
                || pnt[2] > max_z
            {
                continue;
            } else {
                new_pnts.push(pnt);
            }
        }

        //Store the passbanded poimnts
        self.points = new_pnts;
        //Update the point count
        self.no_of_points = self.points.len();
    }

    ///Save the pointcloud to a text file
    pub fn save_to_file(&self, filepath: &str) -> Result<(), anyhow::Error> {
        //Create a file
        let mut file = File::create(filepath.to_owned() + ".txt")?;

        //Save the timestamp from the frame as the first line;
        let datetime_fmt = format!("{}\n", self.global_timestamp);

        file.write_all(datetime_fmt.as_bytes())?;

        //Save the cloud points on seperate lines
        for i in 0..self.no_of_points {
            let pnt = format!(
                "{:?},{:?},{:?}\n",
                self.points[i][0], self.points[i][1], self.points[i][2]
            );

            file.write_all(pnt.as_bytes())?;
        }

        //Return the all clear
        Ok(())
    }

    ///Returns whether the pointcloud is registered as the last of a series
    ///Based on the filename
    pub fn is_end(&self) -> bool {
        //Check that the pointcloud has a filename
        if let Some(fp) = &self.filename {
            return fp.contains("_END");
        }
        false
    }

    ///Absorbs another pointcloud into this pointcloud
    ///Timestamp of previous pointcloud becomes timestamp of this one
    pub fn combine(&mut self, other: PointCloud) {
        for pnt in other.points {
            self.points.push(pnt);
            self.no_of_points += 1;
        }
    }

    ///Applies a decimation filter to the pointcloud, removing every nth point
    pub fn decimation_filter(&mut self, n: i32) {
        //Go through every point
        for i in 0..self.no_of_points {
            if (i as i32) % n == 0 {
                self.points.remove(i);
            }
        }
    }
}
