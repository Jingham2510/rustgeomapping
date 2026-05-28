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
    ///The timestamp of when the pointcloud was captured (relative to the epoch)
    global_timestamp: DateTime<Utc>,
    ///The name of the file the pointcloud was loaded from (if loaded)
    filename: Option<String>,
    ///The rectangular bounds of the point cloud
    bounds : [f32;6],
}

impl PointCloud {
    //Creates an empty point cloud with no information
    pub fn new() -> Self{
        Self { points: vec![], no_of_points: 0, global_timestamp: Utc::now(), filename: None, bounds : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] }
    }


    ///Create a pointcloud from a list of points
    pub fn create_from_list(pnts: Vec<[f32; 3]>,) -> Self {
        //Calculate the number of points
        let no_of_points = pnts.len();


        Self {
            bounds : PointCloud::calculate_bounds(&pnts),
            points: pnts,
            no_of_points,
            global_timestamp: Utc::now(),
            filename: None,
            
        }
    }

    ///Create a pointcloud from a list of vertices - assuming points are valid
    pub fn create_from_iter(
        vertices: Vec<[f32; 3]>
    ) -> Result<Self, anyhow::Error> {
        let mut points: Vec<[f32; 3]> = vec![];
        let mut no_of_points = 0;

        let mut x_max = -9999.0;
        let mut x_min = 9999.0;
        let mut y_max = -9999.0;
        let mut y_min = 9999.0;
        let mut z_max = -9999.0;
        let mut z_min = 9999.0;



        for vertex in vertices.iter() {
  
            let x = vertex[0];
            let y = vertex[1];
            let z = vertex[2];

            if x > x_max{
                x_max = x;
            }else if x < x_min{
                x_min = x
            }
            if y > y_max{
                y_max = y;
            }else if y < y_min{
                y_min = y
            }
            if z > z_max{
                z_max = z;
            }else if z < z_min{
                z_min = z
            }
       

            points.push([vertex[0], vertex[1], vertex[2]]);
            no_of_points += 1;
        }

        Ok(Self{
            points,
            no_of_points,
            global_timestamp: Utc::now(),
            filename: None,   
            bounds : [x_min, x_max, y_min, y_max, z_min, z_max]         
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
            bounds : PointCloud::calculate_bounds(&points),
            points,
            no_of_points,
            global_timestamp,
            filename: Some(fpath),
        })
    }

    fn calculate_bounds(points : &Vec<[f32;3]>) -> [f32;6]{

         let mut x_max = -9999.0;
        let mut x_min = 9999.0;
        let mut y_max = -9999.0;
        let mut y_min = 9999.0;
        let mut z_max = -9999.0;
        let mut z_min = 9999.0;


        for pnt in points.into_iter(){
            let x = pnt[0];
            let y = pnt[1];
            let z = pnt[2];

            if x > x_max{
                x_max = x;
            }else if x < x_min{
                x_min = x
            }
            if y > y_max{
                y_max = y;
            }else if y < y_min{
                y_min = y
            }
            if z > z_max{
                z_max = z;
            }else if z < z_min{
                z_min = z
            }
        }

        [x_min, x_max, y_min, y_max, z_min, z_max]

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

    ///Get the xyz bounds of the point cloud
    pub fn bounds(&self) -> [f32;6]{
        self.bounds
    }
    

    ///Calculate the xy plane area of the pcl
    pub fn get_xy_area(&self) -> f32 {       

        ((self.bounds[1] - self.bounds[0]) * (self.bounds[3] - self.bounds[2])).abs()
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
        self.update_bounds();
    }

    ///Translate a pointcloud with xyz coords
    pub fn translate(&mut self, x: f32, y: f32, z: f32) {

        self.points = self.points.iter().map(|[pnt_x,pnt_y,pnt_z]| {[pnt_x + x, pnt_y+y, pnt_z+z]}).collect::<Vec<[f32;3]>>();

        let curr_bounds = self.bounds;

        self.bounds = [curr_bounds[0] + x, curr_bounds[1] + x, curr_bounds[2] + y, curr_bounds[3] + y, curr_bounds[4] + z, curr_bounds[5] + z]

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

        //Calculate the new bounds by transforming the point
        let curr_bounds = self.bounds;

        let mut min_pnt = Vector4::new(curr_bounds[0], curr_bounds[2], curr_bounds[4], 1.0);
        let mut max_pnt = Vector4::new(curr_bounds[1], curr_bounds[3], curr_bounds[5], 1.0);

        min_pnt = tmat * min_pnt;
        max_pnt = tmat * max_pnt;

        self.bounds = [min_pnt[0], max_pnt[0], min_pnt[1], max_pnt[1], min_pnt[2], max_pnt[2]]



    }

    fn update_bounds(&mut self){
        self.bounds = PointCloud::calculate_bounds(&self.points)
    }

    ///Scale every point by the same value
    pub fn scale_even(&mut self, scale_val: f32) {

        self.points = self.points.iter_mut().map(|[x, y, z]|{[*x * scale_val, *y * scale_val, *z * scale_val]}).collect::<Vec<[f32;3]>>();
        
        let bnds = self.bounds;
        self.bounds = [bnds[0]* scale_val, bnds[1]* scale_val, bnds[2]* scale_val, bnds[3]* scale_val, bnds[4]* scale_val, bnds[5]* scale_val];

    }

    ///Filter a pointcloud by specifying the valid bounds
    ///Inclusive of points that lie on the boundary
    pub fn crop(&mut self, min_x: f32, max_x: f32, min_y: f32, max_y: f32, min_z: f32, max_z: f32) {

        //Filter the points out
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
        //Update the bounds
        self.update_bounds();
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
    pub fn combine(&mut self, mut other: PointCloud) {

        self.points.append(&mut other.points);

        //Update the bounds
        self.bounds = [*[self.bounds[0], other.bounds[0]].iter().min_by(|a, b| a.total_cmp(b)).unwrap(), 
        *[self.bounds[1], other.bounds[1]].iter().max_by(|a, b| a.total_cmp(b)).unwrap(),
                     *[self.bounds[2], other.bounds[2]].iter().min_by(|a, b| a.total_cmp(b)).unwrap(), 
                     *[self.bounds[3], other.bounds[3]].iter().max_by(|a, b| a.total_cmp(b)).unwrap(),
                     *[self.bounds[4], other.bounds[4]].iter().min_by(|a, b| a.total_cmp(b)).unwrap(),
                      *[self.bounds[5], other.bounds[5]].iter().max_by(|a, b| a.total_cmp(b)).unwrap()];

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
