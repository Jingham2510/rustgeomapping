use rustgeomapping::depth_cam::{DepthCam, Required};
use rustgeomapping::backend::realsense::realsense_cam::RealsenseCam;

use std::io::stdin;

fn main(){
    let mut cam :DepthCam<RealsenseCam> = DepthCam::connect_realsense(0).expect("Failed to connect");

    println!("{}", cam);

    let mut host_resp = String::new();

    stdin()
        .read_line(&mut host_resp)
        .expect("Failed to read line");


    




    let pnts = cam.get_pointcloud().expect("Failed to get points");

    
    println!("{:?}", pnts.consume_points());
    



}