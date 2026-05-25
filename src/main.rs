use rustgeomapping::depth_cam::{DepthCam, Required};
use rustgeomapping::backend::realsense::realsense_cam::RealsenseCam;


fn main(){
    let mut cam :DepthCam<RealsenseCam> = DepthCam::connect_realsense(0).expect("Failed to connect");

    println!("{}", cam);

    



    let pnts = cam.get_pointcloud().expect("Failed to get points");

    pnts.save_to_file("test");



}