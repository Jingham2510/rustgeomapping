use rustgeomapping::depth_cam::DepthCam;
use rustgeomapping::backend::realsense::realsense_cam::RealsenseCam;


fn main(){
    let cam :DepthCam<RealsenseCam> = DepthCam::connect_realsense(0).expect("Failed to connect");

    println!("{}", cam);

    


}