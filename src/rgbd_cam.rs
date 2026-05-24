///High levels functions used to generate information from the RGBD cameras
use crate::data_types::pointcloud::PointCloud;
use anyhow::bail;

///Available depth camera types
enum CamType {
    Realsense,
}

///Generic depth camera implementation
pub struct DepthCam<T> {
    cam: T,
    cam_type: CamType,
    id: u32,
}

///Connect to the camera
pub trait Connect<T> {
    fn connect() -> DepthCam<T>;
}

///Take a pointcloud from the camera
pub trait GetPointCloud<T> {
    fn get_pointcloud(depth_cam: &DepthCam<T>) -> Result<PointCloud, anyhow::Error>;
}
