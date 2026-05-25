use crate::backend::realsense::realsense_cam::{self, RealsenseCam};
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
    fn connect(id: u32) -> Result<DepthCam<T>, anyhow::Error>;
}

///Take a pointcloud from the camera
pub trait GetPointCloud<T> {
    fn get_pointcloud(&self) -> Result<PointCloud, anyhow::Error>;
}

impl Connect<RealsenseCam> for DepthCam<RealsenseCam> {
    fn connect(id: u32) -> Result<DepthCam<RealsenseCam>, anyhow::Error> {
        let realsense_cam = RealsenseCam::initialise_raw(id as usize)?;

        Ok(DepthCam {
            cam: realsense_cam,
            cam_type: CamType::Realsense,
            id,
        })
    }
}

impl GetPointCloud<RealsenseCam> for DepthCam<RealsenseCam> {
    fn get_pointcloud(&self) -> Result<PointCloud, anyhow::Error> {
        const MAX_RANGE: f32 = 2.0;

        let pcl_info = self.cam.get_depth_pnts(MAX_RANGE);

        //Split the info into cloud and timestamp
        PointCloud::create_from_iter(pcl_info.0, pcl_info.1)
    }
}
