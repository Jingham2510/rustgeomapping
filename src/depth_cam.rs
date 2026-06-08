use crate::backend::realsense::realsense_cam::RealsenseCam;
///High levels functions used to generate information from the RGBD cameras
use crate::data_types::pointcloud::PointCloud;

use std::fmt;

///Available depth camera types
pub enum CamType {
    RealsenseCam(DepthCam<RealsenseCam>),
}

impl CamType {
    ///Take a pointcloud regardless of camera type
    pub fn take_pcl(&mut self) -> Result<PointCloud, anyhow::Error> {
        match self {
            CamType::RealsenseCam(cam) => cam.get_pointcloud(),
        }
    }
}

///Generic depth camera implementation

pub struct DepthCam<T> {
    cam: T,
    id: u32,
}

impl fmt::Display for DepthCam<RealsenseCam> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Cam ID: {}", self.id)
    }
}

///The minimum required traits for a depth camera to be useful

pub trait Required<T> {
    ///Connect to the camera
    fn connect(id: u32) -> Result<DepthCam<T>, anyhow::Error>;
    ///Take a pointcloud from the camera
    fn get_pointcloud(&mut self) -> Result<PointCloud, anyhow::Error>;
}

impl Required<RealsenseCam> for DepthCam<RealsenseCam> {
    fn connect(id: u32) -> Result<DepthCam<RealsenseCam>, anyhow::Error> {
        let realsense_cam = RealsenseCam::initialise_raw(id as usize)?;

        Ok(DepthCam {
            cam: realsense_cam,
            id,
        })
    }

    fn get_pointcloud(&mut self) -> Result<PointCloud, anyhow::Error> {
        const MAX_RANGE: f32 = 2.0;

        let pcl_info = self.cam.get_depth_pnts(MAX_RANGE)?;

        //Split the info into cloud and timestamp
        PointCloud::create_from_iter(pcl_info)
    }
}

impl DepthCam<RealsenseCam> {
    pub fn connect_realsense(id: u32) -> Result<Self, anyhow::Error> {
        DepthCam::connect(id)
    }
}
