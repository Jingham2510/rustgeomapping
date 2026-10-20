use crate::backend::realsense::realsense_cam::RealsenseCam;
use crate::backend::zed::zed_cam::ZedCamera;
///High levels functions used to generate information from the RGBD cameras
use crate::data_types::pointcloud::PointCloud;
use crate::data_types::intrinsic_info::IntrinsicInfo;

use std::fmt;

///Available depth camera types
pub enum CamType {
    RealsenseCam(DepthCam<RealsenseCam>),
    ZedCamera(DepthCam<ZedCamera>)
}

impl CamType {
    ///Take a pointcloud regardless of camera type
    pub fn take_pcl(&mut self) -> Result<PointCloud, anyhow::Error> {
        match self {
            CamType::RealsenseCam(cam)=> cam.get_pointcloud(),
            CamType::ZedCamera(cam) => cam.get_pointcloud()
        }
    }


    pub fn get_intrinsics(&self) ->Result<IntrinsicInfo, anyhow::Error>{
        match self{
            CamType::RealsenseCam(cam) => cam.get_intrinsic(),
            CamType::ZedCamera(cam) => cam.get_intrinsic()
        }
    }

    pub fn get_colour_image(&mut self, filepath : &str) -> Result<String, anyhow::Error>{
        match self{
            CamType::RealsenseCam(cam) => cam.get_colour_image(filepath),
            CamType::ZedCamera(cam) => cam.get_colour_image(filepath)
        }
    }


    pub fn get_pos(&mut self) -> Result<[f32;3], anyhow::Error>{
        match self{
            CamType::RealsenseCam(cam) => cam.get_pos(),
            CamType::ZedCamera(cam) => cam.get_pos()
        }
    }

    pub fn id(&self) -> u32{
        match self{
            CamType::RealsenseCam(cam) => cam.id(),
            CamType::ZedCamera(cam) => cam.id()
            
        }
    }

    pub fn cam_type(&self) -> &str{
        match self{
            CamType::RealsenseCam(cam) => "Realsense",
            CamType::ZedCamera(cam) => "Zed"            
        }
    }

    pub fn imu_enabled(&self) -> bool{
        match self{
            CamType::RealsenseCam(cam) => {cam.imu_enabled()},
            CamType::ZedCamera(cam) => {cam.imu_enabled()}           
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
impl fmt::Display for DepthCam<ZedCamera> {
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

    ///Get the intrinsic information from a camera
    fn get_intrinsic(&self) -> Result<IntrinsicInfo, anyhow::Error>;

    ///Save a colour image
    fn get_colour_image(&mut self, filepath: &str ) -> Result<String, anyhow::Error>;

    ///Get the estimated relative translation
    fn get_pos(&mut self) -> Result<[f32;3], anyhow::Error>;

    ///Check if the imu is enabled
    fn imu_enabled(&self) -> bool;

    ///Reset all connected hardware
    fn reset_all() -> Result<(), anyhow::Error>;

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

    fn get_intrinsic(&self) -> Result<IntrinsicInfo, anyhow::Error>{
        self.cam.get_intrinsics()
    }

    fn get_colour_image(&mut self, filepath: &str ) -> Result<String, anyhow::Error>{
        self.cam.get_image(filepath)
    }

    fn get_pos(&mut self) -> Result<[f32;3], anyhow::Error>{
        self.cam.get_rel_pos()
    }


    fn imu_enabled(&self)->bool{
        self.cam.imu_enabled()
    }

    fn reset_all() -> Result<(), anyhow::Error>{
        RealsenseCam::reset_all()
    }

}



impl Required<ZedCamera> for DepthCam<ZedCamera> {
    fn connect(id: u32) -> Result<DepthCam<ZedCamera>, anyhow::Error> {
        let zed_cam = ZedCamera::connect(id as i32)?;

        Ok(DepthCam {
            cam: zed_cam,
            id,
        })
    }

    fn get_pointcloud(&mut self) -> Result<PointCloud, anyhow::Error> {
        //Range is determined by the setup config
        self.cam.get_pointcloud(true)
    }

    fn get_intrinsic(&self) -> Result<IntrinsicInfo, anyhow::Error>{
        self.cam.get_intrinsics(true)
    }

    fn get_colour_image(&mut self, filepath: &str ) -> Result<String, anyhow::Error>{
        //Auto assume left lens
        self.cam.save_image(String::from(filepath), true)
    }

    fn get_pos(&mut self) -> Result<[f32;3], anyhow::Error>{
        self.cam.get_rel_pos(true)
    }

    //Not implemented
    fn reset_all() -> Result<(), anyhow::Error>{
        todo!()
    }


    //IMU always on
    fn imu_enabled(&self)->bool{ true }

}

impl DepthCam<RealsenseCam> {
    pub fn connect_realsense(id: u32) -> Result<Self, anyhow::Error> {
        DepthCam::connect(id)
    }

    pub fn id(&self) -> u32{
        self.id
    }
}

impl DepthCam<ZedCamera> {
    pub fn connect_zed(id: u32) -> Result<Self, anyhow::Error> {
        DepthCam::connect(id)
    }

    pub fn id(&self) -> u32{
        self.id
    }
}