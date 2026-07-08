///A handful of useful computer vision algorithms for image processing using the rust opencv wrapper
use nalgebra::Matrix4;
use anyhow::bail;

use crate::data_types::intrinsic_info::IntrinsicInfo;

use opencv::prelude::*;
use opencv::objdetect::{ArucoDetector, PredefinedDictionaryType, get_predefined_dictionary, DetectorParameters, RefineParameters};
use opencv::imgcodecs::{imread, IMREAD_GRAYSCALE};
use opencv::core::{Point2f, Point3f, Vector, Mat, MatTrait};
use opencv::calib3d::{solve_pnp, rodrigues};


///Caculate the inverse extrinsic matrix from an image
pub fn get_extrinsic_inv_from_aruco(filepath : &str, marker_ids : Vec<i32>,marker_coords : Vec<[f32; 3]>,  marker_type : PredefinedDictionaryType, intrinsic_info: &IntrinsicInfo) -> Result<Matrix4<f32>, anyhow::Error>{

    //Estimate the pose from the image
    let (rvec, tvec) = estimate_pose_from_aruco(filepath, marker_ids, marker_coords, marker_type, intrinsic_info)?;

    //Calculate the extrinsic matrix from the rotation and translation vector
    let extrinsic = calc_extrinsic(rvec, tvec)?;

    //Invert the extrinsic and return--inverse guaranteed as square matrix
    Ok(extrinsic.try_inverse().unwrap())
}

///Estimates the translation and rotation from an image with aruco tags in
pub fn estimate_pose_from_aruco(filepath : &str, marker_ids : Vec<i32>,marker_coords : Vec<[f32; 3]>, marker_type : PredefinedDictionaryType, intrinsic_info: &IntrinsicInfo) -> Result<(Vector::<f32>, Vector::<f32>), anyhow::Error> {

     //Load the aruco dictionary
     let aruco_dict = get_predefined_dictionary(marker_type)?;
     let aruco_detector = ArucoDetector::new(&aruco_dict, &DetectorParameters::default()?, RefineParameters::new_def()?)?;


    //Load the image in grayscale
    let image = imread(filepath, IMREAD_GRAYSCALE)?;


    //Detect aruco tags
    let mut corners = Vector::<Vector<Point2f>>::new(); 
    let mut ids  = Vector::<i32>::new();
    aruco_detector.detect_markers_def(&image, &mut corners, &mut ids)?;

    //Check that enough tags were spotted
    if ids.len() < marker_ids.len(){
        bail!("Not enough ids spotted")
    }

    //Create the object/image point pairs
    let mut object_points = Vector::<Point3f>::new();
    let mut image_points = Vector::<Point2f>::new();

    //Go through each of the detected ids
    for (i, id) in ids.iter().enumerate(){
        //Check if the detected id is in the list of marker ids
        if marker_ids.contains(&id){

            println!("Marker {:?} detected", id);

            //Get the marker id index - guaranteed to be in the array as we already checked
            let marker_index = marker_ids.iter().position(|&x| x == id).unwrap();            
            //Add the marker center            
            object_points.push(Point3f::new(marker_coords[marker_index][0], marker_coords[marker_index][1], marker_coords[marker_index][2]));

            //Add the detected marker center
            let detected_center_x = (corners.get(i)?.get(0)?.x + corners.get(i)?.get(1)?.x + corners.get(i)?.get(2)?.x +corners.get(i)?.get(2)?.x)/4;
            let detected_center_y = (corners.get(i)?.get(0)?.y + corners.get(i)?.get(1)?.y + corners.get(i)?.get(2)?.y +corners.get(i)?.get(2)?.y)/4;
            println!("{:?}", detected_center);
            image_points.push(Point2f::new(detected_center_x, detected_center_y));   
        }
    }

    
    //Estimate the pose from the aruco tags
    let mut rvec = Vector::<f32>::new();
    let mut tvec = Vector::<f32>::new();
    solve_pnp(&object_points, &image_points, &intrinsic_to_opencv_mat(intrinsic_info), &Vector::<f32>::new(), &mut rvec, &mut tvec, false, 3)?;

    Ok((rvec, tvec))
}

///Generate an extrinsic transofmration matrix for rvec and tvec
pub fn calc_extrinsic(rvec : Vector::<f32>, tvec : Vector::<f32>) -> Result<Matrix4<f32>, anyhow::Error>{


    //Calculate the rotation matrix from rvec
    let mut rot_mat = Mat::default();
    let mut jacobian = Mat::default();

    rodrigues(&rvec, &mut rot_mat, &mut jacobian)?;

    println!("{:?}", rot_mat);

    //Create the base matrix
    let mut ext_matrix = Matrix4::new(0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0);

    //Input the rotation matrix into the transformation matrix
    ext_matrix.m11 = *rot_mat.at_2d_mut::<f32>(0,0).unwrap();
    ext_matrix.m12 = *rot_mat.at_2d_mut::<f32>(0,1).unwrap();
    ext_matrix.m13 = *rot_mat.at_2d_mut::<f32>(0,2).unwrap();
    ext_matrix.m21 = *rot_mat.at_2d_mut::<f32>(1,0).unwrap();
    ext_matrix.m22 = *rot_mat.at_2d_mut::<f32>(1,1).unwrap();
    ext_matrix.m23 = *rot_mat.at_2d_mut::<f32>(1,2).unwrap();
    ext_matrix.m31 = *rot_mat.at_2d_mut::<f32>(2,0).unwrap();
    ext_matrix.m32 = *rot_mat.at_2d_mut::<f32>(2,1).unwrap();
    ext_matrix.m33 = *rot_mat.at_2d_mut::<f32>(2,2).unwrap();

    //Input the translation vector into the transformation matrix
    ext_matrix.m14 = tvec.get(0).unwrap();
    ext_matrix.m24 = tvec.get(1).unwrap();
    ext_matrix.m34 = tvec.get(2).unwrap();
    ext_matrix.m44 = 1.0;
    

    Ok(ext_matrix)
}


///Wrapper to allow TRL subsystem controller to access without use of rust-openCV crates
pub fn get_extrinsic_inv_from_aruco_4x4_250(filepath : &str, marker_ids : Vec<i32>,marker_coords : Vec<[f32; 3]>, intrinsic_info: &IntrinsicInfo) -> Result<Matrix4<f32>, anyhow::Error>{

   get_extrinsic_inv_from_aruco(filepath, marker_ids ,marker_coords, PredefinedDictionaryType::DICT_4X4_250, intrinsic_info)
}

///Convert intrinsic info to a opencv matrix
fn intrinsic_to_opencv_mat(intrinsic : &IntrinsicInfo) -> Mat{

        //Define the empty matrix -- 5 is the f32 type
         let mut mat = unsafe{
            Mat::new_rows_cols(3, 3, 5).unwrap()  
        };


        //Set the opencv matrix values
        *mat.at_2d_mut::<f32>(0, 0).unwrap() = intrinsic.focal_length_x();
        *mat.at_2d_mut::<f32>(0, 1).unwrap() = intrinsic.skew();
        *mat.at_2d_mut::<f32>(0, 2).unwrap() = intrinsic.principal_off_x();

        *mat.at_2d_mut::<f32>(1, 1).unwrap() = intrinsic.focal_length_y();
        *mat.at_2d_mut::<f32>(1, 2).unwrap() = intrinsic.principal_off_y();

        *mat.at_2d_mut::<f32>(2, 2).unwrap() = 1.0;
                
        mat
}