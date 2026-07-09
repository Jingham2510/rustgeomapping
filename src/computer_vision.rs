///A handful of useful computer vision algorithms for image processing using the rust opencv wrapper
use nalgebra::Matrix4;
use anyhow::bail;

use crate::data_types::intrinsic_info::IntrinsicInfo;

use opencv::prelude::*;
use opencv::objdetect::{ArucoDetector, PredefinedDictionaryType, get_predefined_dictionary, DetectorParameters, RefineParameters, draw_detected_markers, Board};
use opencv::imgcodecs::{imread, IMREAD_GRAYSCALE, imwrite, ImwriteFlags};
use opencv::core::{Point2i, Point2f, Point3f, Vector, Mat, MatTrait, Scalar, VecN};
use opencv::calib3d::{solve_pnp, rodrigues, draw_frame_axes};



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


    //Create the corner objects
    let marker_size = 0.29 /2.0;
    let mut marker_corners = Vector::<Vector<Point3f>>::new();
    for coord in marker_coords{
        let mut marker = Vector::<Point3f>::new();       


        marker.push(Point3f::new(coord[0] + marker_size, coord[1] + marker_size, coord[2]));
        marker.push(Point3f::new(coord[0] + marker_size, coord[1] - marker_size, coord[2]));

        marker.push(Point3f::new(coord[0] - marker_size, coord[1] - marker_size, coord[2]));

        marker.push(Point3f::new(coord[0] - marker_size, coord[1] + marker_size, coord[2]));
        


        marker_corners.push(marker);
    }

    //Create the id opencv object
    let mut marker_id_vec = Vector::<i32>::new();
    for id in marker_ids{
        marker_id_vec.push(id);
    }


    //Create the board object
    let board = Board::new(&marker_corners, &aruco_dict, &marker_id_vec)?;


    //Load the image in grayscale
    let mut image = imread(filepath, IMREAD_GRAYSCALE)?;


    //Detect aruco tags
    let mut corners = Vector::<Vector<Point2f>>::new(); 
    let mut ids  = Vector::<i32>::new();
    aruco_detector.detect_markers_def(&image, &mut corners, &mut ids)?;

    //Check that enough tags were spotted
    if ids.len() < marker_id_vec.len(){
        bail!("Not enough ids spotted")
    }

    //Create the object/image point pairs
    let mut object_points = Vector::<Point3f>::new();
    let mut image_points = Vector::<Point2f>::new();

    //Match up the board and image points
    board.match_image_points(&corners, &ids, &mut object_points, &mut image_points);
  


    
    //Estimate the pose from the aruco tags
    let mut rvec = Vector::<f32>::new();
    let mut tvec = Vector::<f32>::new();

    solve_pnp(&object_points, &image_points, &intrinsic_to_opencv_mat(intrinsic_info), &Vector::<f32>::new(), &mut rvec, &mut tvec, false, 4)?;



    
    //Draw the detected markers
    draw_detected_markers(&mut image, &corners, &ids, VecN::new(256.0, 256.0, 0.0, 0.0));

    //Draw the estimate frame axes
    draw_frame_axes( &mut image, &intrinsic_to_opencv_mat(intrinsic_info),  &Vector::<f32>::new(), &rvec, &tvec, 0.5, 2);


      //Save the modified image
    imwrite(filepath, &image, &Vector::<i32>::new());

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

        *mat.at_2d_mut::<f32>(1, 0).unwrap() = 0.0;
        *mat.at_2d_mut::<f32>(1, 1).unwrap() = intrinsic.focal_length_y();
        *mat.at_2d_mut::<f32>(1, 2).unwrap() = intrinsic.principal_off_y();

        *mat.at_2d_mut::<f32>(2, 0).unwrap() = 0.0;
        *mat.at_2d_mut::<f32>(2, 1).unwrap() = 0.0;
        *mat.at_2d_mut::<f32>(2, 2).unwrap() = 1.0;
                
        mat
}