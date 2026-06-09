///A handful of useful computer vision algorithms for image processing using the rust opencv wrapper
use nalgebra::{Matrix4, Matrix3};
use anyhow::Error;

use crate::data_types::intrinsic_info::IntrinsicInfo;

use opencv::prelude::*;
use opencv::objdetect::{ArucoDetector, PredefinedDictionaryType, get_predefined_dictionary, DetectorParameters, RefineParameters};
use opencv::imgcodecs::{imread, IMREAD_GRAYSCALE};
use opencv::core::{Point2f, Point3f,Vector};
use opencv::calib3d::solve_pnp;


///Caculate the inverse extrinsic matrix from an image
pub fn get_extrinsic_inv_from_aruco(filepath : &str, board_size : f32, marker_ids : Vec<i32>,marker_coords : Vec<[f32; 3]>,  marker_type : PredefinedDictionaryType, intrinsic_info: IntrinsicInfo) -> Result<Matrix4<f32>, anyhow::Error>{

    //Estimate the pose from the image
   

    //Calculate the extrinsic matrix from the rotation and translation vector

    //Invert the extrinsic and return

    todo!()
}

///Estimates the translation and rotation from an image with aruco tags in
pub fn estimate_pose_from_aruco(filepath : &str, marker_ids : Vec<i32>,marker_coords : Vec<[f32; 3]>, marker_type : PredefinedDictionaryType) -> Result<(), anyhow::Error>{

     //Load the aruco dictionary
     let aruco_dict = get_predefined_dictionary(marker_type)?;
     let aruco_detector = ArucoDetector::new(&aruco_dict, &DetectorParameters::default()?, RefineParameters::new_def()?)?;


    //Load the image in greyscale format
    let mut image = imread(filepath, IMREAD_GRAYSCALE)?;

    //Detect aruco tags
    let mut corners = Vector::<Vector<Point2f>>::new(); 
    let mut ids  = Vector::<i32>::new();
    let mut rejected_points= Vector::<Point2f>::new();;
    aruco_detector.detect_markers(&image, &mut corners, &mut ids, &mut rejected_points);


    //Create the object/image point pairs
    let mut object_points = Vector::<Point3f>::new();
    let mut image_points = Vector::<Point2f>::new();

    //Go through each of the detected ids
    for (i, id) in ids.iter().enumerate(){
        //Check if the detected id is in the list of marker ids
        if marker_ids.iter().any(|x| *x == id){

            println!("Marker {:?} detected", id);

            //Get the marker id index - guaranteed to be in the array as we already checked
            let marker_index = marker_ids.iter().position(|&x| x == id).unwrap();            
            //Add the marker center            
            object_points.push(Point3f::new(marker_coords[marker_index][0], marker_coords[marker_index][1], marker_coords[marker_index][2]));

            //Add the detected marker center
            let detected_center = corners.get(i)?.get(0)?;
            image_points.push(detected_center)   
        }
    }
    
    //Estimate the pose from the aruco tags



    todo!()
}
