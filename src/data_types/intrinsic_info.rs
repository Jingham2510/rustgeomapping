///Intrinsic information related to camera distortion
use nalgebra::Matrix3;
use opencv::core::{Mat, MatTrait};


///Intrinsic camera information
#[derive(Debug)]
pub struct IntrinsicInfo{
    focal_length_x : f32,
    focal_length_y : f32,
    principal_off_x : f32,
    principal_off_y : f32,
    skew : f32,    
}

impl IntrinsicInfo{
    ///Create an intrinsic info structure
    pub fn create(focal_length_x : f32, focal_length_y : f32, principal_off_x : f32, principal_off_y : f32, skew : f32) -> Self{
        IntrinsicInfo { focal_length_x, focal_length_y, principal_off_x, principal_off_y, skew }
    }

    ///Get the focal length in the x axis
    pub fn focal_length_x(&self) -> f32{
        self.focal_length_x
    }
    ///Get the focal length in the y axis
    pub fn focal_length_y(&self) -> f32{
        self.focal_length_y
    }
    ///Get the principal offset in the x axis
    pub fn principal_off_x(&self) -> f32{
        self.principal_off_x
    }
    ///Get the principal offset in the y axis
    pub fn principal_off_y(&self) -> f32{
        self.principal_off_y
    }

    ///Get the skew of the robot
    pub fn skew(&self) -> f32{
        self.skew
    }


    ///Turn the intrinsic information into a nalgebra matrix
    pub fn as_matrix(&self) ->  Matrix3<f32>{
        Matrix3::new(self.focal_length_x, self.skew, self.principal_off_x,
                            0.0, self.focal_length_y, self.principal_off_y,
                            0.0, 0.0, 1.0)
    }


    ///Get the intrinsic information as a opencv format matrix
    pub fn as_opencv_mat(&self) -> Mat{

        //Define the empty matrix -- 5 is the f32 type
         let mut mat = unsafe{
            Mat::new_rows_cols(3, 3, 5).unwrap()  
        };


        //Set the opencv matrix values
        *mat.at_2d_mut::<f32>(0, 0).unwrap() = self.focal_length_x;
        *mat.at_2d_mut::<f32>(0, 1).unwrap() = self.skew;
        *mat.at_2d_mut::<f32>(0, 2).unwrap() = self.principal_off_x;

        *mat.at_2d_mut::<f32>(1, 1).unwrap() = self.focal_length_y;
        *mat.at_2d_mut::<f32>(1, 2).unwrap() = self.principal_off_y;

        *mat.at_2d_mut::<f32>(2, 2).unwrap() = 1.0;
                
        mat
    }

}
