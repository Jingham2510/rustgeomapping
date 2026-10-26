/*
A kalman filtered heightmap

The intended use is:
Global heightmap insertion -> The global heightmap will have a confidence representation and when a new local heightmap is inserted - it will not necessarily override previous measurements


Design notes:
- for the global heightmap insertion it will be important to include a high process noise (Q) locally so that terrain modifications aren't ignored


*/

use std::{fs::File, io::Write};

use crate::data_types::{heightmap::Heightmap, pointcloud::PointCloud};
use anyhow::{Error, bail};

struct UncHeightmap{
    //The heightmap
    hmap : Heightmap,
    //The variance mapping
    variance : Vec<Vec<f32>>
}


impl UncHeightmap{
     ///Creates a heightmap with a desired bin size (square) 
    /// Also creates an associated variance map
    pub fn create_from_list(
        pcl_list: Vec<PointCloud>,
        unc_list : Vec<f32>,
        desired_bin_size: f32,
    ) -> Result<UncHeightmap, anyhow::Error> {
        let len = pcl_list.len();

        if len < 2 {
            bail!("Not enough pointclouds to transform");
        }else {

            //Combine the pointclouds into one pointcloud
            let mut base_pcl = PointCloud::new();

            //Take the pointclouds from the list - consuming the pointclouds
            for pcl in pcl_list.into_iter() {
                base_pcl.combine(pcl)
            }

            //Get the bounds of the pointcloud
            let bounds = base_pcl.bounds();
            let bins_per_row: usize = ((bounds[1] - bounds[0]) / desired_bin_size) as usize;

            let bins_per_col: usize = ((bounds[3] - bounds[2]) / desired_bin_size) as usize;

            //Calculate the real distance and height of the heightmap
            let total_width = bounds[1] - bounds[0];
            let total_height = bounds[3] - bounds[2];

            //Calculate the cells
            let (cells, variance) = trans_to_heightmap_w_variance(base_pcl.consume_points(), unc_list,
                                    bins_per_row, bins_per_col, total_width, total_height, bounds[0], bounds[2])?;

            let hmap = Heightmap::create_from_cells(cells, &bounds[0..4], bins_per_row, bins_per_col);

            
            Ok(Self{
                hmap,
                variance
            }
        )
        }
    }


    ///Updates a section of the heightmap by slotting in another heightmap
    /// Assumes that the other heightmap has the same resolution
    pub fn kalman_update_section(&mut self, other: UncHeightmap) -> Result<(), anyhow::Error> {


        let self_upper_bnds = self.hmap.upper_coord_bounds();
        let self_lower_bnds = self.hmap.lower_coord_bounds();
        let other_upper_bnds = other.hmap.upper_coord_bounds();
        let other_lower_bnds = other.hmap.lower_coord_bounds();

        //Check that it is feasible for the other heightmap to sit on the main heightmap
        if self_upper_bnds[0] < other_lower_bnds[0]
            || self_lower_bnds[0] > other_upper_bnds[0]
            || self_lower_bnds[1] > other_upper_bnds[1]
            || self_upper_bnds[1] <other_lower_bnds[1]
        {
            println!("out of bounds");
            bail!("Other heightmap does not sit on main!")
        }

        //--Locate bin in main that corresponds to first bin in other (can be negative - i.e. it starts outside)--
        //Get bin resolution of main
        let bin_res_width =
            (self_upper_bnds[0] - self_lower_bnds[0]) / self.hmap.width() as f32;
        let bin_res_height =
            (self_upper_bnds[1] - self_lower_bnds[1]) / self.hmap.height() as f32;

        let start_col = ((other_lower_bnds[0] - self_lower_bnds[0]) / bin_res_width).round() as i32;
        let start_row = ((other_lower_bnds[1] - self_lower_bnds[1]) / bin_res_height).round() as i32;


        //Calculate the center point of the new heightmap
        let x_centre = (start_col  as f32 + (other.hmap.width() as f32 /2.0)) as usize;
        let y_centre = (start_row  as f32 + (other.hmap.height() as f32 /2.0)) as usize;


        //Go through each bin and and update the corresponding bin using a kalman filter approach
         for x in start_col..(start_col + other.hmap.width() as i32) {
                if x < 0 || x >= self.hmap.width() as i32 {
                    continue;
                }
            for y in start_row..(start_row + other.hmap.height() as i32) {
                if y < 0 || y >= self.hmap.height() as i32 {
                    continue;
                }


                //KALMAN FILTER APPROACH----------               
                let curr_val = self.hmap.cells()[x as usize][y as usize];
                let curr_var = self.variance[x as usize][y as usize];
                   
                let other_val = other.hmap.cells()[(x - start_col) as usize][(y - start_row) as usize];
                let other_var = other.variance[(x - start_col) as usize][(y - start_row) as usize];

                //Baseline Q value
                const Q_UPD : f32 = 0.002;

                //Calculate a noise field for likelihood of terrain change (the closer to the centre the more likely the change)
                let q = Q_UPD / ((x - x_centre as i32).pow(2) + (y - y_centre as i32).pow(2)).pow(2) as f32;



                if let Some((posterior_height, posterior_var)) = UncHeightmap::kalman_cell(curr_val, curr_var, other_val, other_var, q){
                    self.hmap.set_cell_height(x as usize, y as usize, posterior_height)?;
                    self.variance[x as usize][y as usize] = posterior_var;
                };
             
              
                
            }
        }
        //Check that the bin doesnt reside outside the heightmap area

        Ok(())
    }

    fn kalman_cell(curr_val: f32, curr_var: f32, other_val: f32, other_var: f32, q : f32) -> Option<(f32, f32)> {
        if other_val.is_nan() || !other_var.is_finite() || other_var <= 0.0 {
            return None;
        }
        // Unobserved cell (or unusable variance): take the measurement directly
        if curr_val.is_nan() || !curr_var.is_finite() {
            return Some((other_val, other_var));
        }        
        //Update the priori variation with a provided q (random walk or otherwise)
        let prior_var = curr_var + q;
        //Calculate the kalman gain
        let k = prior_var / (prior_var + other_var);
        //Calculate the posterior height and the posterior variation
        Some((curr_val + k * (other_val - curr_val), (1.0 - k) * prior_var))
    }


    ///Saves the heightmap to a text file and the variance map to a seperate text file
    pub fn save_to_file(&self, filepath: &str) -> Result<(), anyhow::Error> {

        //Save the heightmap
        self.hmap.save_to_file(filepath);

        //Save the variance heightmap in the same way

        //Create a file
        let mut file = File::create(filepath.to_owned() + "_var.txt")?;

        //Format the first line to store the real bounds
        let first_line = format!(
            "bnds(low_point,upper_point):[{},{}][{},{}]\n",
            self.hmap.lower_coord_bounds()[0],
            self.hmap.lower_coord_bounds()[1],
            self.hmap.upper_coord_bounds()[0],
            self.hmap.upper_coord_bounds()[1]
        );
        file.write_all(first_line.as_bytes())?;

        let mut mat_string = String::from("");

        //Iterate thorugh each row
        for row in self.variance.iter() {
            let mut row_string = String::new();

            for var in row {
                row_string.push_str(&format!("{},", var));
            }

            row_string.push('\n');

            mat_string.push_str(&row_string);
        }

        file.write_all(mat_string.as_bytes())?;

        Ok(())
    }
    



}


///Transforms 3 point data into a 2.5d heightmap ( where the 3rd data point is the "height"/intensity)
    /// Consider  the  variances provided by unc_list - which are assumed to all be positive and non-zero
    fn trans_to_heightmap_w_variance(
        points: Vec<[f32; 3]>,
        unc_list : Vec<f32>,
        bins_per_row: usize,
        bins_per_col: usize,
        total_width: f32,
        total_height: f32,
        min_x_bnd: f32,
        min_y_bnd: f32,
    ) -> Result<(Vec<Vec<f32>>, Vec<Vec<f32>>), anyhow::Error> {
        //Create the empty cell matrix
        //NaN spots are areas with no information
        let mut cells_pnt_list = vec![vec![vec![]; bins_per_col]; bins_per_row];
        let mut uncertainties_list = vec![vec![vec![]; bins_per_col]; bins_per_row];

        let width_resolution = total_width / bins_per_row as f32;
        let height_resolution = total_height / bins_per_col as f32;

        //Bin each cell
        for (i, pnt) in points.iter().enumerate() {

            let mut n = ((pnt[0] - min_x_bnd) / width_resolution).floor() as usize;
            let mut m = ((pnt[1] - min_y_bnd) / height_resolution).floor() as usize;

            //Nudge the edges in
            if n == bins_per_row{
                n = n - 1;
            }
            if m == bins_per_col {
                m = m - 1;
            }

            //add the point to the cell point list
            cells_pnt_list[n][m].push(pnt[2]);
            uncertainties_list[n][m].push(unc_list[i])

        }

        //Calculate the height of each cell based on the chosen hmap option
        let mut cells: Vec<Vec<f32>> = vec![vec![f32::NAN; bins_per_col]; bins_per_row];
        let mut variance: Vec<Vec<f32>> = vec![vec![f32::NAN; bins_per_col]; bins_per_row];

        //Iterate through each point list
        for (x, row) in cells_pnt_list.iter_mut().enumerate() {
            for (y, pnts) in row.iter_mut().enumerate(){
                //If the cell is NAN - keep it as a null cell
                if pnts.iter().all(|pnt| pnt.is_nan()) || pnts.is_empty() {
                    cells[x][y] = f32::NAN;
                    variance[x][y] = f32::NAN;
                    continue;
                }

                let mut top = 0.0;
                let mut var_top = 0.0;
                let mut bottom = 0.0;

                let mut cnt = 0;

                //Calculate the weighting of each point (for the variance also)
                //Note that this will produce a biased variance if one point has a particularly high uncertainty
                for (i, pnt) in pnts.iter().enumerate(){

                    if pnt.is_nan(){
                        continue;
                    }
                    top += (pnt / uncertainties_list[x][y][i]);

                    bottom += ( 1.0 / uncertainties_list[x][y][i]); 

                    cnt += 1;                 
                }
                let val_to_add = top/bottom;



                cells[x][y] = val_to_add;

                //No variance for cells of no points
                if cnt == 0{
                    variance[x][y] = f32::NAN;
                }else{
                    variance[x][y] = 1.0/bottom;
                }
                

            }
        }

        Ok((cells, variance))
    }