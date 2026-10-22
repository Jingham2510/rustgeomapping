/*
A kalman filtered heightmap

The intended use is:
Global heightmap insertion -> The global heightmap will have a confidence representation and when a new local heightmap is inserted - it will not necessarily override previous measurements


Design notes:
- for the global heightmap insertion it will be important to include a high process noise (Q) locally so that terrain modifications aren't ignored


*/

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
            let (cells, variance) = UncHeightmap::trans_to_heightmap(base_pcl.consume_points(), unc_list,
                                    bins_per_row, bins_per_col, total_width, total_height, bounds[0], bounds[2])?;

            let hmap = Heightmap::create_from_cells(cells, &bounds[0..4], bins_per_row, bins_per_col);

            
            Ok(Self{
                hmap,
                variance
            }
        )
        }
    }


    ///Transforms 3 point data into a 2.5d heightmap ( where the 3rd data point is the "height"/intensity)
    /// Consider  the uncertainties provided by unc_list (which are already weights)
    fn trans_to_heightmap(
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

                    top += (pnt * uncertainties_list[x][y][i]);

                    var_top += (pnt.powi(2) * uncertainties_list[x][y][i]);

                    bottom += (uncertainties_list[x][y][i]); 

                    cnt += 1;                 
                }
                let val_to_add = top/bottom;



                cells[x][y] = val_to_add;

                //If a single value there is no valid variance
                if cnt == 0 || cnt == 1{
                    variance[x][y] = f32::NAN;
                }else{
                    variance[x][y] = ((var_top/bottom) - val_to_add.powi(2)) * (cnt as f32/(cnt as f32 - 1.0));
                }
                

            }
        }

        Ok((cells, variance))
    }



}