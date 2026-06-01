///A set of tools around creating/manipulating/combining 2.5D heightmap objects
use crate::data_types::pointcloud::PointCloud;
use anyhow::bail;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
///Heightmap structure - contains the size and height information for each cell
#[derive(Clone)]
pub struct Heightmap {
    ///Number of pixel rows
    height: usize,
    ///Number of pixel columns
    width: usize,

    ///The number of cells present in the heightmap
    pub no_of_cells: u32,

    ///The point defined minimum cell height
    min: f32,
    ///The point defined maximum cell height
    max: f32,

    ///State that determines whether the minimum height needs to be checked
    min_updated: bool,
    ///State that determines whether the maximum height needs to be checked
    max_updated: bool,

    ///Cell location at minimum height
    min_pos: (usize, usize),
    ///Cell location at maximum height
    max_pos: (usize, usize),

    ///the 2d vector representing the cells
    cells: Vec<Vec<f32>>,

    ///The pointcloud real-world lower bounds the hieghtmap is constructed from
    lower_coord_bounds: [f32; 2],
    ///The pointcloud real-world upper bounds the hieghtmap is constructed from
    upper_coord_bounds: [f32; 2],

    ///Store the filepath for usage in analysis
    pub filename: String,
}

impl Default for Heightmap {
    ///Default heightmap is a blank 250x250
    fn default() -> Heightmap {
        Heightmap::new(250, 250)
    }
}

//Map tools - including display and modification etc
impl Heightmap {
    ///Create a new empty heightmap
    pub fn new(width: usize, height: usize) -> Self {
        //No filename as is a new object
        let filename = "No filepath".to_string();

        //Generate an empty cell bed of height*width size
        Self {
            height,
            width,
            no_of_cells: height as u32 * width as u32,
            cells: vec![vec![0.0; height]; width],
            min: 999.0,
            max: -999.0,
            min_updated: false,
            max_updated: false,
            min_pos: (0, 0),
            max_pos: (0, 0),
            lower_coord_bounds: [0.0, 0.0],
            upper_coord_bounds: [0.0, 0.0],
            filename,
        }
    }

    ///Create a heightmap from a pointcloud (takes ownership of pcl object)
    pub fn create_from_pcl(
        pcl: PointCloud,
        width: usize,
        height: usize,
    ) -> Result<Self, anyhow::Error> {
        //create the filename from the relative timestamp
        let filename = format!("created from pcl- {}", pcl.timestamp());

        //Get the bounds
        let bounds = pcl.bounds();

        //Calculate the real distance and height of the heightmap
        let total_width = bounds[1] - bounds[0];
        let total_height = bounds[3] - bounds[2];

        if let Ok(cells) = trans_to_heightmap(
            pcl.consume_points(),
            width,
            height,
            total_width,
            total_height,
            bounds[0],
            bounds[2],
            MapGenOpt::Mean,
        ) {
            Ok(Self {
                height,
                width,
                no_of_cells: height as u32 * width as u32,
                min: 999.0,
                max: -999.0,
                min_updated: false,
                max_updated: false,
                min_pos: (0, 0),
                max_pos: (0, 0),
                cells,
                lower_coord_bounds: [bounds[0], bounds[2]],
                upper_coord_bounds: [bounds[1], bounds[3]],
                filename,
            })
        } else {
            bail!("Failed to make height map from pcl")
        }
    }

    ///Create a heightmap from a pointcloud without consuming it
    pub fn create_from_pcl_ref(
        pcl: &PointCloud,
        width: usize,
        height: usize,
    ) -> Result<Self, anyhow::Error> {

        //create the filename from the relative timestamp
        let filename = format!("created from pcl- {}", pcl.timestamp());

        //Get the bounds
        let bounds = pcl.bounds();

        //Calculate the real distance and height of the heightmap
        let total_width = bounds[1] - bounds[0];
        let total_height = bounds[3] - bounds[2];

        if let Ok(cells) = trans_to_heightmap(
            pcl.points(),
            width,
            height,
            total_width,
            total_height,
            bounds[0],
            bounds[2],
            MapGenOpt::Mean,
        ) {
            Ok(Self {
                height,
                width,
                no_of_cells: height as u32 * width as u32,
                min: 999.0,
                max: -999.0,
                min_updated: false,
                max_updated: false,
                min_pos: (0, 0),
                max_pos: (0, 0),
                cells,
                lower_coord_bounds: [bounds[0], bounds[2]],
                upper_coord_bounds: [bounds[1], bounds[3]],
                filename,
            })
        } else {
            bail!("Failed to make height map from pcl reference")
        }
    }


    ///Create from a group of point clouds
    pub fn create_from_pcl_list(mut pcl_list: Vec<PointCloud>,
        width: usize,
        height: usize) -> Result<Self, anyhow::Error>{

            let len = pcl_list.len();

            if len == 0{
                bail!("No pointclouds to transform");
            }

            if len == 1{
                return Heightmap::create_from_pcl_ref(&pcl_list[0], width, height)
            }else{


                //Combine the pointclouds into one pointcloud
                let mut base_pcl = PointCloud::new();

                //Take the pointclouds from the list - consuming the pointclouds
                for pcl in pcl_list.into_iter(){
                    base_pcl.combine(pcl)
                }
                //Turn that pointcloud into a heightmap 
                return Heightmap::create_from_pcl(base_pcl, width, height)
      
            }



    }


    ///Creates a heightmap with a desired bin size (square) -- assume all pointclouds have been preprocessed and fit together
    pub fn create_from_pcl_list_with_res(mut pcl_list: Vec<PointCloud>, desired_bin_size : f32) -> Result<Heightmap, anyhow::Error>{

       
         let len = pcl_list.len();

            if len == 0{
                bail!("No pointclouds to transform");
            }

            if len == 1{

                let bnds = pcl_list[0].bounds(); 
                let bins_per_row : usize = ((bnds[1] - bnds[0])/desired_bin_size) as usize;

                let bins_per_col: usize = ((bnds[3] - bnds[2])/desired_bin_size) as usize;



                return Heightmap::create_from_pcl_ref(&pcl_list[0], bins_per_row, bins_per_col)
            }else{

                //Combine the pointclouds into one pointcloud
                let mut base_pcl = PointCloud::new();

                //Take the pointclouds from the list - consuming the pointclouds
                for pcl in pcl_list.into_iter(){
                    base_pcl.combine(pcl)
                }

                //Get the bounds of the pointcloud
                let bnds = base_pcl.bounds(); 
                let bins_per_row : usize = ((bnds[1] - bnds[0])/desired_bin_size) as usize;

                let bins_per_col : usize = ((bnds[3] - bnds[2])/desired_bin_size) as usize;


                //Turn that pointcloud into a heightmap
                return Heightmap::create_from_pcl(base_pcl, bins_per_row, bins_per_col);     
            }
    }

    ///Loads a hmap file and creates a heightmap from it
    pub fn create_from_file(filepath: String) -> Result<Self, anyhow::Error> {
        //Open the file and create a buffer to read the lines
        let file = File::open(filepath.clone())?;
        let mut line_reader = BufReader::new(file);

        //Read the first line to extract the bounds
        let mut first_line = String::new();
        line_reader.read_line(&mut first_line)?;

        let bounds = Self::extract_bounds(first_line);

        let mut bounds_res: [f32; 4] = [f32::NAN, f32::NAN, f32::NAN, f32::NAN];

        if bounds.is_ok() {
            bounds_res = bounds?
        }

        let mut height: usize = 0;
        let mut width: usize = 0;
        let mut width_set = false;

        let mut cells: Vec<Vec<f32>> = vec![];
        //Go through each cell and update the
        for line in line_reader.lines() {
            //Create the empty row
            let mut row: Vec<f32> = vec![];

            //Split via comma then iterate
            for token in line?.split(",") {
                if !token.is_empty() {
                    if !width_set {
                        width += 1;
                    }

                    row.push(token.parse::<f32>()?);
                }
            }

            if !width_set {
                width_set = true;
            }

            //Store the row
            cells.push(row);
            height += 1;
        }

        Ok(Self {
            height,
            width,
            no_of_cells: height as u32 * width as u32,
            min: 999.0,
            max: -999.0,
            min_updated: false,
            max_updated: false,
            min_pos: (0, 0),
            max_pos: (0, 0),
            cells,
            lower_coord_bounds: [bounds_res[0], bounds_res[1]],
            upper_coord_bounds: [bounds_res[2], bounds_res[3]],
            filename: filepath,
        })
    }

    ///Creates a heightmap from a given pcl file
    pub fn create_from_pcl_file(
        filepath: String,
        width: usize,
        height: usize,
    ) -> Result<Self, anyhow::Error> {
        //Load the pcl file
        let pcl = PointCloud::create_from_file(filepath)?;

        //Turn the pcl into a heightmap and return it
        Heightmap::create_from_pcl(pcl, width, height)
    }

    pub fn cells(&self) -> Vec<Vec<f32>> {
        self.cells.clone()
    }

    ///Print the value contained in each cell
    pub fn print_cells(&self) {
        for row in &self.cells {
            for cell in row {
                print!("{} ", cell);
            }
            println!();
        }
    }

    ///Get the height for a given cell
    pub fn get_cell_height(&self, x: usize, y: usize) -> Result<f32, anyhow::Error> {
        if x > self.width || y > self.height {
            bail!("Warning - attempting to read from cell that doesnt exist!");
        }

        Ok(self.cells[x][y])
    }

    ///Set the height of a given cell
    pub fn set_cell_height(
        &mut self,
        x: usize,
        y: usize,
        new_height: f32,
    ) -> Result<(), anyhow::Error> {
        if x > self.width || y > self.height {
            bail!("Warning - attempting to write to cell that doesnt exist!");
        }

        self.cells[x][y] = new_height;

        //Check that the cell doesnt store the min or the max
        if (x, y) == self.max_pos {
            self.get_max();
        } else if (x, y) == self.min_pos {
            self.get_min();
        } else if new_height > self.max {
            self.max = new_height;
            self.max_pos = (x, y);
        } else if new_height < self.min {
            self.min = new_height;
            self.min_pos = (x, y);
        } else {
            println!("{0} is not larger than {1}!", new_height, self.max);
        }

        Ok(())
    }

    ///Set the height of a given cell without checking for a new max/min
    pub fn set_cell_height_no_check(
        &mut self,
        x: usize,
        y: usize,
        new_height: f32,
    ) -> Result<(), anyhow::Error> {
        if x > self.width || y > self.height {
            bail!("Warning - attempting to write to cell that doesnt exist!");
        }

        self.cells[x][y] = new_height;

        Ok(())
    }

    pub fn lower_coord_bounds(&self) -> [f32; 2] {
        self.lower_coord_bounds
    }
    pub fn upper_coord_bounds(&self) -> [f32; 2] {
        self.upper_coord_bounds
    }

    pub fn set_lower_coord_bounds(&mut self, new: [f32; 2]) {
        self.lower_coord_bounds = new;
    }

    pub fn set_upper_coord_bounds(&mut self, new: [f32; 2]) {
        self.upper_coord_bounds = new;
    }

    ///Set the height of all cells
    pub fn set_map(&mut self, new_heights: Heightmap) {
        //Sweep through each cell and replace with the new map height
        for (x, row) in self.cells.iter_mut().enumerate() {
            for (y, col) in row.iter_mut().enumerate() {
                *col = new_heights.cells[x][y]
            }
        }

        self.max_updated = false;
        self.min_updated = false;

        self.get_max();
        self.get_min();
    }

    ///Modify the height of every cell by a specified value
    pub fn offset_map(&mut self, height_change: f32) {
        //Sweep through each cell and replace with the new map height
        for row in self.cells.iter_mut() {
            for col in row.iter_mut() {
                *col = add_nan_f32(*col, height_change);
            }
        }
        self.max_updated = false;
        self.min_updated = false;
    }

    ///Get the maximum cell distance
    fn get_max(&mut self) -> f32 {
        //Skip the first as they contain plenty of nans
        let mut skip_first_row = true;
        let mut skip_first_column = true;

        let mut max_pos: (usize, usize) = (0, 0);

        //Check whether a new maximum is required
        if !self.max_updated {
            let mut new_max: f32 = -999.0;

            //Check every value to see if its the largest
            for (i, row) in self.cells.iter_mut().enumerate() {
                if skip_first_row {
                    skip_first_row = false;
                    continue;
                }

                for (j, col) in row.iter_mut().enumerate() {
                    if skip_first_column {
                        skip_first_column = false;
                        continue;
                    }

                    if col > &mut new_max {
                        new_max = *col;
                        max_pos = (i, j);
                    }
                }
            }

            self.max = new_max;
            self.max_pos = max_pos;
            self.max_updated = true;
        }

        self.max
    }

    ///Get the minimum cell distance
    fn get_min(&mut self) -> f32 {
        //Check whether a new minimum calc is required
        if !self.min_updated {
            //Skip the first as they contain plenty of nans
            let mut skip_first_row = true;
            let mut skip_first_column = true;

            let mut new_min: f32 = 999.0;
            let mut min_pos: (usize, usize) = (0, 0);

            //Check every value to see if its the smallest
            for (i, row) in self.cells.iter_mut().enumerate() {
                if skip_first_row {
                    skip_first_row = false;
                    continue;
                }
                for (j, col) in row.iter_mut().enumerate() {
                    if skip_first_column {
                        skip_first_column = false;
                        continue;
                    }
                    //Skip 0.0 (nan values)
                    if *col == 0.0 {
                        continue;
                    }

                    if col < &mut new_min {
                        new_min = *col;
                        min_pos = (i, j);
                    }
                }
            }
            self.min = new_min;
            self.min_pos = min_pos;
            self.min_updated = true;
        }

        self.min
    }

    ///Returns the flattened cells (i.e. every single cell in a single vector)
    pub fn get_flattened_cells(&self) -> Result<Vec<f32>, anyhow::Error> {
        let mut cell_list: Vec<f32> = vec![];

        for x in 0..(self.width - 1) {
            for y in 0..(self.height - 1) {
                cell_list.push(self.cells[x][y]);
            }
        }

        Ok(cell_list)
    }

    ///Saves the heightmap to a text file
    pub fn save_to_file(&self, filepath: &str) -> Result<(), anyhow::Error> {
        //Create a file
        let mut file = File::create(filepath.to_owned() + ".txt")?;

        //Format the first line to store the real bounds
        let first_line = format!(
            "bnds(low_point,upper_point):[{},{}][{},{}]\n",
            self.lower_coord_bounds[0],
            self.lower_coord_bounds[1],
            self.upper_coord_bounds[0],
            self.upper_coord_bounds[1]
        );
        file.write_all(first_line.as_bytes())?;

        let mut mat_string = String::from("");

        //Iterate thorugh each row
        for row in self.cells.iter() {

            let mut row_string = format!("");

            for cell in row {
                row_string.push_str(&format!("{},", cell));
            }

            row_string.push_str("\n");

            mat_string.push_str(&row_string);
            
        }

        file.write_all(mat_string.as_bytes())?;

        Ok(())
    }

    ///Width getter
    pub fn width(&self) -> usize {
        self.width
    }
    ///Height getter
    pub fn height(&self) -> usize {
        self.height
    }

    ///Extracts the bounds from a string
    fn extract_bounds(bnd_line: String) -> Result<[f32; 4], anyhow::Error> {
        let mut bounds: [f32; 4] = [f32::NAN, f32::NAN, f32::NAN, f32::NAN];

        let mut bnd_cnt = 0;

        //Split the line into 3 sections
        let bnd_split = bnd_line.split("[");

        let mut ignore = true;

        //Second and third sections have the data we want
        for split in bnd_split {
            //Ignore the first split
            if ignore {
                ignore = false;
                continue;
            }

            //Tokenize the second and third section (also remove the final parentheses)
            let mut str = split.trim().to_string();
            str.pop();

            for token in str.split(",") {
                bounds[bnd_cnt] = token.parse()?;
                bnd_cnt += 1;
            }
        }

        Ok(bounds)
    }

    ///Calculates the middle coordinates of a given cell (based on the upper/lower bounds)
    pub fn calc_cell_mid_pnt(&self, n: usize, m: usize) -> Result<[f32; 2], anyhow::Error> {
        if n >= self.width {
            bail!("Error - out of width bounds!")
        }
        if m >= self.height {
            bail!("Error - out of height bounds!");
        }

        let mut mid_pnt: [f32; 2] = [f32::NAN, f32::NAN];

        let cell_width =
            (self.upper_coord_bounds[0] - self.lower_coord_bounds[0]).abs() / self.width as f32;
        let cell_height =
            (self.upper_coord_bounds[1] - self.lower_coord_bounds[1]).abs() / self.width as f32;

        //Calc the mid point with the offset
        mid_pnt[0] = (n as f32 * cell_width) + self.lower_coord_bounds[0];
        mid_pnt[1] = (m as f32 * cell_height) + self.lower_coord_bounds[1];

        Ok(mid_pnt)
    }

    ///Count and return the number of unknown cells in the heightmap
    pub fn no_of_nans(&self) -> u32 {
        let mut nan_cnt = 0;

        for col in self.cells.iter() {
            for cell in col.iter() {
                if cell.is_nan() {
                    nan_cnt += 1;
                }
            }
        }

        nan_cnt
    }

    ///Remove any NaN entries in the provided data matrix
    /// Achieved by interpolating the point as an average between every surrounding point
    /// Assumes a 3x3 kernel (does not account for equal or smaller data matrices)
    fn interpolate_nan(&mut self) {
        //Flags to indicate whether the data is at the edge of matrix
        let mut l_edge_flag = false;
        let mut r_edge_flag = false;
        let mut t_edge_flag = false;
        let mut b_edge_flag = false;

        //Go through every row
        for i in 0..self.height {
            if i == 0 {
                t_edge_flag = true;
            } else if i == self.height - 1 {
                b_edge_flag = true;
            } else {
                t_edge_flag = false;
                b_edge_flag = false;
            }

            //Go through every pixel in each row
            for j in 0..self.width {
                if !self.cells[i][j].is_nan() {
                    continue;
                }

                if j == 0 {
                    l_edge_flag = true;
                } else if j == self.width - 1 {
                    r_edge_flag = true;
                } else {
                    l_edge_flag = false;
                    r_edge_flag = false;
                }

                let mut total = 0.0;
                let mut cnt = 0;

                //Check the top row
                if !t_edge_flag {
                    total = add_nan_f32(total, self.cells[i - 1][j]);
                    cnt += 1;

                    if !l_edge_flag {
                        total = add_nan_f32(total, self.cells[i - 1][j - 1]);
                        cnt += 1;
                    }
                    if !r_edge_flag {
                        total = add_nan_f32(total, self.cells[i - 1][j + 1]);
                        cnt += 1;
                    }
                }
                //Check the middle row
                if !l_edge_flag {
                    total = add_nan_f32(total, self.cells[i][j - 1]);
                    cnt += 1;
                }
                if !r_edge_flag {
                    total = add_nan_f32(total, self.cells[i][j + 1]);
                    cnt += 1;
                }

                //check the bottom row
                if !b_edge_flag {
                    total = add_nan_f32(total, self.cells[i + 1][j]);
                    cnt += 1;
                    if !l_edge_flag {
                        total = add_nan_f32(total, self.cells[i + 1][j - 1]);
                        cnt += 1;
                    }
                    if !r_edge_flag {
                        total = add_nan_f32(total, self.cells[i + 1][j + 1]);
                        cnt += 1;
                    }
                }

                self.cells[i][j] = total / (cnt as f32);
            }
        }
    }

    ///Calculates a simple estimation of a feature in a heightmap
    /// Returned as (depth, width, (slope median, slope mode))
    /// Assumes only two edges (i.e. a simple trough)
    /// Assumes that the deep point is in the middle
    pub fn calc_simple_feature(&mut self) -> Result<(f32, f32, f32), anyhow::Error> {
        //Calculate the bin size -- Assuming square so only need one set of bounds
        let bin_size =
            (self.upper_coord_bounds[0] - self.lower_coord_bounds[0]) / (self.width as f32 - 1.0);

        //Get the lowest point and its row (which is actually the max - furhter away from heightmap)
        if !self.max_updated {
            self.get_max();
            self.get_min();
        }
        let max_row = self.max_pos.0;

        //Load the row - CHECKED TO BE CORRECT USING PREGENNED HEIGHTMAPS
        let max_row = self.cells[max_row].clone();

        //Take edge point as highest and use to calculate depth of feature
        const EDGE_THRESHOLD: f32 = 0.0005;
        const EDGE_HIGH_THRESHHOLD: f32 = 0.01;
        let mut first_edge: usize = 0;
        let mut second_edge: usize = 0;

        //Compare a cell to see if it meets the threshold
        for i in 0..(self.width() - 1) {
            //ignoring 0.0 as they are NaN
            if max_row[i] == 0.0 || max_row[i + 1] == 0.0 {
                continue;
            }

            //REMEMBER: An increase in height means an increase in distance from the camera (i.e. going down)
            if (max_row[i + 1] - max_row[i]) >= EDGE_THRESHOLD
                && (max_row[i + 1] - max_row[i]) < EDGE_HIGH_THRESHHOLD
            {
                first_edge = i;
                break;
            }

            if i == (self.width() - 1) {
                bail!("Failed to find first edge in hmap")
            }
        }
        //Go backwards to find the edge on the other side
        for i in (1..(self.width())).rev() {
            //ignoring 0.0 as they are NaN
            if max_row[i] == 0.0 || max_row[i - 1] == 0.0 {
                continue;
            }

            if (max_row[i - 1] - max_row[i]) >= EDGE_THRESHOLD
                && (max_row[i - 1] - max_row[i]) < EDGE_HIGH_THRESHHOLD
            {
                second_edge = i;
                break;
            }
        }

        if first_edge == second_edge {
            bail!("Only one edge detected!")
        }

        if self.max_pos.0 > second_edge || self.max_pos.0 < first_edge {
            bail!("Invalid edge setup - deepest point outside edges")
        }

        //Calculate the height by doing (edge bin to deepest bin)
        let highest_edge = if max_row[first_edge] > max_row[second_edge] {
            max_row[first_edge]
        } else {
            max_row[second_edge]
        };
        let depth = self.max - highest_edge;

        //Calculate the width by doing (number of bins between edges multiplied by bin size)

        let width: f32 = if second_edge > first_edge {
            ((second_edge - first_edge) as f32) * bin_size
        } else {
            bail!("Invalid edge config")
        };

        if second_edge == self.max_pos.0 || first_edge == self.max_pos.0 {
            bail!("Invalid edge config")
        }

        let slope = if max_row[second_edge] >= max_row[first_edge] {
            depth / ((second_edge - self.max_pos.0) as f32 * bin_size)
        } else {
            depth / ((self.max_pos.0 - first_edge) as f32 * bin_size)
        };

        Ok((depth, width, slope))
    }



    ///Updates a section of the heightmap by slotting in another heightmap
    /// Assumes that the other heightmap has the same resolution
    pub fn update_section(&mut self, mut other : Heightmap) -> Result<(), anyhow::Error>{

        //Check that it is feasible for the other heightmap to sit on the main heightmap
        if self.upper_coord_bounds[0] < other.lower_coord_bounds()[0] || self.lower_coord_bounds[0] > other.upper_coord_bounds[0] || self.lower_coord_bounds[1] > other.upper_coord_bounds[1]|| self.upper_coord_bounds[1] < other.lower_coord_bounds()[1]{
            println!("out of bounds");
            bail!("Other heightmap does not sit on main!")            
        }


        //--Locate bin in main that corresponds to first bin in other (can be negative - i.e. it starts outside)--
        //Get bin resolution of main
        let bin_res_width = (self.upper_coord_bounds[0] - self.lower_coord_bounds[0]) / self.width as f32;
        let bin_res_height = (self.upper_coord_bounds[1] - self.lower_coord_bounds[1]) / self.height as f32;

        let start_col = ((other.lower_coord_bounds[0] - self.lower_coord_bounds[0]) / bin_res_width) as i32;
        let start_row = ((other.lower_coord_bounds[1] - self.lower_coord_bounds[1]) / bin_res_height) as i32;

        //Go through each bin and and update the corresponding bin
        for i in start_row..(start_row + other.height as i32){
            
            if i < 0 || i >= self.height as i32{
                continue;
            }

            for j in start_col..(start_col + other.width as i32){
                if j < 0 || j >= self.width as i32{
                continue;
            }
                let val = other.cells[(i  - start_row) as usize][(j  - start_col) as usize];
                if !val.is_nan(){
                    self.cells[i as usize][j as usize]  = val;
                }
            
            }
        }
        //Check that the bin doesnt reside outside the heightmap area



        Ok(())

    }

    pub fn set_all_cells(&mut self, val : f32){
        //Sweep through each cell and replace with the new map height
        for (x, row) in self.cells.iter_mut().enumerate() {
            for (y, col) in row.iter_mut().enumerate() {
                *col = val
            }
        }
        self.max_updated = false;
        self.min_updated = false;

        self.get_max();
        self.get_min();
    }


}

///A selection of different intensity schemas for generating the 2.5D maps from 3D data
pub enum MapGenOpt {
    Mean,
    //Warning! Median will be a slow process - you have to sort the list of points in each section
    Median,
    Min,
    Max,
}

///Transforms 3 point data into a 2.5d heightmap ( where the 3rd data point is the "height"/intensity)
pub fn trans_to_heightmap(
    data: Vec<[f32; 3]>,
    bins_per_row: usize,
    bins_per_col: usize,
    total_width: f32,
    total_height: f32,
    min_x_bnd: f32,
    min_y_bnd: f32,
    opt: MapGenOpt,
) -> Result<Vec<Vec<f32>>, anyhow::Error> {
    //Create the empty cell matrix
    //NaN spots are areas with no information
    let mut cells_pnt_list = vec![vec![vec![]; bins_per_row]; bins_per_col];

    let width_resolution = (total_width / bins_per_row as f32);
    let height_resolution = (total_height / bins_per_col as f32);

    //Check each points and direct it to a cell (updating the average height)
    for pnt in data {
        //Start in the first bin (as the point will never sit at the minimum x bound)
        let mut n = 0;
        let mut m = 0;

        let mut n_fnd = false;
        let mut m_fnd = false;

        let curr_x = pnt[0];
        let curr_y = pnt[1];

        //Find the horizontal pos
        while !n_fnd | !m_fnd{
            if !n_fnd{
                if curr_x <= ((width_resolution * n as f32) + (min_x_bnd + width_resolution)) {
                    n_fnd = true;
                } else {
                    n += 1;
                }
                //Check if end pos
                if n == bins_per_row - 1{                
                    n_fnd = true;
                }
            }
            if !m_fnd{
                //Find the vertical pos
                if curr_y <= (( height_resolution * m as f32) + (min_y_bnd + height_resolution)) {
                    m_fnd = true;
                } else {
                    m += 1;
                }
                //Check if end pos
                if m == bins_per_col - 1{                     
                    
                    m_fnd = true;
                }
            }
        }
        //add the point to the cell point list
        cells_pnt_list[m][n].push(pnt[2]);
    }

    //Calculate the height of each cell based on the chosen hmap option
    let mut cells: Vec<Vec<f32>> = vec![vec![]; bins_per_col];

    //Iterate through each point list
    for (i, pnt_list) in cells_pnt_list.iter_mut().enumerate() {
        for pnts in pnt_list {
            //If the cell is NAN - keep it as a null cell
            if pnts.iter().all(|x| x.is_nan()) || pnts.is_empty() {
                cells[i].push(f32::NAN);
                continue;
            }

            let mut pnt_to_add: f32 = 0.0;

            //Determine the value of the cell bsaed on the provided heightmap options
            match opt {
                MapGenOpt::Mean => {
                    let mut cnt = 0;

                    //sum all the points in the list
                    for pnt in pnts {
                        //Ignore the nan points
                        if pnt.is_nan() {
                            continue;
                        }

                        pnt_to_add += *pnt;
                        cnt += 1;
                    }

                    //Divide by the length of the list
                    pnt_to_add /= cnt as f32;
                }

                MapGenOpt::Median => {
                    //Sort the list - unstable because we dont care about initial order of indices and also its faster
                    pnts.sort_unstable_by(f32::total_cmp);

                    if pnts.len() == 1 {
                        pnt_to_add = pnts[0]
                    } else {
                        //check if length is odd/even
                        let even = { pnts.len() % 2 == 0 };

                        //Identify the median value
                        if even {
                            pnt_to_add =
                                (pnts[pnts.len().div_ceil(2)] + pnts[(pnts.len() - 1) / 2]) / 2.0
                        } else {
                            pnt_to_add = pnts[(pnts.len()) / 2]
                        }
                    }
                }

                MapGenOpt::Max => {
                    //Set the max default
                    pnt_to_add = -9999.0;
                    //Iterate through every point and get the max
                    for pnt in pnts {
                        if *pnt > pnt_to_add {
                            pnt_to_add = *pnt;
                        }
                    }
                }

                MapGenOpt::Min => {
                    //Iterate through every point and get the min
                    //Set the min default
                    pnt_to_add = 9999.0;
                    //Iterate through every point and get the max
                    for pnt in pnts {
                        if *pnt < pnt_to_add {
                            pnt_to_add = *pnt;
                        }
                    }
                }
            }

            cells[i].push(pnt_to_add);
        }
    }

    Ok(cells)
}

///Adds a value (that could possibly be NaN) to a variable
fn add_nan_f32(var: f32, val: f32) -> f32 {
    //If the value is NaN just add nothing
    if val.is_nan() { var } else { var + val }
}

///Subtracts a value (that could possibly be NaN) to a variable
fn sub_nan_f32(var: f32, val: f32) -> f32 {
    //If the value is NaN just add nothing
    add_nan_f32(var, -val)
}
