///A set of tools around creating/manipulating/combining 2.5D heightmap objects
use crate::pointcloud::PointCloud;
use anyhow::bail;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
///Heightmap structure - contains the size and height information for each cell
#[derive(Clone)]
pub struct Heightmap {
    ///Number of pixel rows
    height: u32,
    ///Number of pixel columns
    width: u32,

    ///The number of cells present in the heightmap
    pub no_of_cells: u32,

    ///The point defined minimum cell height
    min: f64,
    ///The point defined maximum cell height
    max: f64,

    ///State that determines whether the minimum height needs to be checked
    min_updated: bool,
    ///State that determines whether the maximum height needs to be checked
    max_updated: bool,

    ///Cell location at minimum height
    min_pos: (u32, u32),
    ///Cell location at maximum height
    max_pos: (u32, u32),

    ///the 2d vector representing the cells
    cells: Vec<Vec<f64>>,

    ///The pointcloud real-world lower bounds the hieghtmap is constructed from
    lower_coord_bounds: [f64; 2],
    ///The pointcloud real-world upper bounds the hieghtmap is constructed from
    upper_coord_bounds: [f64; 2],

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
    pub fn new(width: u32, height: u32) -> Self {
        //No filename as is a new object
        let filename = "No filepath".to_string();

        //Generate an empty cell bed of height*width size
        Self {
            height,
            width,
            no_of_cells: height * width,
            cells: vec![vec![0.0; height as usize]; width as usize],
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
        width: u32,
        height: u32,
    ) -> Result<Self, anyhow::Error> {
        //create the filename from the relative timestamp
        let filename = format!("created from pcl- {}", pcl.timestamp());

        //Get the bounds
        let bounds = pcl.get_bounds();

        //Calculate the real distance and height of the heightmap
        let total_width = bounds[1] - bounds[0];
        let total_height = bounds[3] - bounds[2];

        if let Ok(cells) = trans_to_heightmap(
            pcl.consume_points(),
            width as usize,
            height as usize,
            total_width,
            total_height,
            bounds[0],
            bounds[2],
            MapGenOpt::Mean,
        ) {
            Ok(Self {
                height,
                width,
                no_of_cells: height * width,
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
    pub fn create_using_pcl_ref(
        pcl: &PointCloud,
        width: u32,
        height: u32,
    ) -> Result<Self, anyhow::Error> {
        //create the filename from the relative timestamp
        let filename = format!("created from pcl- {}", pcl.timestamp());

        //Get the bounds
        let bounds = pcl.get_bounds();

        //Calculate the real distance and height of the heightmap
        let total_width = bounds[1] - bounds[0];
        let total_height = bounds[3] - bounds[2];

        if let Ok(cells) = trans_to_heightmap(
            pcl.points(),
            width as usize,
            height as usize,
            total_width,
            total_height,
            bounds[0],
            bounds[2],
            MapGenOpt::Mean,
        ) {
            Ok(Self {
                height,
                width,
                no_of_cells: height * width,
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

    ///Lodas a hmap file and creates a heightmap from it
    pub fn create_from_file(filepath: String) -> Result<Self, anyhow::Error> {
        //Open the file and create a buffer to read the lines
        let file = File::open(filepath.clone())?;
        let mut line_reader = BufReader::new(file);

        //Read the first line to extract the bounds
        let mut first_line = String::new();
        line_reader.read_line(&mut first_line)?;

        let bounds = Self::extract_bounds(first_line);

        let mut bounds_res: [f64; 4] = [f64::NAN, f64::NAN, f64::NAN, f64::NAN];

        if bounds.is_ok() {
            bounds_res = bounds?
        }

        let mut height = 0;
        let mut width = 0;
        let mut width_set = false;

        let mut cells: Vec<Vec<f64>> = vec![];
        //Go through each cell and update the
        for line in line_reader.lines() {
            //Create the empty row
            let mut row: Vec<f64> = vec![];

            //Split via comma then iterate
            for token in line?.split(",") {
                if !token.is_empty() {
                    if !width_set {
                        width += 1;
                    }

                    row.push(token.parse::<f64>()?);
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
            no_of_cells: height * width,
            min: 999.0,
            max: -999.0,
            min_updated: false,
            max_updated: false,
            min_pos: (0, 0),
            max_pos: (0, 0),
            cells,
            lower_coord_bounds: [bounds_res[0], bounds_res[2]],
            upper_coord_bounds: [bounds_res[1], bounds_res[3]],
            filename: filepath,
        })
    }

    ///Creates a heightmap from a given pcl file
    pub fn create_from_pcl_file(
        filepath: String,
        width: u32,
        height: u32,
    ) -> Result<Self, anyhow::Error> {
        //Load the pcl file
        let pcl = PointCloud::create_from_file(filepath)?;

        //Turn the pcl into a heightmap and return it
        Heightmap::create_from_pcl(pcl, width, height)
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
    pub fn get_cell_height(&self, x: u32, y: u32) -> Result<f64, anyhow::Error> {
        if x > self.width || y > self.height {
            bail!("Warning - attempting to read from cell that doesnt exist!");
        }

        Ok(self.cells[x as usize][y as usize])
    }

    ///Set the height of a given cell
    pub fn set_cell_height(
        &mut self,
        x: u32,
        y: u32,
        new_height: f64,
    ) -> Result<(), anyhow::Error> {
        if x > self.width || y > self.height {
            bail!("Warning - attempting to write to cell that doesnt exist!");
        }

        self.cells[x as usize][y as usize] = new_height;

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

    pub fn lower_coord_bounds(&self) -> [f64; 2] {
        self.lower_coord_bounds
    }
    pub fn upper_coord_bounds(&self) -> [f64; 2] {
        self.upper_coord_bounds
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
    pub fn offset_map(&mut self, height_change: f64) {
        //Sweep through each cell and replace with the new map height
        for row in self.cells.iter_mut() {
            for col in row.iter_mut() {
                *col = add_nan(*col, height_change);
            }
        }
        self.max_updated = false;
        self.min_updated = false;
    }

    ///Get the maximum cell distance
    fn get_max(&mut self) -> f64 {
        //Skip the first as they contain plenty of nans
        let mut skip_first_row = true;
        let mut skip_first_column = true;

        let mut max_pos: (u32, u32) = (0, 0);

        //Check whether a new maximum is required
        if !self.max_updated {
            let mut new_max: f64 = -999.0;

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
                        max_pos = (i as u32, j as u32);
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
    fn get_min(&mut self) -> f64 {
        //Check whether a new minimum calc is required
        if !self.min_updated {
            //Skip the first as they contain plenty of nans
            let mut skip_first_row = true;
            let mut skip_first_column = true;

            let mut new_min: f64 = 999.0;
            let mut min_pos: (u32, u32) = (0, 0);

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
                        min_pos = (i as u32, j as u32);
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
    pub fn get_flattened_cells(&self) -> Result<Vec<f64>, anyhow::Error> {
        let mut cell_list: Vec<f64> = vec![];

        for x in 0..(self.width - 1) as usize {
            for y in 0..(self.height - 1) as usize {
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
            "bnds:[{},{}][{},{}]\n",
            self.lower_coord_bounds[0],
            self.lower_coord_bounds[1],
            self.upper_coord_bounds[0],
            self.upper_coord_bounds[1]
        );
        file.write_all(first_line.as_bytes())?;

        //Iterate thorugh each row
        for row in self.cells.iter() {
            for cell in row {
                let cell_val = format!("{:?},", cell);
                file.write_all(cell_val.as_bytes())?
            }

            file.write_all("\n".as_ref())?;
        }

        Ok(())
    }

    ///Width getter
    pub fn width(&self) -> u32 {
        self.width
    }
    ///Height getter
    pub fn height(&self) -> u32 {
        self.height
    }

    ///Extracts the bounds from a string
    fn extract_bounds(bnd_line: String) -> Result<[f64; 4], anyhow::Error> {
        let mut bounds: [f64; 4] = [f64::NAN, f64::NAN, f64::NAN, f64::NAN];

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
    pub fn calc_cell_mid_pnt(&self, n: u32, m: u32) -> Result<[f64; 2], anyhow::Error> {
        if n >= self.width {
            bail!("Error - out of width bounds!")
        }
        if m >= self.height {
            bail!("Error - out of height bounds!");
        }

        let mut mid_pnt: [f64; 2] = [f64::NAN, f64::NAN];

        let cell_width =
            (self.upper_coord_bounds[0] - self.lower_coord_bounds[0]).abs() / self.width as f64;
        let cell_height =
            (self.upper_coord_bounds[1] - self.lower_coord_bounds[1]).abs() / self.width as f64;

        //Calc the mid point with the offset
        mid_pnt[0] = (n as f64 * cell_width) + self.lower_coord_bounds[0];
        mid_pnt[1] = (m as f64 * cell_height) + self.lower_coord_bounds[1];

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
                if !self.cells[i as usize][j as usize].is_nan() {
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
                    total = add_nan(total, self.cells[(i - 1) as usize][j as usize]);
                    cnt += 1;

                    if !l_edge_flag {
                        total = add_nan(total, self.cells[(i - 1) as usize][(j - 1) as usize]);
                        cnt += 1;
                    }
                    if !r_edge_flag {
                        total = add_nan(total, self.cells[(i - 1) as usize][(j + 1) as usize]);
                        cnt += 1;
                    }
                }
                //Check the middle row
                if !l_edge_flag {
                    total = add_nan(total, self.cells[i as usize][(j - 1) as usize]);
                    cnt += 1;
                }
                if !r_edge_flag {
                    total = add_nan(total, self.cells[i as usize][(j + 1) as usize]);
                    cnt += 1;
                }

                //check the bottom row
                if !b_edge_flag {
                    total = add_nan(total, self.cells[(i + 1) as usize][j as usize]);
                    cnt += 1;
                    if !l_edge_flag {
                        total = add_nan(total, self.cells[(i + 1) as usize][(j - 1) as usize]);
                        cnt += 1;
                    }
                    if !r_edge_flag {
                        total = add_nan(total, self.cells[(i + 1) as usize][(j + 1) as usize]);
                        cnt += 1;
                    }
                }

                self.cells[i as usize][j as usize] = total / (cnt as f64);
            }
        }
    }

    ///Calculates a simple estimation of a feature in a heightmap
    /// Returned as (depth, width, (slope median, slope mode))
    /// Assumes only two edges (i.e. a simple trough)
    /// Assumes that the deep point is in the middle
    pub fn calc_simple_feature(&mut self) -> Result<(f64, f64, f64), anyhow::Error> {
        //Calculate the bin size -- Assuming square so only need one set of bounds
        let bin_size =
            (self.upper_coord_bounds[0] - self.lower_coord_bounds[0]) / (self.width as f64 - 1.0);

        //Get the lowest point and its row (which is actually the max - furhter away from heightmap)
        if !self.max_updated {
            self.get_max();
            self.get_min();
        }
        let max_row = self.max_pos.0;

        //Load the row - CHECKED TO BE CORRECT USING PREGENNED HEIGHTMAPS
        let max_row = self.cells[max_row as usize].clone();

        //Take edge point as highest and use to calculate depth of feature
        const EDGE_THRESHOLD: f64 = 0.0005;
        const EDGE_HIGH_THRESHHOLD: f64 = 0.01;
        let mut first_edge: usize = 0;
        let mut second_edge: usize = 0;

        //Compare a cell to see if it meets the threshold
        for i in 0..((self.width() - 1) as usize) {
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

            if i == (self.width() - 1) as usize {
                bail!("Failed to find first edge in hmap")
            }
        }
        //Go backwards to find the edge on the other side
        for i in (1..((self.width()) as usize)).rev() {
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

        if self.max_pos.0 > second_edge as u32 || self.max_pos.0 < first_edge as u32 {
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

        let width: f64 = if second_edge > first_edge {
            ((second_edge - first_edge) as f64) * bin_size
        } else {
            bail!("Invalid edge config")
        };

        if second_edge == self.max_pos.0 as usize || first_edge == self.max_pos.0 as usize {
            bail!("Invalid edge config")
        }

        let slope = if max_row[second_edge] >= max_row[first_edge] {
            depth / ((second_edge as u32 - self.max_pos.0) as f64 * bin_size)
        } else {
            depth / ((self.max_pos.0 - first_edge as u32) as f64 * bin_size)
        };

        Ok((depth, width, slope))
    }
}

///Compares a given map with a desired map and outputs a map of height differences
pub fn comp_maps(
    curr_map: &Heightmap,
    desired_map: &Heightmap,
) -> Result<Heightmap, anyhow::Error> {
    //Check the maps are the same size - if not exit
    if curr_map.height != desired_map.height || curr_map.width != desired_map.width {
        bail!("Warning - Maps are not the same size - cannot be compared");
    }

    //Create a new empty map that holds the difference
    let mut diff_map: Heightmap = Heightmap::new(curr_map.width, curr_map.height);

    //Sweep through each cell and replace with the new map height
    for (m, row) in diff_map.cells.iter_mut().enumerate() {
        for (n, col) in row.iter_mut().enumerate() {
            //First index is the row number (i.e. the height)
            let diff = curr_map.cells[m][n] - desired_map.cells[m][n];

            //Not entirely sure why y and x are the opposite way rounds but hey ho
            *col = diff;
        }
    }

    Ok(diff_map)
}

//General heightmap statistical functions

///Takes a list of heightmaps that are the same size and averages them
pub fn average_heightmaps(
    hmap_list: &[Heightmap],
    lower_bounds: [f64; 2],
    upper_bounds: [f64; 2],
) -> Heightmap {
    //Create an empty heightmap the same size as the heightmaps in the list
    let mut avg_hmap = Heightmap::new(hmap_list[0].width(), hmap_list[0].height());

    //Go through each point in the empty heightmap and take the mean of the genned heightmaps
    for (y, row) in avg_hmap.cells.iter_mut().enumerate() {
        for (x, val) in row.iter_mut().enumerate() {
            let mut total_val = 0.0;
            let mut valid_pnt_cnt = 0;
            for hmap in hmap_list {
                let cell_val = hmap.cells[y][x];

                //Ignore nan valued cells - dont diminish mean error
                if cell_val.is_nan() {
                    continue;
                }
                total_val += cell_val;
                valid_pnt_cnt += 1;
            }

            if total_val == 0.0 {
                *val = f64::NAN
            } else {
                *val = total_val / valid_pnt_cnt as f64;
            }
        }
    }

    avg_hmap.lower_coord_bounds = lower_bounds;
    avg_hmap.upper_coord_bounds = upper_bounds;

    avg_hmap
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
fn trans_to_heightmap(
    data: Vec<[f64; 3]>,
    width: usize,
    height: usize,
    total_width: f64,
    total_height: f64,
    min_x_bnd: f64,
    min_y_bnd: f64,
    opt: MapGenOpt,
) -> Result<Vec<Vec<f64>>, anyhow::Error> {
    //Create the empty cell matrix
    //NaN spots are areas with no information
    let mut cells_pnt_list = vec![vec![vec![]; width]; height];

    //Check where the trajectory lies within the cell space - copied from heightmap generation
    //Check each points and direct it to a cell (updating the average height)
    for pnt in data {
        let mut n = 0;
        let mut m = 0;

        let mut n_fnd = false;
        let mut m_fnd = false;

        //Find the horizontal pos
        while !n_fnd {
            if pnt[0] <= (((total_width / width as f64) * n as f64) + min_x_bnd) {
                n_fnd = true;
            } else {
                n += 1;
            }

            //Check if end pos
            if n == width - 1 {
                n_fnd = true;
            }
        }

        //Find the vertical pos
        while !m_fnd {
            if pnt[1] <= (((total_height / height as f64) * m as f64) + min_y_bnd) {
                m_fnd = true;
            } else {
                m += 1;
            }
            //Check if end pos
            if m == height - 1 {
                m_fnd = true;
            }
        }

        //add the point to the cell point list
        cells_pnt_list[n][m].push(pnt[2]);
    }

    //Calculate the height of each cell based on the chosen hmap option
    let mut cells: Vec<Vec<f64>> = vec![vec![]; width];

    //Iterate through each point list
    for (i, pnt_list) in cells_pnt_list.iter_mut().enumerate() {
        for pnts in pnt_list {
            //If the cell is NAN - keep it as a null cell
            if pnts.iter().all(|x| x.is_nan()) || pnts.is_empty() {
                cells[i].push(f64::NAN);
                continue;
            }

            let mut pnt_to_add: f64 = 0.0;

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
                    pnt_to_add /= cnt as f64;
                }

                MapGenOpt::Median => {
                    //Sort the list - unstable because we dont care about initial order of indices and also its faster
                    pnts.sort_unstable_by(f64::total_cmp);

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
fn add_nan(var: f64, val: f64) -> f64 {
    //If the value is NaN just add nothing
    if val.is_nan() { var } else { var + val }
}

///Subtracts a value (that could possibly be NaN) to a variable
fn sub_nan(var: f64, val: f64) -> f64 {
    //If the value is NaN just add nothing
    add_nan(var, -val)
}
