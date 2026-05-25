///Analyses test results in the form of displaying heightmap results
use crate::analysis::data_handler::DataHandler;
use crate::data_types::heightmap::{Heightmap, MapGenOpt, trans_to_heightmap};
use crate::data_types::pointcloud::PointCloud;
use anyhow::bail;
use std::fs;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};

///The analyser structure which contains all the relevant test information
pub struct Analyser {
    ///Filepath location of the test data
    filepath: String,
    ///Name of the test
    test_name: String,
    ///Folder location holding the test data
    test_fp: String,
    ///The struct containing all the raw pos/force info etc
    data_handler: Option<DataHandler>,
    ///Number of pointclouds taken in this test
    no_of_pcl: i32,
    ///Coordination pre-transform flag - True if trajectory already transformed
    pre_trans: bool,
}

impl Analyser {
    ///Create an analyser structure based on the provided file
    pub fn init(filepath: String, test_name: String) -> Result<Self, anyhow::Error> {
        //Check that the filepath exists
        if !fs::read_dir(&filepath)?
            .any(|e| e.unwrap().file_name().into_string().unwrap() == test_name)
        {
            bail!("Test {} does not exist!", test_name);
        }

        let test_fp = format!("{}/{}", filepath, test_name);

        //Count the number of pointclouds
        let mut no_of_pcl = 0;
        for path in fs::read_dir(&test_fp)? {
            if path?.file_name().to_str().unwrap().starts_with("pcl_") {
                no_of_pcl += 1;
            }
        }

        //Create the data handler
        let data_fp = format!("{}/data_{}.txt", test_fp, test_name);

        //println!("Loading data - {data_fp}");
        let data_handler: Option<DataHandler>;
        if let Ok(data) = DataHandler::read_data_from_file(data_fp) {
            data_handler = Some(data);
        } else {
            println!("WARNING: No data file");
            data_handler = None;
        }

        //Get the config info
        //println!("Loading config - {test_fp}");

        let mut pre_trans = false;
        if let Ok(info) = get_config(&test_fp, &test_name) {
            pre_trans = info;
        }

        Ok(Self {
            filepath,
            test_name: test_name.to_string(),
            test_fp,
            data_handler,
            no_of_pcl,
            pre_trans,
        })
    }

    ///Get the rectangular trajectory bound of the test
    pub fn get_traj_bounds(&self) -> Result<[f64; 4], anyhow::Error> {
        if self.data_handler.is_some() {
            self.data_handler
                .as_ref()
                .expect("Failed to unwrap existing data handler")
                .get_traj_rect_bnds()
        } else {
            bail!("No data file associated with test");
        }
    }

    ///Counts the number of generated hmaps saved in the test dir
    fn get_hmap_cnt(&mut self) -> Result<i32, anyhow::Error> {
        //Count the number of heightmaps saved in the test
        let mut hmap_cnt = 0;

        for path in fs::read_dir(&self.test_fp)? {
            if path?.file_name().to_str().unwrap().starts_with("hmap_") {
                hmap_cnt += 1;
            }
        }

        Ok(hmap_cnt)
    }

    ///Generate a heightmap from a specific point cloud
    fn gen_hmap_n(
        &mut self,
        width: usize,
        height: usize,
        n: i32,
    ) -> Result<Heightmap, anyhow::Error> {
        println!("GENERATING HEIGHTMAP {n} ---------");

        //Check that the n number is valid
        if n >= self.no_of_pcl {
            bail!("Invalid hmap number")
        }

        //Create the filepath
        let fp: String;
        if n == -1 {
            fp = format!("{}/pcl_{}_START.txt", self.test_fp, self.test_name);
        } else if n == -2 {
            fp = format!("{}/pcl_{}_END.txt", self.test_fp, self.test_name);
        } else {
            fp = format!("{}/pcl_{}_{}.txt", self.test_fp, self.test_name, n);
        }
        //Generate the heightmap from the pcl file
        let hmap = Heightmap::create_from_pcl_file(fp, width, height)?;

        Ok(hmap)
    }

    ///Load all the hmaps saved from a test directory to a vector
    fn load_all_genned_hmaps(&mut self) -> Result<Vec<Heightmap>, anyhow::Error> {
        println!("LOADING HEIGHTMAPS ---------");

        let mut heightmaps: Vec<Heightmap> = vec![];

        //Iterate through each file
        for path in fs::read_dir(&self.test_fp)? {
            let path_str = path?.file_name();
            let path_str = path_str.to_str().unwrap();

            //Identify the hmap files
            if path_str.starts_with("hmap_") {
                let fp = format!("{}/{}", self.test_fp, path_str);

                //Load the heightmap file
                heightmaps.push(Heightmap::create_from_file(fp)?)
            }
        }
        Ok(heightmaps)
    }

    ///Load all the pointclouds saved from a test directory to a vector
    fn load_all_pcl(&mut self) -> Result<Vec<PointCloud>, anyhow::Error> {
        println!("LOADING POINTCLOUDS ----------");

        let mut pcls: Vec<PointCloud> = vec![];

        //Iterate through each file
        for path in fs::read_dir(&self.test_fp)? {
            let path_str = path?.file_name();
            let path_str = path_str.to_str().unwrap();

            //Identify the hmap files
            if path_str.starts_with("pcl_") {
                let fp = format!("{}/{}", self.test_fp, path_str);

                //Load the heightmap file
                pcls.push(PointCloud::create_from_file(fp)?)
            }
        }
        Ok(pcls)
    }

    ///Get pcls that have the identifier in the filepath
    pub fn get_pcl_with_identifier(
        &self,
        identifier: &str,
    ) -> Result<Vec<PointCloud>, anyhow::Error> {
        let mut pcls: Vec<PointCloud> = vec![];

        //Iterate through each file
        for path in fs::read_dir(&self.test_fp)? {
            let path_str = path?.file_name();
            let path_str = path_str.to_str().unwrap();

            //Identify the hmap files
            if path_str.starts_with("pcl_") && path_str.contains(identifier) {
                let fp = format!("{}/{}", self.test_fp, path_str);

                //Load the heightmap file
                pcls.push(PointCloud::create_from_file(fp)?)
            }
        }
        Ok(pcls)
    }

    ///Load all PCLs and cut them to meet the specified passband
    ///WARNING: Will delete your data
    pub fn apply_passband(
        &mut self,
        min_x: f32,
        max_x: f32,
        min_y: f32,
        max_y: f32,
        min_z: f32,
        max_z: f32,
    ) -> Result<(), anyhow::Error> {
        let mut cnt = 0;

        //Iterate through every pcl file in the test
        //Iterate through each file
        for path in fs::read_dir(&self.test_fp)? {
            let path_str = path?.file_name();
            let path_str = path_str.to_str().unwrap();

            //Identify the pcl files
            if path_str.starts_with("pcl_") {
                println!("Trimming PCL: {}", cnt);

                let pcl_fp = format!("{}/{}", self.test_fp, path_str);

                let mut curr_pcl = PointCloud::create_from_file(pcl_fp.clone())?;

                //Rotate the PCL
                curr_pcl.crop(min_x, max_x, min_y, max_y, min_z, max_z);

                //Resave the PCL
                curr_pcl.save_to_file(pcl_fp.strip_suffix(".txt").unwrap())?;

                cnt += 1;
            }
        }

        println!("Trimming complete");

        Ok(())
    }

    ///From a tests pointclouds - create a set of parametric heightmaps
    ///Define the resolutions, averages and identifiers
    pub fn create_parametric_hmaps(
        &mut self,
        resolutions: Vec<usize>,
        averages: Vec<u32>,
        identifiers: Vec<&str>,
    ) -> Result<(), anyhow::Error> {
        //Get the maximum average value
        let max_avg = *averages.iter().max().expect("Failed to get maximum") as i32;

        //For each identifier specified
        for identifier in identifiers.iter() {
            //Loda all the pointclouds with that identifier
            let mut curr_pcls: Vec<PointCloud> = self
                .get_pcl_with_identifier(*identifier)
                .expect("Failed to find identifier");

            //check that there are enough for the maxmimum average
            if curr_pcls.len() < max_avg as usize {
                bail!("Not enough pointclouds for the average required!")
            }

            for resolution in resolutions.iter() {
                let mut cnt: u32 = 0;

                //Turn all of the pointclouds into heightmaps
                let mut curr_hmaps: Vec<Heightmap> = vec![];

                for pcl in curr_pcls.iter() {
                    curr_hmaps.push(Heightmap::create_using_pcl_ref(
                        pcl,
                        *resolution,
                        *resolution,
                    )?);

                    cnt += 1;

                    if averages.contains(&cnt) {
                        let filepath = format!(
                            "{}/hmap_{}_{}_res_{}_avg_{}",
                            self.test_fp, self.test_name, *identifier, *resolution, cnt
                        );

                        let _ = average_heightmaps(
                            &curr_hmaps,
                            curr_hmaps[0].lower_coord_bounds(),
                            curr_hmaps[0].upper_coord_bounds(),
                        )?
                        .save_to_file(&filepath);

                        println!(
                            "Average created for {} number of pcls at resolution {} at identifier {}",
                            cnt, *resolution, *identifier
                        );
                    }
                }
                println!("Completed for resolution {}", *resolution);
            }

            //Empty the current list
            curr_pcls.clear();

            println!("Completed for identifier {}", *identifier);
        }

        Ok(())
    }

    ///From a tests pointclouds - create a set of parametric heightmaps
    ///Define the resolutions, averages and identifiers
    /// Compares these heightmaps with a provided pointcloud ground truth
    /// Save the generated error map
    /// Allows a user to specify height deltas to offset height differences
    pub fn save_parametric_error_maps(
        &mut self,
        resolutions: Vec<usize>,
        averages: Vec<u32>,
        identifiers: Vec<&str>,
        gnd_truth_pcl: &PointCloud,
        height_deltas: Vec<f32>,
    ) -> Result<(), anyhow::Error> {
        //Get the maximum average value
        let max_avg = *averages.iter().max().expect("Failed to get maximum") as i32;

        //For each identifier specified
        for (i, identifier) in identifiers.iter().enumerate() {
            //Loda all the pointclouds with that identifier
            let mut curr_pcls: Vec<PointCloud> = self
                .get_pcl_with_identifier(*identifier)
                .expect("Failed to find identifier");

            //check that there are enough for the maxmimum average
            if curr_pcls.len() < max_avg as usize {
                bail!("Not enough pointclouds for the average required!")
            }

            if identifiers.len() != height_deltas.len() {
                bail!("Number of identifiers does not equal number of height deltas!")
            }

            for resolution in resolutions.iter() {
                let mut cnt: u32 = 0;

                //Turn all of the pointclouds into heightmaps
                let mut curr_hmaps: Vec<Heightmap> = vec![];

                let curr_gnd_truth =
                    Heightmap::create_using_pcl_ref(gnd_truth_pcl, *resolution, *resolution)?;

                for pcl in curr_pcls.iter() {
                    curr_hmaps.push(Heightmap::create_using_pcl_ref(
                        pcl,
                        *resolution,
                        *resolution,
                    )?);

                    cnt += 1;

                    if averages.contains(&cnt) {
                        let mut curr_hmap = average_heightmaps(
                            &curr_hmaps,
                            curr_hmaps[0].lower_coord_bounds(),
                            curr_hmaps[0].upper_coord_bounds(),
                        )?;

                        //Acount for user specified height offsets
                        if height_deltas[i] != 0.0 {
                            curr_hmap.offset_map(height_deltas[i]);
                        }

                        let err_map =
                            comp_maps(&curr_gnd_truth, &curr_hmap).expect("Failed to calc err map");

                        //Save the stats file
                        let filepath = format!(
                            "{}/hmap_err_{}_{}_{}_{}.txt",
                            self.test_fp, self.test_name, *identifier, resolution, cnt
                        );

                        err_map.save_to_file(&filepath)?;
                    }
                }
                println!("{}-Completed for resolution {}", identifier, *resolution);
            }

            //Empty the current list
            curr_pcls.clear();

            println!("Completed for identifier {}", *identifier);
        }

        Ok(())
    }

    ///From a tests pointclouds - create a set of parametric heightmaps
    ///Define the resolutions, averages and identifiers
    /// Compares these heightmaps with a provided pointcloud ground truth
    /// Calculates the mean error and std_dev err
    /// Allows a user to specify height deltas to offset height differences
    pub fn save_parametric_hmap_stats(
        &mut self,
        resolutions: Vec<usize>,
        averages: Vec<u32>,
        identifiers: Vec<&str>,
        gnd_truth_pcl: &PointCloud,
    ) -> Result<(), anyhow::Error> {
        //Get the maximum average value
        let max_avg = *averages.iter().max().expect("Failed to get maximum") as i32;

        //For each identifier specified
        for (i, identifier) in identifiers.iter().enumerate() {
            let mut dist_stats: Vec<(usize, u32, f32, f32, u32)> = vec![];

            //Loda all the pointclouds with that identifier
            let mut curr_pcls: Vec<PointCloud> = self
                .get_pcl_with_identifier(*identifier)
                .expect("Failed to find identifier");

            //check that there are enough for the maxmimum average
            if curr_pcls.len() < max_avg as usize {
                bail!("Not enough pointclouds for the average required!")
            }

            for resolution in resolutions.iter() {
                let mut cnt: u32 = 0;

                //Turn all of the pointclouds into heightmaps
                let mut curr_hmaps: Vec<Heightmap> = vec![];

                let curr_gnd_truth =
                    Heightmap::create_using_pcl_ref(gnd_truth_pcl, *resolution, *resolution)?;

                for pcl in curr_pcls.iter() {
                    curr_hmaps.push(Heightmap::create_using_pcl_ref(
                        pcl,
                        *resolution,
                        *resolution,
                    )?);

                    cnt += 1;

                    if averages.contains(&cnt) {
                        let mut curr_hmap = average_heightmaps(
                            &curr_hmaps,
                            curr_hmaps[0].lower_coord_bounds(),
                            curr_hmaps[0].upper_coord_bounds(),
                        )?;

                        let err_map =
                            comp_maps(&curr_gnd_truth, &curr_hmap).expect("Failed to calc err map");

                        let stats = get_err_stats(&err_map);

                        dist_stats.push((*resolution, cnt, stats.0, stats.1, err_map.no_of_nans()));
                    }
                }
                println!("{}-Completed for resolution {}", identifier, *resolution);
            }

            //Save the stats file
            let filepath = format!(
                "{}/comp_stats_{}_{}.csv",
                self.test_fp, self.test_name, *identifier
            );

            let mut file = fs::OpenOptions::new()
                .append(true)
                .create(true)
                .open(filepath)
                .unwrap();

            let csv_header = format!("resolution,average,mean_err,std.dev_err,no_of_nans\n");
            file.write_all(csv_header.as_bytes());

            for stat in dist_stats.iter() {
                let stats_str = format!("{},{},{},{},{}\n", stat.0, stat.1, stat.2, stat.3, stat.4);
                file.write_all(stats_str.as_bytes());
            }

            //Empty the current list
            curr_pcls.clear();

            println!("Completed for identifier {}", *identifier);
        }

        Ok(())
    }

    ///Go through the pcls in sets of identifiers
    ///Calculate the point density (no. of points / area) - assume rectangular plane
    ///
    pub fn save_avg_point_density(&self, identifiers: Vec<&str>) {
        //Density list
        let mut avg_densities: Vec<(&str, f32)> = vec![];

        //For each identifier specified
        for identifier in identifiers.iter() {
            //Loda all the pointclouds with that identifier
            let curr_pcls: Vec<PointCloud> = self
                .get_pcl_with_identifier(*identifier)
                .expect("Failed to find identifier");

            let mut avg_density = 0.0;
            //Calculate the density for each pointcloud
            for pcl in curr_pcls.iter() {
                let pcl_cnt = pcl.size() as f32;

                let area = pcl.get_xy_area();

                avg_density += pcl_cnt / area;
            }

            avg_density /= curr_pcls.len() as f32;
            //Log the overall average
            avg_densities.push((identifier, avg_density))
        }

        //Save the avg densities in a csv
        //Save the stats file
        let filepath = format!("{}/pnt_density_{}.csv", self.test_fp, self.test_name);

        let mut file = fs::OpenOptions::new()
            .write(true)
            .create(true)
            .open(filepath)
            .unwrap();

        let csv_header = format!("distance,density\n");
        file.write_all(csv_header.as_bytes());

        for density in avg_densities.iter() {
            let stats_str = format!("{},{}\n", density.0, density.1);
            file.write_all(stats_str.as_bytes());
        }
    }

    ///Make a simple estimate of a simple feature
    /// Calculates the depth/width and slope of a set of heightmaps created from pcls with a specific identifier
    pub fn group_simple_feature_estimator(
        &self,
        resolutions: Vec<usize>,
        averages: Vec<u32>,
        identifiers: Vec<&str>,
    ) -> Result<(), anyhow::Error> {
        //Get the maximum average value
        let max_avg = *averages.iter().max().expect("Failed to get maximum") as i32;

        //For each identifier specified
        for (i, identifier) in identifiers.iter().enumerate() {
            //List of measured features - (depth, width, slope)
            let mut features: Vec<(usize, u32, f32, f32, f32)> = vec![];

            //Loda all the pointclouds with that identifier
            let mut curr_pcls: Vec<PointCloud> = self
                .get_pcl_with_identifier(*identifier)
                .expect("Failed to find identifier");

            //check that there are enough for the maxmimum average
            if curr_pcls.len() < max_avg as usize {
                bail!("Not enough pointclouds for the average required!")
            }

            for resolution in resolutions.iter() {
                let mut cnt: u32 = 0;

                //Turn all of the pointclouds into heightmaps
                let mut curr_hmaps: Vec<Heightmap> = vec![];

                for pcl in curr_pcls.iter() {
                    curr_hmaps.push(Heightmap::create_using_pcl_ref(
                        pcl,
                        *resolution,
                        *resolution,
                    )?);

                    cnt += 1;

                    if averages.contains(&cnt) {
                        let mut curr_hmap = average_heightmaps(
                            &curr_hmaps,
                            curr_hmaps[0].lower_coord_bounds(),
                            curr_hmaps[0].upper_coord_bounds(),
                        )?;

                        let feature_info: (usize, u32, f32, f32, f32);
                        //Calculate the feature size
                        if let Ok((depth, width, slope_avg)) = curr_hmap.calc_simple_feature() {
                            feature_info = (*resolution, cnt, depth, width, slope_avg);
                        } else {
                            //If feature isn't identified
                            feature_info = (*resolution, cnt, f32::NAN, f32::NAN, f32::NAN);
                        }
                        features.push(feature_info)
                    }
                }
            }

            //Save the stats file
            let filepath = format!("{}/features_{}.csv", self.test_fp, self.test_name);

            let mut file = fs::OpenOptions::new()
                .append(true)
                .create(true)
                .open(filepath)
                .unwrap();

            if i == 0 {
                let csv_header = format!("distance,resolution,average,depth,width,slope avg\n");
                file.write_all(csv_header.as_bytes());
            }

            for feature in features.iter() {
                let stats_str = format!(
                    "{},{},{},{},{},{}\n",
                    identifier, feature.0, feature.1, feature.2, feature.3, feature.4
                );
                file.write_all(stats_str.as_bytes());
            }
            //Empty the features vector
            features.clear();
        }

        Ok(())
    }
}

///For a given error heightmap, calculate the average (mean) error, and the standard deviation
fn get_err_stats(err_map: &Heightmap) -> (f32, f32) {
    //First flatten the map
    let flattened_cells = err_map.get_flattened_cells().expect("Failed to flatten");

    //Calculate the mean value of the cells
    let mut no_of_cells = flattened_cells.len() as f32;
    let mut sum = 0.0;
    for cell in flattened_cells.iter() {
        if cell.is_nan() {
            //Dont include the NaN cells in the mean count
            no_of_cells -= 1.0;
            continue;
        }
        sum = add_nan_f32(sum, cell.abs());
    }
    let mean = sum / no_of_cells;

    //Calculate the std deviation of the cells
    let mut sum = 0.0;
    for cell in flattened_cells.iter() {
        //Ignore nan cells
        if cell.is_nan() {
            continue;
        }
        sum += (*cell - mean).powf(2.0);
    }
    let std_dev = (sum / no_of_cells).sqrt();

    (mean, std_dev)
}

///Determines the method for force intensity calculation
pub enum ForceSel {
    ForceAvg,
    MomAvg,
    X,
    Y,
    Z,
    Xmom,
    Ymom,
    Zmom,
}

///Loads the stored test configuration
fn get_config(filepath: &String, test_name: &String) -> Result<bool, anyhow::Error> {
    //Get the first line of the cam config file
    let config_fp = format!("{}/conf_{}.txt", filepath, test_name);

    let conf_file = File::open(config_fp)?;
    let mut line_reader = BufReader::new(conf_file);

    //Read the second line to get the robot info
    let mut rob_info_line = String::new();
    line_reader.read_line(&mut rob_info_line)?;

    //Check if trajectory data has already been transformed (to the box frame)
    println!("Loading transform flag");
    let mut trans_flag_str = String::new();
    line_reader.read_line(&mut trans_flag_str)?;
    println!("{trans_flag_str}");
    let trans_flag = trans_flag_str.contains("true");

    //Attempt to create both the configs inplace
    Ok(trans_flag)
}

///Compares a given map with a desired map and outputs a map of height differences
pub fn comp_maps(
    curr_map: &Heightmap,
    desired_map: &Heightmap,
) -> Result<Heightmap, anyhow::Error> {
    //Check the maps are the same size - if not exit
    if curr_map.height() != desired_map.height() || curr_map.width() != desired_map.width() {
        bail!("Warning - Maps are not the same size - cannot be compared");
    }

    //Create a new empty map that holds the difference
    let diff_map: Heightmap = Heightmap::new(curr_map.width(), curr_map.height());

    //Sweep through each cell and replace with the new map height
    for (m, row) in diff_map.cells().iter_mut().enumerate() {
        for (n, col) in row.iter_mut().enumerate() {
            //First index is the row number (i.e. the height)
            let diff = curr_map.get_cell_height(n, m)? - desired_map.get_cell_height(n, m)?;

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
    lower_bounds: [f32; 2],
    upper_bounds: [f32; 2],
) -> Result<Heightmap, anyhow::Error> {
    //Create an empty heightmap the same size as the heightmaps in the list
    let mut avg_hmap = Heightmap::new(hmap_list[0].width(), hmap_list[0].height());

    //Go through each point in the empty heightmap and take the mean of the genned heightmaps
    for (y, row) in avg_hmap.cells().iter_mut().enumerate() {
        for (x, val) in row.iter_mut().enumerate() {
            let mut total_val = 0.0;
            let mut valid_pnt_cnt = 0;
            for hmap in hmap_list {
                let cell_val = hmap.get_cell_height(x, y)?;

                //Ignore nan valued cells - dont diminish mean error
                if cell_val.is_nan() {
                    continue;
                }
                total_val += cell_val;
                valid_pnt_cnt += 1;
            }

            if total_val == 0.0 {
                *val = f32::NAN
            } else {
                *val = total_val / valid_pnt_cnt as f32;
            }
        }
    }

    avg_hmap.set_lower_coord_bounds(lower_bounds);
    avg_hmap.set_upper_coord_bounds(upper_bounds);

    Ok(avg_hmap)
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
