/*
A kalman filtered heightmap

The intended use is:
Global heightmap insertion -> The global heightmap will have a confidence representation and when a new local heightmap is inserted - it will not necessarily override previous measurements


Design notes:
- for the global heightmap insertion it will be important to include a high process noise (Q) locally so that terrain modifications aren't ignored


*/

use crate::data_types::heightmap::Heightmap;


struct UncHeightmap{
    //The heightmap
    hmap : Heightmap,
    //The variance mapping
    variance : Vec<Vec<f32>>
}