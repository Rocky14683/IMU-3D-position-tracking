mod filter;
mod imu;

pub use nalgebra as na;
pub use serde::*;
pub use serial::*;
use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();
    let port = open(&args[1]).unwrap();

    let imu = imu::IMU::new(&port);

    loop {
        /*
        1. read data
        2. update data
        3. extract data and throw it into a filter
        4. get filtered data
        5. plot 3D
        */
    }
}
