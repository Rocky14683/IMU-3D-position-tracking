mod filter;
mod imu;
mod pose3d;

pub use nalgebra as na;
pub use serde::*;
use std::env;
use std::process::{exit, ExitCode, Termination};
use std::sync::{Arc, mpsc, Mutex};
use imu::*;

fn main() {
    let args: Vec<String> = env::args().collect();

    let (tx, rx) = mpsc::channel();
    let (filter_tx, filter_rx) = mpsc::channel();

    let mut imu = Arc::new(Mutex::new(IMU::new(if args.len() > 1 {
        &args[1]
    } else {
        "/dev/tty.usbserial-0001"
    }, tx)));


    let mut imu_clone = Arc::clone(&imu);
    std::thread::spawn(move || {
        imu_clone.lock().unwrap().update_loop();
    });

    // std::thread::spawn(move || {
    //     filter::filter();
    // });


    loop {
        match rx.recv() {
            Ok((quaternion,
                   euler,
                   acceleration,
                   angular_velocity,
                   magnetic_field)) => {
                println!("Quaternion: {:?}", quaternion);
                println!("Euler: {:?}", euler);
                println!("Acceleration: {:?}", acceleration);
                println!("Angular Velocity: {:?}", angular_velocity);
                println!("Magnetic Field: {:?}", magnetic_field);

                filter_tx.send((quaternion,
                                euler,
                                acceleration,
                                angular_velocity,
                                magnetic_field)).unwrap();
            }
            Err(e) => {
                println!("Error receiving data: {:?}", e);
                break;
            }
        }
    }
}
