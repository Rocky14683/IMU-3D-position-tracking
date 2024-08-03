mod filter;
mod imu;
mod pose3d;

use imu::*;
use serde::*;
use std::{env, thread};
use std::process::{exit, ExitCode, Termination};
use std::sync::{mpsc, Arc, Mutex};
use rerun::*;
use rerun::external::glam;
use rerun::external::glam::quat;
use three_d::Quat;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let rec = RecordingStreamBuilder::new("imu_orientation_visualization").spawn().unwrap();

    let args: Vec<String> = env::args().collect();

    let (imu_tx, imu_rx) = mpsc::channel();

    let (filter_tx, filter_rx) = mpsc::channel();

    let mut imu = Arc::new(Mutex::new(IMU::new(
        if args.len() > 1 {
            &args[1]
        } else {
            "/dev/tty.usbserial-0001"
        },
        imu_tx,
    )));

    std::thread::spawn(move || {
        imu.lock().unwrap().update_loop();
    });


    let mut filter = Arc::new(Mutex::new(filter::EkfFilter::new(imu_rx, filter_tx)));

    std::thread::spawn(move || {
        filter.lock().unwrap().update_loop();
    });






    loop {
        match imu_rx.recv() {
            Ok((
                   quaternion,
                   euler,
                   a,
                   w,
                   m,
               )) => {

                rec.log("box", &Boxes3D::from_centers_and_sizes(
                    [(0.0, 0.0, 0.0)],
                    [(2.0, 2.0, 1.0)],
                ).with_rotations([
                    <rerun::Quaternion as Into<crate::components::Rotation3D>>::into(rerun::Quaternion::from_xyzw([quaternion.i, quaternion.j, quaternion.k, quaternion.w])), // 45 degrees around Z
                ])).unwrap();

            }
            Err(e) => {
                println!("Error receiving data: {:?}", e);
                break;
            }
        }
    }

    Ok(())
}
