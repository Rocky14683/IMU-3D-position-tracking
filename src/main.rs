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

    let (tx, rx) = mpsc::channel();

    let mut imu = Arc::new(Mutex::new(IMU::new(
        if args.len() > 1 {
            &args[1]
        } else {
            "/dev/tty.usbserial-0001"
        },
        tx,
    )));

    let mut imu_clone = Arc::clone(&imu);
    std::thread::spawn(move || {
        imu_clone.lock().unwrap().update_loop();
    });


    loop {
        match rx.recv() {
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
