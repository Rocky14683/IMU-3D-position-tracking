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
use nalgebra::geometry::*;
use nalgebra::Vector3;

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

                let arrow = vec![
                    Vector3::new(2.0, 0.0, 0.0),
                    Vector3::new(0.0, 2.0, 0.0),
                    Vector3::new(0.0, 0.0, 2.0),
                ];



                let rotation_matrix = Rotation::from_euler_angles(euler.roll, euler.pitch, euler.yaw);

                let rotated_arrow: Vec<Vector3D> = arrow.iter()
                    .map(|v| {
                        let rotated_vec = rotation_matrix.transform_vector(v);
                        Vector3D::from([rotated_vec.x, rotated_vec.y, rotated_vec.z])
                    })
                    .collect();


                rec.log("axes",
                        &rerun::Arrows3D::from_vectors(rotated_arrow).with_colors(vec![
                            Color::from_unmultiplied_rgba(255, 0, 0, 255), // Red for X axis
                            Color::from_unmultiplied_rgba(0, 255, 0, 255), // Green for Y axis
                            Color::from_unmultiplied_rgba(0, 0, 255, 255), // Blue for Z axis
                        ]).with_radii(vec![
                            0.05, // Radius of X axis
                            0.05, // Radius of Y axis
                            0.05, // Radius of Z axis
                        ]).with_labels(vec![
                            "X".to_string(), // Label for X axis
                            "Y".to_string(), // Label for Y axis
                            "Z".to_string(), // Label for Z axis
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
