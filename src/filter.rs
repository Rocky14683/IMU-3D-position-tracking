use crate::imu::*;
use nalgebra as na;
use std::sync::mpsc::{Receiver, Sender};
use plotters::style::text_anchor::Pos;
use crate::pose3d::*;

const SAMPLE_RATE_HZ: f32 = 1000.0;

struct EkfValue {
    gn: na::Vector3<f32>,
    g0: f32,
    mn: na::Vector3<f32>,
    gyro_noise: f32,
    gyro_bias: na::Vector3<f32>,
    acc_noise: f32,
    mag_noise: f32,
}

struct EkfRequired {
    pub accel: Acceleration,
    pub angular_v: AngularVelocity,
    pub mag: MagneticField,
}

struct Velocity {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl EkfRequired {
    fn new(accel: Acceleration, angular_v: AngularVelocity, mag: MagneticField) -> Self {
        Self {
            accel,
            angular_v,
            mag,
        }
    }
}


pub struct EkfFilter {
    rx: Receiver<(
        Orientation,
        Euler,
        Acceleration,
        AngularVelocity,
        MagneticField,
    )>,
    tx: Sender<Pose3D>,
    orientation: Orientation,
    euler: Euler,
    accel: Acceleration,
    angular_v: AngularVelocity,
    mag: MagneticField,

}


impl EkfFilter {
    pub fn new(rx: Receiver<(
        Orientation,
        Euler,
        Acceleration,
        AngularVelocity,
        MagneticField,
    )>, tx: Sender<Pose3D>) -> Self {
        Self {
            rx,
            tx,
            orientation: Orientation {
                i: 0.0,
                j: 0.0,
                k: 0.0,
                w: 1.0,
            },
            euler: Euler {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
            },
            accel: Acceleration {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angular_v: AngularVelocity {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            mag: MagneticField {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        }
    }


    pub fn update_loop(&mut self) {
        let mut init_data: Vec<EkfRequired> = Vec::new();
        loop {
            match self.rx.recv() {
                Ok((orientation, euler, accel, angular_v, mag)) => {
                    init_data.push(EkfRequired::new(accel, angular_v, mag));

                    if init_data.len() < 50 {
                        println!("initializing filter....");
                        continue;
                    }


                    // self.tx.send(pose).unwrap();
                }
                Err(e) => {
                    println!("Error receiving data: {:?}", e);
                    break;
                }
            }
        }
    }




}