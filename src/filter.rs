use crate::imu::{Acceleration, AngularVelocity, Euler, MagneticField, Orientation};
use nalgebra as na;
use std::sync::mpsc::{Receiver, Sender};
use nalgebra::{Quaternion, UnitQuaternion};
use rerun::external::uuid::Timestamp;
use rerun::Time;
use crate::imu;

const G: f32 = 9.81;

const SAMPLE_RATE_HZ: f32 = 1000.0;

pub struct Integration {
    receiver: Receiver<(
        Orientation,
        Euler,
        Acceleration,
        AngularVelocity,
        MagneticField,
    )>,
    sender: Sender<(na::Vector3<f32>, na::Vector3<f32>)>,
    position: na::Vector3<f32>,
    velocity: na::Vector3<f32>,
    prev_time: Time,
}

impl Integration {
    pub fn new(receiver: Receiver<(
        Orientation,
        Euler,
        Acceleration,
        AngularVelocity,
        MagneticField,
    )>, sender: Sender<(na::Vector3<f32>, na::Vector3<f32>)>) -> Self {
        Self {
            receiver,
            sender,
            position: na::Vector3::zeros(),
            velocity: na::Vector3::zeros(),
            prev_time: Time::now(),
        }
    }

    pub fn run(&mut self) {
        loop {
            match self.receiver.recv() {
                Ok((orientation, euler, acceleration, angular_velocity, m)) => {
                    let start_t = Time::now();
                    let dt = start_t - self.prev_time;
                    let rotation = na::Rotation::from(UnitQuaternion::from_euler_angles(euler.roll, euler.pitch, euler.yaw));


                    let acceleration = na::Vector3::new(acceleration.x, acceleration.y, acceleration.z) * dt.as_secs_f32(); // meter/ms^2
                    // println!("Acceleration: {:?}", acceleration);

                    self.velocity += rotation.transform_vector(&(acceleration));
                    // println!("Velocity: {:?}", self.velocity);



                    self.position += self.velocity * dt.as_secs_f32();

                    self.prev_time = start_t;

                    self.sender.send((self.position, self.velocity)).unwrap();
                }
                Err(_) => {
                    println!("Error in Integration");
                }
            }
        }
    }
}


pub struct Preintegration {
    receiver: Receiver<(
        Orientation,
        Euler,
        Acceleration,
        AngularVelocity,
    )>,
}
