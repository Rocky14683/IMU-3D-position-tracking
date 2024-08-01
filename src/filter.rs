use std::sync::mpsc::Receiver;
use nalgebra as na;
use crate::imu::{Acceleration, AngularVelocity, Euler, MagneticField, Orientation};

pub struct EkfFilter{
    receiver: Receiver<(Orientation, Euler, Acceleration, AngularVelocity, MagneticField)>,
}