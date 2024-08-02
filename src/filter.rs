use crate::imu::{Acceleration, AngularVelocity, Euler, MagneticField, Orientation};
use nalgebra as na;
use std::sync::mpsc::Receiver;

pub struct EkfFilter {
    receiver: Receiver<(
        Orientation,
        Euler,
        Acceleration,
        AngularVelocity,
        MagneticField,
    )>,
}
