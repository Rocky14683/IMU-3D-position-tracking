use crate::imu::{Acceleration, AngularVelocity, Euler, MagneticField, Orientation};
use nalgebra as na;
use std::sync::mpsc::Receiver;

const SAMPLE_RATE_HZ: f32 = 1000.0;
pub struct EkfFilter {
    receiver: Receiver<(
        Orientation,
        Euler,
        Acceleration,
        AngularVelocity,
        MagneticField,
    )>,

}
