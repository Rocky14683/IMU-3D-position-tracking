use serial::{*};
use serde::{*};


struct Euler {
    roll: f32,
    pitch: f32,
    yaw: f32,
}

struct Acceleration {
    x: f32,
    y: f32,
    z: f32,
}

pub struct IMU {
    port: * SystemPort,
    eular: Euler,
    acceleration: Acceleration
}


impl IMU {
    pub fn new(port: &SystemPort) -> Self {
        IMU {
            port,
            eular: Euler {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
            },
            acceleration: Acceleration {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }
        }
    }
}