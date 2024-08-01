use nalgebra as na;
use nalgebra::Quaternion;
use serde::*;
use serialport::*;
use std::io::{Read, Write};
use std::sync::Arc;
use std::{error, time};
use std::ops::Deref;
use std::sync::mpsc::Sender;

type Result<T> = std::result::Result<T, Box<dyn error::Error>>;

#[derive(Clone, Debug)]
pub struct Euler {
    roll: f32,
    pitch: f32,
    yaw: f32,
}

#[derive(Clone, Debug)]
pub struct Orientation {
    i: f32,
    j: f32,
    k: f32,
    w: f32,
}

#[derive(Clone, Debug)]
pub struct Acceleration {
    x: f32,
    y: f32,
    z: f32,
}

#[derive(Clone, Debug)]
pub struct AngularVelocity {
    x: f32,
    y: f32,
    z: f32,
}

#[derive(Clone, Debug)]
pub struct MagneticField {
    x: f32,
    y: f32,
    z: f32,
}

pub struct IMU {
    pub port: Box<dyn SerialPort>,
    sender: Sender<(Orientation, Euler, Acceleration, AngularVelocity, MagneticField)>,
    buffer: Vec<u8>,
    quaternion: Arc<Orientation>,
    euler: Arc<Euler>,
    acceleration: Arc<Acceleration>,
    angular_velocity: Arc<AngularVelocity>,
    magnetic_field: Arc<MagneticField>,
}

const G: f32 = 9.81;

impl IMU {
    pub fn new(port_name: &str, sender: Sender<(Orientation, Euler, Acceleration, AngularVelocity, MagneticField)>) -> Self {
        let mut imu = IMU {
            port: serialport::new(port_name, 921600)
                .open()
                .expect("Failed to open port"),
            sender,
            buffer: Vec::new(),
            quaternion: Arc::new(Orientation {
                i: 0.0,
                j: 0.0,
                k: 0.0,
                w: 0.0,
            }),
            euler: Arc::new(Euler {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
            }),
            acceleration: Arc::new(Acceleration {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
            angular_velocity: Arc::new(AngularVelocity {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
            magnetic_field: Arc::new(MagneticField {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
        };

        imu.port
            .set_timeout(std::time::Duration::from_millis(500))
            .expect("Failed to set timeout");
        imu.port.flush().unwrap();

        imu
    }

    pub fn update_loop(&mut self) {
        self.port.flush().unwrap();
        loop {
            let to_read_len = self.port.bytes_to_read().unwrap();
            if to_read_len > 0 {
                let mut temp_buf: Vec<u8> = vec![0; self.port.bytes_to_read().unwrap() as usize]; // this will always be re-init
                self.port.read(&mut temp_buf).unwrap();

                for data in temp_buf {
                    self.buffer.push(data);
                    // println!("{:?}", self.buffer);

                    if self.buffer.len() > 0 {
                        if self.buffer[0] != 0xaa {
                            self.buffer.clear();
                            continue;
                        }
                    }

                    if self.buffer.len() > 2 {
                        if self.buffer[1] != 0x55 || !(self.buffer[2] == 20 || self.buffer[2] == 44) {
                            self.buffer.clear();
                            continue;
                        }
                    } else {
                        continue;
                    }

                    if self.buffer.len() < self.buffer[2] as usize + 5 {
                        continue;
                    }

                    if Self::check_sum(
                        &self.buffer[2..(self.buffer.len() - 2)],
                        &self.buffer[(self.buffer.len() - 2)..self.buffer.len()],
                    ) {
                        let len = self.buffer[2];
                        let total_buf_len = self.buffer.len();

                        if len == 20 {
                            let data =
                                Self::hex_to_ieee(self.buffer[7..total_buf_len - 2].to_vec());
                            // println!("{:?}", data);

                            let (q, euler) = self.decode_angle_data(&data).unwrap();
                            self.quaternion = Arc::new(q);
                            self.euler = Arc::new(euler);
                        } else if len == 44 {
                            let data =
                                Self::hex_to_ieee(self.buffer[7..total_buf_len - 2].to_vec());
                            // println!("{:?}", data);

                            let (w, a, mag) = self.decode_9dof_data(&data).unwrap();
                            self.angular_velocity = Arc::new(w);
                            self.acceleration = Arc::new(a);
                            self.magnetic_field = Arc::new(mag);
                        } else {
                            println!("Unknown packet length");
                            self.buffer.clear();
                            continue;
                        }


                        self.sender.send((self.quaternion.deref().clone(),
                                              self.euler.deref().clone(),
                                              self.acceleration.deref().clone(),
                                              self.angular_velocity.deref().clone(),
                                              self.magnetic_field.deref().clone())).unwrap();

                        self.buffer.clear();
                    } else {
                        println!("Checksum failed");
                        self.buffer.clear();
                        continue;
                    }
                }
            }
            std::thread::sleep(time::Duration::from_millis(100));
        }
    }

    fn check_sum(list_data: &[u8], check_data: &[u8]) -> bool {
        let mut crc: u16 = 0xFFFF;
        for &pos in list_data {
            crc ^= pos as u16;
            for _ in 0..8 {
                if (crc & 1) != 0 {
                    crc >>= 1;
                    crc ^= 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        ((crc & 0xff) << 8 | crc >> 8) == ((check_data[0] as u16) << 8 | check_data[1] as u16)
    }

    fn hex_to_ieee(mut raw_data: Vec<u8>) -> Vec<f32> {
        raw_data.reverse();
        let mut ieee_data = Vec::new();
        for chunk in raw_data.chunks(4) {
            let data2str = format!(
                "{:02x}{:02x}{:02x}{:02x}",
                chunk[0], chunk[1], chunk[2], chunk[3]
            );
            ieee_data.push(f32::from_be_bytes(
                hex::decode(data2str).unwrap().try_into().unwrap(),
            ));
        }
        ieee_data.reverse();
        ieee_data
    }

    fn decode_9dof_data(
        &mut self,
        data: &Vec<f32>,
    ) -> Result<(AngularVelocity, Acceleration, MagneticField)> {
        let w = AngularVelocity {
            x: data[0],
            y: data[1],
            z: data[2],
        };

        let a = Acceleration {
            x: data[3] * -G,
            y: data[4] * -G,
            z: data[5] * -G,
        };

        let mag = MagneticField {
            x: data[6],
            y: data[7],
            z: data[8],
        };

        // println!("{:?}\n {:?}\n {:?}\n", w, a, mag);
        Ok((w, a, mag))
    }

    fn decode_angle_data(&mut self, data: &Vec<f32>) -> Result<(Orientation, Euler)> {
        let angle_radian: Vec<f32> = data[0..4].iter().map(|&x| f32::to_radians(x)).collect();

        let quaternion = Orientation {
            i: angle_radian[0],
            j: angle_radian[1],
            k: angle_radian[2],
            w: angle_radian[3],
        };

        let na_quat = Quaternion::new(quaternion.w, quaternion.i, quaternion.j, quaternion.k);

        let euler = na::geometry::UnitQuaternion::from_quaternion(na_quat).euler_angles();

        let euler = Euler {
            roll: euler.0,
            pitch: euler.1,
            yaw: euler.2,
        };

        // println!("{:?}\n {:?}", quaternion, euler);
        Ok((quaternion, euler))
    }
}
