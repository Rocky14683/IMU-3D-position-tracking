mod filter;
mod imu;
mod pose3d;

use imu::*;
pub use serde::*;
use std::env;
use std::process::{exit, ExitCode, Termination};
use std::sync::{mpsc, Arc, Mutex};
use three_d::*;
use three_d::ColorTexture::CubeMap;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let window = Window::new(WindowSettings {
        title: "Shapes!".to_string(),
        max_size: Some((1280, 720)),
        ..Default::default()
    }).unwrap();

    let context = window.gl();

    let mut camera = Camera::new_perspective(
        window.viewport(),
        vec3(5.0, 0.0, 0.0),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 5.0, 0.0),
        degrees(100.0),
        0.1,
        1000.0,
    );

    let mut control = OrbitControl::new(*camera.target(), 1.0, 100.0);

    let mut axes = Axes::new(&context, 0.05, 3.0);

    let mut cube = Gm::new(
        Mesh::new(&context, &CpuMesh::cube()),
        PhysicalMaterial::new_transparent(
            &context,
            &CpuMaterial {
                albedo: Srgba {
                    r: 0,
                    g: 0,
                    b: 255,
                    a: 240,
                },
                ..Default::default()
            },
        ),
    );

    let mut bounding_box_cube = Gm::new(
        BoundingBox::new(&context, cube.aabb()),
        ColorMaterial {
            color: Srgba::BLACK,
            ..Default::default()
        },
    );



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

    // std::thread::spawn(move || {
    //     filter::filter();
    // });
    let light0 = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, -0.5, -0.5));
    let light1 = DirectionalLight::new(&context, 1.0, Srgba::WHITE, &vec3(0.0, 0.5, 0.5));

    window.render_loop(move |mut frame_input| {
        camera.set_viewport(frame_input.viewport);
        control.handle_events(&mut camera, &mut frame_input.events);

        if let Ok((quaternion, euler, acceleration, angular_velocity, magnetic_field)) = rx.try_recv() {
            let rotation_matrix = euler.to_rotation_mat();
            axes.set_transformation(rotation_matrix);
            cube.set_transformation(rotation_matrix);
            bounding_box_cube.set_transformation(rotation_matrix);
        }

        frame_input
            .screen()
            .clear(ClearState::color_and_depth(0.8, 0.8, 0.8, 1.0, 1.0))
            .render(
                &camera,
                axes.into_iter().chain(&cube).chain(&bounding_box_cube),
                &[&light0, &light1],
            );


        FrameOutput::default()
    });

    Ok(())
}
