#![allow(irrefutable_let_patterns)]
#![cfg(not(target_arch = "wasm32"))]

mod engine;

use std::{f32::consts, fs, mem, time};

fn default_vec() -> mint::Vector3<f32> {
    [0.0; 3].into()
}
fn default_scale() -> f32 {
    1.0
}

#[derive(serde::Deserialize)]
struct VisualConfig {
    model: String,
    #[serde(default = "default_vec")]
    pos: mint::Vector3<f32>,
    #[serde(default = "default_vec")]
    rot: mint::Vector3<f32>,
    #[serde(default = "default_scale")]
    scale: f32,
}
impl Default for VisualConfig {
    fn default() -> Self {
        Self {
            model: String::new(),
            pos: default_vec(),
            rot: default_vec(),
            scale: default_scale(),
        }
    }
}

#[derive(serde::Deserialize)]
enum Shape {
    Ball { radius: f32 },
    Cylinder { half_height: f32, radius: f32 },
    Cuboid { half: mint::Vector3<f32> },
    ConvexHull { points: Vec<mint::Vector3<f32>> },
}

#[derive(serde::Deserialize)]
struct ColliderConfig {
    mass: f32,
    shape: Shape,
    #[serde(default = "default_vec")]
    pos: mint::Vector3<f32>,
    #[serde(default = "default_vec")]
    rot: mint::Vector3<f32>,
}

#[derive(serde::Deserialize)]
pub struct ObjectConfig {
    #[serde(default)]
    name: String,
    visuals: Vec<VisualConfig>,
    colliders: Vec<ColliderConfig>,
}

#[derive(serde::Deserialize)]
struct VehiclePartConfig {
    visual: VisualConfig,
    collider: ColliderConfig,
}

#[derive(Default, serde::Deserialize)]
struct SteerConfig {
    max_angle: f32,
    stiffness: f32,
    damping: f32,
}

#[derive(serde::Deserialize)]
struct AxleConfig {
    z: f32,
    x: f32,
    #[serde(default)]
    steer: SteerConfig,
}

#[derive(serde::Deserialize)]
struct VehicleConfig {
    body: VehiclePartConfig,
    wheel: VehiclePartConfig,
    axles: Vec<AxleConfig>,
}

#[derive(serde::Deserialize)]
pub struct EngineConfig {
    shader_path: String,
}

#[derive(serde::Deserialize)]
struct LevelConfig {
    #[serde(default)]
    environment: String,
    gravity: f32,
    average_luminocity: f32,
    ground: ObjectConfig,
}

#[derive(serde::Deserialize)]
struct CameraConfig {
    azimuth: f32,
    altitude: f32,
    distance: f32,
    target: mint::Vector3<f32>,
    fov: f32,
}

#[derive(serde::Deserialize)]
struct GameConfig {
    engine: EngineConfig,
    level: LevelConfig,
    camera: CameraConfig,
    vehicle: String,
}

struct Wheel {
    _object: engine::ObjectHandle,
    joint: engine::JointHandle,
}

struct WheelAxle {
    left: Wheel,
    right: Wheel,
    steer: Option<SteerConfig>,
}

struct Vehicle {
    body_handle: engine::ObjectHandle,
    wheel_axles: Vec<WheelAxle>,
}

struct Game {
    // engine stuff
    engine: engine::Engine,
    last_physics_update: time::Instant,
    is_paused: Option<egui_gizmo::GizmoMode>,
    // windowing
    window: winit::window::Window,
    egui_state: egui_winit::State,
    egui_context: egui::Context,
    // game data
    _ground_handle: engine::ObjectHandle,
    vehicle: Vehicle,
    cam_config: CameraConfig,
}

impl Game {
    fn new(event_loop: &winit::event_loop::EventLoop<()>) -> Self {
        log::info!("Initializing");

        let window = winit::window::WindowBuilder::new()
            .with_title("RayCraft")
            .build(event_loop)
            .unwrap();

        let config: GameConfig =
            ron::de::from_bytes(&fs::read("data/config.ron").expect("Unable to open the config"))
                .expect("Unable to parse the config");

        let mut engine = engine::Engine::new(&window, &config.engine);
        engine.set_environment_map(&config.level.environment);
        engine.set_gravity(config.level.gravity);
        engine.set_average_luminosity(config.level.average_luminocity);

        let ground_handle = engine.add_object(
            &config.level.ground,
            nalgebra::Isometry3::default(),
            engine::BodyType::Fixed,
        );

        let veh_config: VehicleConfig = ron::de::from_bytes(
            &fs::read(format!("data/vehicles/{}.ron", config.vehicle))
                .expect("Unable to open the vehicle config"),
        )
        .expect("Unable to parse the vehicle config");
        let body_config = ObjectConfig {
            name: format!("{}/body", config.vehicle),
            visuals: vec![veh_config.body.visual],
            colliders: vec![veh_config.body.collider],
        };
        let init_pos = nalgebra::Vector3::new(0.0, 10.0, 0.0);
        let mut vehicle = Vehicle {
            body_handle: engine.add_object(
                &body_config,
                nalgebra::Isometry3::translation(init_pos.x, init_pos.y, init_pos.z),
                engine::BodyType::Dynamic,
            ),
            wheel_axles: Vec::new(),
        };
        let wheel_config = ObjectConfig {
            name: format!("{}/wheel", config.vehicle),
            visuals: vec![veh_config.wheel.visual],
            colliders: vec![veh_config.wheel.collider],
        };
        for ac in veh_config.axles {
            let offset_left = nalgebra::Vector3::new(ac.x, 0.0, ac.z);
            let offset_right = nalgebra::Vector3::new(-ac.x, 0.0, ac.z);

            let wheel_left = engine.add_object(
                &wheel_config,
                nalgebra::Isometry3::new(
                    init_pos + offset_left,
                    nalgebra::Vector3::y_axis().scale(consts::PI),
                ),
                engine::BodyType::Dynamic,
            );
            let wheel_right = engine.add_object(
                &wheel_config,
                nalgebra::Isometry3::new(init_pos + offset_right, nalgebra::Vector3::zeros()),
                engine::BodyType::Dynamic,
            );

            let has_steer = ac.steer.max_angle > 0.0;
            let max_angle = ac.steer.max_angle.to_radians();
            let locked_axes = if has_steer {
                rapier3d::dynamics::JointAxesMask::LIN_AXES
                    | rapier3d::dynamics::JointAxesMask::ANG_Z
            } else {
                rapier3d::dynamics::JointAxesMask::LOCKED_REVOLUTE_AXES
            };
            let joint_left = engine.add_joint(
                vehicle.body_handle,
                wheel_left,
                rapier3d::dynamics::GenericJointBuilder::new(locked_axes)
                    .contacts_enabled(false)
                    .local_anchor1(offset_left.into())
                    .local_frame2(nalgebra::Isometry3::rotation(
                        nalgebra::Vector3::y_axis().scale(consts::PI),
                    ))
                    .limits(rapier3d::dynamics::JointAxis::Y, [-max_angle, max_angle])
                    .motor_position(rapier3d::dynamics::JointAxis::X, 0.0, 1.0, 1.0)
                    .motor_position(
                        rapier3d::dynamics::JointAxis::Y,
                        0.0,
                        ac.steer.stiffness,
                        ac.steer.damping,
                    )
                    .build(),
            );
            let joint_right = engine.add_joint(
                vehicle.body_handle,
                wheel_right,
                rapier3d::dynamics::GenericJointBuilder::new(locked_axes)
                    .contacts_enabled(false)
                    .local_anchor1(offset_right.into())
                    .limits(rapier3d::dynamics::JointAxis::Y, [-max_angle, max_angle])
                    .motor_position(rapier3d::dynamics::JointAxis::X, 0.0, 1.0, 1.0)
                    .motor_position(
                        rapier3d::dynamics::JointAxis::Y,
                        0.0,
                        ac.steer.stiffness,
                        ac.steer.damping,
                    )
                    .build(),
            );

            vehicle.wheel_axles.push(WheelAxle {
                left: Wheel {
                    _object: wheel_left,
                    joint: joint_left,
                },
                right: Wheel {
                    _object: wheel_right,
                    joint: joint_right,
                },
                steer: if has_steer { Some(ac.steer) } else { None },
            });
        }

        Self {
            engine,
            last_physics_update: time::Instant::now(),
            is_paused: None,
            window,
            egui_state: egui_winit::State::new(event_loop),
            egui_context: egui::Context::default(),
            _ground_handle: ground_handle,
            vehicle,
            cam_config: config.camera,
        }
    }

    fn destroy(&mut self) {
        self.engine.destroy();
    }

    fn set_velocity(&mut self, velocity: f32) {
        for wax in self.vehicle.wheel_axles.iter() {
            for &joint_handle in &[wax.left.joint, wax.right.joint] {
                let joint = self.engine.get_joint_mut(joint_handle);
                joint
                    .data
                    .set_motor_velocity(rapier3d::dynamics::JointAxis::X, velocity, 100.0);
            }
        }
    }

    fn set_steering(&mut self, angle_rad: f32) {
        for wax in self.vehicle.wheel_axles.iter() {
            let steer = match wax.steer {
                Some(ref steer) => steer,
                None => continue,
            };
            for &joint_handle in &[wax.left.joint, wax.right.joint] {
                let joint = self.engine.get_joint_mut(joint_handle);
                joint.data.set_motor_position(
                    rapier3d::dynamics::JointAxis::Y,
                    angle_rad,
                    steer.stiffness,
                    steer.damping,
                );
            }
        }
    }

    fn on_event(&mut self, event: &winit::event::WindowEvent) -> bool {
        let response = self.egui_state.on_event(&self.egui_context, event);
        if response.consumed {
            return false;
        }
        if response.repaint {
            self.window.request_redraw();
        }

        match *event {
            winit::event::WindowEvent::KeyboardInput {
                input:
                    winit::event::KeyboardInput {
                        virtual_keycode: Some(key_code),
                        state: winit::event::ElementState::Pressed,
                        ..
                    },
                ..
            } => match key_code {
                winit::event::VirtualKeyCode::Escape => {
                    return true;
                }
                winit::event::VirtualKeyCode::Up => {
                    self.set_velocity(100.0);
                }
                winit::event::VirtualKeyCode::Down => {
                    self.set_velocity(-20.0);
                }
                winit::event::VirtualKeyCode::Left => {
                    self.set_steering(1.0);
                }
                winit::event::VirtualKeyCode::Right => {
                    self.set_steering(-1.0);
                }
                winit::event::VirtualKeyCode::Space => {
                    self.engine
                        .apply_impulse(self.vehicle.body_handle, [0.0, 10.0, 0.0].into());
                }
                _ => {}
            },
            winit::event::WindowEvent::KeyboardInput {
                input:
                    winit::event::KeyboardInput {
                        virtual_keycode: Some(key_code),
                        state: winit::event::ElementState::Released,
                        ..
                    },
                ..
            } => match key_code {
                winit::event::VirtualKeyCode::Up | winit::event::VirtualKeyCode::Down => {
                    self.set_velocity(0.0);
                }
                winit::event::VirtualKeyCode::Left | winit::event::VirtualKeyCode::Right => {
                    self.set_steering(0.0);
                }
                _ => {}
            },
            winit::event::WindowEvent::CloseRequested => {
                return true;
            }
            _ => {}
        }
        false
    }

    fn add_camera_manipulation(&mut self, ui: &mut egui::Ui) {
        let mode = match self.is_paused {
            Some(mode) => mode,
            None => return,
        };

        let cc = &self.cam_config;
        let eye_dir = nalgebra::Vector3::new(
            -cc.azimuth.sin() * cc.altitude.cos(),
            cc.altitude.sin(),
            -cc.azimuth.cos() * cc.altitude.cos(),
        );

        let rotation = {
            let z = eye_dir;
            //let x = z.cross(&nalgebra::Vector3::y_axis()).normalize();
            let x = nalgebra::Vector3::new(cc.azimuth.cos(), 0.0, -cc.azimuth.sin());
            //let y = z.cross(&x);
            let y = nalgebra::Vector3::new(
                cc.altitude.sin() * -cc.azimuth.sin(),
                -cc.altitude.cos(),
                cc.altitude.sin() * -cc.azimuth.cos(),
            );
            nalgebra::geometry::UnitQuaternion::from_rotation_matrix(
                &nalgebra::geometry::Rotation3::from_basis_unchecked(&[x, y, z]).transpose(),
            )
        };
        let view = {
            let t = rotation * (nalgebra::Vector3::from(cc.target) - eye_dir.scale(cc.distance));
            nalgebra::geometry::Isometry3::from_parts(t.into(), rotation)
        };

        let aspect = self.engine.screen_aspect();
        let depth_range = 1.0f32..10000.0; //TODO?
        let projection_matrix =
            nalgebra::Matrix4::new_perspective(aspect, cc.fov, depth_range.start, depth_range.end);

        let gizmo = egui_gizmo::Gizmo::new("Object")
            .model_matrix(view.to_homogeneous())
            .projection_matrix(projection_matrix)
            .mode(mode)
            .orientation(egui_gizmo::GizmoOrientation::Global)
            .snapping(true);

        if let Some(result) = gizmo.interact(ui) {
            let q = nalgebra::Unit::new_normalize(nalgebra::Quaternion::from(
                result.rotation.to_array(),
            ));
            let m = q.inverse().to_rotation_matrix();
            self.cam_config.azimuth = -m[(2, 0)].atan2(m[(0, 0)]);
            self.cam_config.altitude = (-m[(1, 1)]).acos();
            let t_local = q
                .inverse()
                .transform_vector(&nalgebra::Vector3::from(result.translation.to_array()));
            self.cam_config.target = (t_local + eye_dir.scale(self.cam_config.distance)).into();
        }
    }

    fn populate_hud(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Camera")
            .default_open(true)
            .show(ui, |ui| {
                ui.add(
                    egui::DragValue::new(&mut self.cam_config.distance)
                        .prefix("Distance")
                        .clamp_range(1.0..=100.0),
                );
                ui.horizontal(|ui| {
                    ui.selectable_value(
                        &mut self.is_paused,
                        Some(egui_gizmo::GizmoMode::Translate),
                        "Target",
                    );
                    ui.add(egui::DragValue::new(&mut self.cam_config.target.y));
                    ui.add(egui::DragValue::new(&mut self.cam_config.target.z));
                });
                ui.horizontal(|ui| {
                    let eps = 0.01;
                    ui.selectable_value(
                        &mut self.is_paused,
                        Some(egui_gizmo::GizmoMode::Rotate),
                        "Angle",
                    );
                    ui.add(
                        egui::DragValue::new(&mut self.cam_config.azimuth)
                            .clamp_range(-consts::FRAC_PI_2..=consts::FRAC_PI_2)
                            .speed(0.1),
                    );
                    ui.add(
                        egui::DragValue::new(&mut self.cam_config.altitude)
                            .clamp_range(eps..=consts::FRAC_PI_2 - eps)
                            .speed(0.1),
                    );
                });
                ui.add(egui::Slider::new(&mut self.cam_config.fov, 0.5f32..=2.0f32).text("FOV"));
                if self.is_paused.is_some() && ui.button("Unpause").clicked() {
                    self.is_paused = None;
                }
            });

        self.add_camera_manipulation(ui);
        self.engine.populate_hud(ui);
    }

    fn on_draw(&mut self) -> time::Duration {
        let raw_input = self.egui_state.take_egui_input(&self.window);
        let egui_context = mem::take(&mut self.egui_context);
        let egui_output = egui_context.run(raw_input, |egui_ctx| {
            let frame = {
                let mut frame = egui::Frame::side_top_panel(&egui_ctx.style());
                let mut fill = frame.fill.to_array();
                for f in fill.iter_mut() {
                    *f = (*f as u32 * 7 / 8) as u8;
                }
                frame.fill =
                    egui::Color32::from_rgba_premultiplied(fill[0], fill[1], fill[2], fill[3]);
                frame
            };
            egui::SidePanel::right("engine")
                .frame(frame)
                .show(egui_ctx, |ui| self.populate_hud(ui));
        });
        self.egui_context = egui_context;

        self.egui_state.handle_platform_output(
            &self.window,
            &self.egui_context,
            egui_output.platform_output,
        );
        let engine_dt = self.last_physics_update.elapsed().as_secs_f32();
        self.last_physics_update = time::Instant::now();
        if self.is_paused.is_none() {
            self.engine.update(engine_dt);
        }

        let camera = {
            let veh_isometry = self.engine.get_object_isometry(self.vehicle.body_handle);
            // Projection of the rotation of the vehicle on the Y axis
            let projection = veh_isometry
                .rotation
                .quaternion()
                .imag()
                .dot(&nalgebra::Vector3::y_axis());
            let base_quat =
                nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::from_parts(
                    veh_isometry.rotation.quaternion().w,
                    nalgebra::Vector3::y_axis().scale(projection.abs()),
                ));
            let base =
                nalgebra::geometry::Isometry3::from_parts(veh_isometry.translation, base_quat);
            //TODO: `nalgebra::Point3::from(mint::Vector3)` doesn't exist?
            let cc = &self.cam_config;
            let source = nalgebra::Vector3::from(cc.target)
                + nalgebra::Vector3::new(
                    -cc.azimuth.sin() * cc.altitude.cos(),
                    cc.altitude.sin(),
                    -cc.azimuth.cos() * cc.altitude.cos(),
                )
                .scale(cc.distance);
            let local = nalgebra::geometry::Isometry3::look_at_rh(
                &source.into(),
                &nalgebra::Vector3::from(cc.target).into(),
                &nalgebra::Vector3::y_axis(),
            );
            engine::Camera {
                isometry: base * local.inverse(),
                fov_y: cc.fov,
            }
        };

        let primitives = self.egui_context.tessellate(egui_output.shapes);
        self.engine.render(
            &camera,
            &primitives,
            &egui_output.textures_delta,
            self.window.inner_size(),
            self.egui_context.pixels_per_point(),
        );

        profiling::finish_frame!();
        egui_output.repaint_after
    }
}

fn main() {
    env_logger::init();
    //let _ = profiling::tracy_client::Client::start();

    let event_loop = winit::event_loop::EventLoop::new();
    let mut game = Game::new(&event_loop);
    let mut last_event = time::Instant::now();

    event_loop.run(move |event, _, control_flow| {
        *control_flow = winit::event_loop::ControlFlow::Poll;
        let _delta = last_event.elapsed().as_secs_f32();
        last_event = time::Instant::now();

        match event {
            winit::event::Event::RedrawEventsCleared => {
                game.window.request_redraw();
            }
            winit::event::Event::WindowEvent { event, .. } => {
                if game.on_event(&event) {
                    *control_flow = winit::event_loop::ControlFlow::Exit;
                }
            }
            winit::event::Event::RedrawRequested(_) => {
                let wait = game.on_draw();

                *control_flow = if let Some(repaint_after_instant) =
                    std::time::Instant::now().checked_add(wait)
                {
                    winit::event_loop::ControlFlow::WaitUntil(repaint_after_instant)
                } else {
                    winit::event_loop::ControlFlow::Wait
                };
            }
            winit::event::Event::LoopDestroyed => {
                game.destroy();
            }
            _ => {}
        }
    })
}
