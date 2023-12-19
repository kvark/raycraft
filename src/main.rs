use std::{f32::consts, fs, mem, time};

#[derive(serde::Deserialize)]
struct VehiclePartConfig {
    visual: blade::config::Visual,
    collider: blade::config::Collider,
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
    drive_factor: f32,
    jump_impulse: f32,
    roll_impulse: f32,
    axles: Vec<AxleConfig>,
}

#[derive(serde::Deserialize)]
struct LevelConfig {
    #[serde(default)]
    environment: String,
    gravity: f32,
    average_luminocity: f32,
    spawn_pos: [f32; 3],
    ground: blade::config::Object,
}

#[derive(serde::Deserialize)]
struct CameraConfig {
    azimuth: f32,
    altitude: f32,
    distance: f32,
    speed: f32,
    target: [f32; 3],
    fov: f32,
}

#[derive(serde::Deserialize)]
struct GameConfig {
    engine: blade::config::Engine,
    camera: CameraConfig,
    level: String,
    vehicle: String,
}

struct Wheel {
    object: blade::ObjectHandle,
    spin_joint: blade::JointHandle,
    steer_joint: Option<blade::JointHandle>,
}

struct WheelAxle {
    left: Wheel,
    right: Wheel,
    steer: Option<SteerConfig>,
}

struct Vehicle {
    body_handle: blade::ObjectHandle,
    drive_factor: f32,
    jump_impulse: f32,
    roll_impulse: f32,
    wheel_axles: Vec<WheelAxle>,
}

struct Game {
    // engine stuff
    engine: blade::Engine,
    last_physics_update: time::Instant,
    last_camera_update: time::Instant,
    last_camera_base_quat: nalgebra::UnitQuaternion<f32>,
    is_paused: bool,
    // windowing
    window: winit::window::Window,
    egui_state: egui_winit::State,
    egui_context: egui::Context,
    // game data
    _ground_handle: blade::ObjectHandle,
    vehicle: Vehicle,
    cam_config: CameraConfig,
    spawn_pos: nalgebra::Vector3<f32>,
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
        let mut engine = blade::Engine::new(&window, &config.engine);

        let lev_config: LevelConfig = ron::de::from_bytes(
            &fs::read(format!("data/levels/{}.ron", config.level))
                .expect("Unable to open the level config"),
        )
        .expect("Unable to parse the level config");
        engine.set_environment_map(&lev_config.environment);
        engine.set_gravity(lev_config.gravity);
        engine.set_average_luminosity(lev_config.average_luminocity);

        let ground_handle = engine.add_object(
            &lev_config.ground,
            nalgebra::Isometry3::default(),
            blade::BodyType::Fixed,
        );

        let veh_config: VehicleConfig = ron::de::from_bytes(
            &fs::read(format!("data/vehicles/{}.ron", config.vehicle))
                .expect("Unable to open the vehicle config"),
        )
        .expect("Unable to parse the vehicle config");
        let body_config = blade::config::Object {
            name: format!("{}/body", config.vehicle),
            visuals: vec![veh_config.body.visual],
            colliders: vec![veh_config.body.collider],
            additional_mass: 0.0,
        };
        let init_pos = nalgebra::Vector3::from(lev_config.spawn_pos);
        let mut vehicle = Vehicle {
            body_handle: engine.add_object(
                &body_config,
                nalgebra::Isometry3::new(init_pos, nalgebra::Vector3::zeros()),
                blade::BodyType::Dynamic,
            ),
            drive_factor: veh_config.drive_factor,
            jump_impulse: veh_config.jump_impulse,
            roll_impulse: veh_config.roll_impulse,
            wheel_axles: Vec::new(),
        };
        let wheel_config = blade::config::Object {
            name: format!("{}/wheel", config.vehicle),
            visuals: vec![veh_config.wheel.visual],
            colliders: vec![veh_config.wheel.collider],
            additional_mass: 0.0,
        };
        let axle_config = blade::config::Object {
            name: format!("{}/axle", config.vehicle),
            visuals: vec![],
            colliders: vec![],
            additional_mass: 1.0,
        };
        //Note: in the vehicle coordinate system X=left, Y=up, Z=forward
        for ac in veh_config.axles {
            let offset_left = nalgebra::Vector3::new(ac.x, 0.0, ac.z);
            let offset_right = nalgebra::Vector3::new(-ac.x, 0.0, ac.z);
            let rotation_left = nalgebra::Vector3::z_axis().scale(consts::PI);
            let rotation_right = nalgebra::Vector3::zeros();

            let wheel_left = engine.add_object(
                &wheel_config,
                nalgebra::Isometry3::new(init_pos + offset_left, rotation_left),
                blade::BodyType::Dynamic,
            );
            let wheel_right = engine.add_object(
                &wheel_config,
                nalgebra::Isometry3::new(init_pos + offset_right, rotation_right),
                blade::BodyType::Dynamic,
            );
            let joint_kind = blade::JointKind::Soft;

            let wheel_axle = if ac.steer.max_angle > 0.0 {
                let max_angle = ac.steer.max_angle.to_radians();
                let axle_left = engine.add_object(
                    &axle_config,
                    nalgebra::Isometry3::new(init_pos + offset_left, nalgebra::Vector3::zeros()),
                    blade::BodyType::Dynamic,
                );
                let axle_right = engine.add_object(
                    &axle_config,
                    nalgebra::Isometry3::new(init_pos + offset_right, nalgebra::Vector3::zeros()),
                    blade::BodyType::Dynamic,
                );

                let axle_joint_left = engine.add_joint(
                    vehicle.body_handle,
                    axle_left,
                    rapier3d::dynamics::RevoluteJointBuilder::new(nalgebra::Vector3::y_axis())
                        .contacts_enabled(false)
                        .local_anchor1(offset_left.into())
                        .limits([-max_angle, max_angle])
                        .motor_position(0.0, ac.steer.stiffness, ac.steer.damping)
                        .build(),
                    joint_kind,
                );
                let axle_joint_right = engine.add_joint(
                    vehicle.body_handle,
                    axle_right,
                    rapier3d::dynamics::RevoluteJointBuilder::new(nalgebra::Vector3::y_axis())
                        .contacts_enabled(false)
                        .local_anchor1(offset_right.into())
                        .limits([-max_angle, max_angle])
                        .motor_position(0.0, ac.steer.stiffness, ac.steer.damping)
                        .build(),
                    joint_kind,
                );

                let wheel_joint_left = engine.add_joint(
                    axle_left,
                    wheel_left,
                    rapier3d::dynamics::GenericJointBuilder::new(
                        rapier3d::dynamics::JointAxesMask::LOCKED_REVOLUTE_AXES,
                    )
                    .contacts_enabled(false)
                    .local_frame2(nalgebra::Isometry3::rotation(rotation_left))
                    .build(),
                    joint_kind,
                );
                let wheel_joint_right = engine.add_joint(
                    axle_right,
                    wheel_right,
                    rapier3d::dynamics::GenericJointBuilder::new(
                        rapier3d::dynamics::JointAxesMask::LOCKED_REVOLUTE_AXES,
                    )
                    .contacts_enabled(false)
                    .local_frame2(nalgebra::Isometry3::rotation(rotation_right))
                    .build(),
                    joint_kind,
                );
                let _support_left = engine.add_joint(
                    vehicle.body_handle,
                    wheel_left,
                    rapier3d::dynamics::GenericJoint {
                        contacts_enabled: false,
                        ..Default::default()
                    },
                    joint_kind,
                );
                let _support_right = engine.add_joint(
                    vehicle.body_handle,
                    wheel_right,
                    rapier3d::dynamics::GenericJoint {
                        contacts_enabled: false,
                        ..Default::default()
                    },
                    joint_kind,
                );

                WheelAxle {
                    left: Wheel {
                        object: wheel_left,
                        spin_joint: wheel_joint_left,
                        steer_joint: Some(axle_joint_left),
                    },
                    right: Wheel {
                        object: wheel_right,
                        spin_joint: wheel_joint_right,
                        steer_joint: Some(axle_joint_right),
                    },
                    steer: Some(ac.steer),
                }
            } else {
                let joint_left = engine.add_joint(
                    vehicle.body_handle,
                    wheel_left,
                    rapier3d::dynamics::GenericJointBuilder::new(
                        rapier3d::dynamics::JointAxesMask::LOCKED_REVOLUTE_AXES,
                    )
                    .contacts_enabled(false)
                    .local_anchor1(offset_left.into())
                    .local_frame2(nalgebra::Isometry3::rotation(rotation_left))
                    .build(),
                    joint_kind,
                );
                let joint_right = engine.add_joint(
                    vehicle.body_handle,
                    wheel_right,
                    rapier3d::dynamics::GenericJointBuilder::new(
                        rapier3d::dynamics::JointAxesMask::LOCKED_REVOLUTE_AXES,
                    )
                    .contacts_enabled(false)
                    .local_anchor1(offset_right.into())
                    .local_frame2(nalgebra::Isometry3::rotation(rotation_right))
                    .build(),
                    joint_kind,
                );

                WheelAxle {
                    left: Wheel {
                        object: wheel_left,
                        spin_joint: joint_left,
                        steer_joint: None,
                    },
                    right: Wheel {
                        object: wheel_right,
                        spin_joint: joint_right,
                        steer_joint: None,
                    },
                    steer: None,
                }
            };
            vehicle.wheel_axles.push(wheel_axle);
        }

        Self {
            engine,
            last_physics_update: time::Instant::now(),
            last_camera_update: time::Instant::now(),
            last_camera_base_quat: Default::default(),
            is_paused: false,
            window,
            egui_state: egui_winit::State::new(event_loop),
            egui_context: egui::Context::default(),
            _ground_handle: ground_handle,
            vehicle,
            cam_config: config.camera,
            spawn_pos: init_pos,
        }
    }

    fn destroy(&mut self) {
        self.engine.destroy();
    }

    fn set_velocity(&mut self, velocity: f32) {
        self.engine.wake_up(self.vehicle.body_handle);
        self.update_time();
        for wax in self.vehicle.wheel_axles.iter() {
            for &joint_handle in &[wax.left.spin_joint, wax.right.spin_joint] {
                self.engine[joint_handle].set_motor_velocity(
                    rapier3d::dynamics::JointAxis::AngX,
                    velocity,
                    self.vehicle.drive_factor,
                );
            }
        }
    }

    fn set_steering(&mut self, angle_rad: f32) {
        self.update_time();
        for wax in self.vehicle.wheel_axles.iter() {
            let steer = match wax.steer {
                Some(ref steer) => steer,
                None => continue,
            };
            for maybe_joint in &[wax.left.steer_joint, wax.right.steer_joint] {
                if let Some(handle) = *maybe_joint {
                    self.engine[handle].set_motor_position(
                        rapier3d::dynamics::JointAxis::AngX,
                        angle_rad,
                        steer.stiffness,
                        steer.damping,
                    );
                }
            }
        }
    }

    fn teleport(&mut self, position: nalgebra::Vector3<f32>) {
        let old_isometry_inv = self
            .engine
            .get_object_isometry_approx(self.vehicle.body_handle)
            .inverse();
        let new_isometry = nalgebra::Isometry3 {
            rotation: Default::default(),
            translation: position.into(),
        };
        self.engine
            .teleport_object(self.vehicle.body_handle, new_isometry);
        for wax in self.vehicle.wheel_axles.iter() {
            for wheel_object in [wax.left.object, wax.right.object].into_iter() {
                let prev = self.engine.get_object_isometry_approx(wheel_object);
                let next = new_isometry * old_isometry_inv * prev;
                self.engine.teleport_object(wheel_object, next);
            }
        }
    }

    fn update_time(&mut self) {
        let engine_dt = self.last_physics_update.elapsed().as_secs_f32();
        self.last_physics_update = time::Instant::now();
        if !self.is_paused {
            //self.align_wheels();
            self.engine.update(engine_dt);
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
                winit::event::VirtualKeyCode::Comma => {
                    let forward = self
                        .engine
                        .get_object_isometry_approx(self.vehicle.body_handle)
                        .transform_vector(&nalgebra::Vector3::z_axis());
                    self.engine.apply_torque_impulse(
                        self.vehicle.body_handle,
                        -self.vehicle.roll_impulse * forward,
                    );
                }
                winit::event::VirtualKeyCode::Period => {
                    let forward = self
                        .engine
                        .get_object_isometry_approx(self.vehicle.body_handle)
                        .transform_vector(&nalgebra::Vector3::z_axis());
                    self.engine.apply_torque_impulse(
                        self.vehicle.body_handle,
                        self.vehicle.roll_impulse * forward,
                    );
                }
                winit::event::VirtualKeyCode::Space => {
                    let mut up = self
                        .engine
                        .get_object_isometry_approx(self.vehicle.body_handle)
                        .transform_vector(&nalgebra::Vector3::y_axis());
                    up.y = up.y.abs();
                    self.engine
                        .apply_impulse(self.vehicle.body_handle, self.vehicle.jump_impulse * up);
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

    fn populate_hud(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Camera")
            .default_open(true)
            .show(ui, |ui| {
                ui.add(
                    egui::Slider::new(&mut self.cam_config.distance, 1.0..=1000.0)
                        .text("Distance")
                        .logarithmic(true),
                );
                ui.horizontal(|ui| {
                    ui.label("Target");
                    ui.add(egui::DragValue::new(&mut self.cam_config.target[1]));
                    ui.add(egui::DragValue::new(&mut self.cam_config.target[2]));
                });
                ui.horizontal(|ui| {
                    let eps = 0.01;
                    ui.label("Angle");
                    ui.add(
                        egui::DragValue::new(&mut self.cam_config.azimuth)
                            .clamp_range(-consts::PI..=consts::PI)
                            .speed(0.1),
                    );
                    ui.add(
                        egui::DragValue::new(&mut self.cam_config.altitude)
                            .clamp_range(eps..=consts::FRAC_PI_2 - eps)
                            .speed(0.1),
                    );
                });
                ui.add(egui::Slider::new(&mut self.cam_config.fov, 0.5f32..=2.0f32).text("FOV"));
                ui.add(
                    egui::Slider::new(&mut self.cam_config.speed, 0.0..=1.0).text("Rotate speed"),
                );
                ui.toggle_value(&mut self.is_paused, "Pause");
            });

        egui::CollapsingHeader::new("Dynamics")
            .default_open(true)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Spawn pos");
                    ui.add(egui::DragValue::new(&mut self.spawn_pos.x));
                    ui.add(egui::DragValue::new(&mut self.spawn_pos.y));
                    ui.add(egui::DragValue::new(&mut self.spawn_pos.z));
                });
                ui.horizontal(|ui| {
                    if ui.button("Recover").clicked() {
                        let pos = self
                            .engine
                            .get_object_isometry_approx(self.vehicle.body_handle)
                            .translation
                            .vector;
                        self.teleport(pos + nalgebra::Vector3::new(0.0, 20.0, 0.0));
                    }
                    if ui.button("Respawn").clicked() {
                        self.teleport(self.spawn_pos);
                    }
                });
                ui.add(
                    egui::DragValue::new(&mut self.vehicle.jump_impulse).prefix("Jump impulse: "),
                );
                ui.add(
                    egui::DragValue::new(&mut self.vehicle.roll_impulse).prefix("Roll impulse: "),
                );
            });

        self.engine.populate_hud(ui);
    }

    fn on_draw(&mut self) -> time::Duration {
        self.update_time();

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
                    nalgebra::Vector3::y_axis().scale(projection),
                ));

            let camera_dt = self.last_camera_update.elapsed().as_secs_f32();
            self.last_physics_update = time::Instant::now();

            let cc = &self.cam_config;
            let smooth_t = (-camera_dt * cc.speed).exp();
            let smooth_quat = nalgebra::UnitQuaternion::new_normalize(
                base_quat.lerp(&self.last_camera_base_quat, smooth_t),
            );
            let base =
                nalgebra::geometry::Isometry3::from_parts(veh_isometry.translation, smooth_quat);
            self.last_camera_base_quat = smooth_quat;

            //TODO: `nalgebra::Point3::from(mint::Vector3)` doesn't exist?
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
            blade::Camera {
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
