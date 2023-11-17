#![allow(irrefutable_let_patterns)]
#![cfg(not(target_arch = "wasm32"))]

mod engine;

use std::{fs, time};

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
struct VehicleBodyConfig {
    model: String,
    collider: ColliderConfig,
}

#[derive(serde::Deserialize)]
struct VehicleConfig {
    body: VehicleBodyConfig,
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

struct Vehicle {
    body_handle: usize,
}

struct Game {
    // engine stuff
    engine: engine::Engine,
    last_physics_update: time::Instant,
    // windowing
    window: winit::window::Window,
    egui_state: egui_winit::State,
    egui_context: egui::Context,
    // game data
    _ground_handle: usize,
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
            visuals: vec![VisualConfig {
                model: veh_config.body.model,
                ..Default::default()
            }],
            colliders: vec![veh_config.body.collider],
        };
        let vehicle = Vehicle {
            body_handle: engine.add_object(
                &body_config,
                nalgebra::Isometry3::translation(0.0, 10.0, 0.0),
                engine::BodyType::Dynamic,
            ),
        };

        Self {
            engine,
            last_physics_update: time::Instant::now(),
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
                winit::event::VirtualKeyCode::W => {
                    let veh_isometry = self.engine.get_object_isometry(self.vehicle.body_handle);
                    let dir = veh_isometry
                        .rotation
                        .transform_vector(&[0.0, 0.0, 10.0].into());
                    self.engine.apply_impulse(self.vehicle.body_handle, dir);
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

    fn on_draw(&mut self) -> time::Duration {
        let raw_input = self.egui_state.take_egui_input(&self.window);
        let egui_output = self.egui_context.run(raw_input, |egui_ctx| {
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
                .show(egui_ctx, |ui| {
                    egui::CollapsingHeader::new("Camera").show(ui, |ui| {
                        ui.add(
                            egui::DragValue::new(&mut self.cam_config.distance)
                                .prefix("Distance")
                                .clamp_range(1.0..=100.0),
                        );
                        ui.add(egui::Slider::new(
                            &mut self.cam_config.azimuth,
                            -std::f32::consts::FRAC_PI_2..=std::f32::consts::FRAC_PI_2,
                        ));
                        ui.add(egui::Slider::new(
                            &mut self.cam_config.altitude,
                            0.01..=std::f32::consts::FRAC_PI_2,
                        ));
                        ui.horizontal(|ui| {
                            ui.label("Target:");
                            ui.add(egui::DragValue::new(&mut self.cam_config.target.y));
                            ui.add(egui::DragValue::new(&mut self.cam_config.target.z));
                        });
                        ui.add(
                            egui::Slider::new(&mut self.cam_config.fov, 0.5f32..=2.0f32)
                                .text("FOV"),
                        );
                    });

                    self.engine.populate_hud(ui);
                });
        });

        self.egui_state.handle_platform_output(
            &self.window,
            &self.egui_context,
            egui_output.platform_output,
        );
        let engine_dt = self.last_physics_update.elapsed().as_secs_f32();
        self.last_physics_update = time::Instant::now();
        self.engine.update(engine_dt);

        let camera = {
            let veh_isometry = self.engine.get_object_isometry(self.vehicle.body_handle);
            //TODO: `nalgebra::Point3::from(mint::Vector3)` doesn't exist?
            let cc = &self.cam_config;
            let source = nalgebra::Vector3::from(cc.target)
                + nalgebra::Vector3::new(cc.azimuth.sin(), cc.altitude.sin(), cc.azimuth.cos())
                    .scale(cc.distance);
            let local = nalgebra::geometry::Isometry3::look_at_rh(
                &source.into(),
                &nalgebra::Vector3::from(cc.target).into(),
                &nalgebra::Vector3::y_axis(),
            );
            engine::Camera {
                isometry: veh_isometry * local.inverse(),
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
