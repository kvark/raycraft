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

#[derive(serde::Deserialize)]
pub struct ObjectConfig {
    #[serde(default)]
    name: String,
    visuals: Vec<VisualConfig>,
}

#[derive(serde::Deserialize)]
pub struct EngineConfig {
    shader_path: String,
}

#[derive(serde::Deserialize)]
struct LevelConfig {
    gravity: f32,
    average_luminocity: f32,
    ground: ObjectConfig,
}

#[derive(serde::Deserialize)]
struct GameConfig {
    engine: EngineConfig,
    level: LevelConfig,
}

struct Game {
    engine: engine::Engine,
    last_physics_update: time::Instant,
    egui_context: egui::Context,
    _ground_handle: usize,
}

impl Game {
    fn new(window: &winit::window::Window) -> Self {
        log::info!("Initializing");

        let config: GameConfig =
            ron::de::from_bytes(&fs::read("data/config.ron").expect("Unable to open the config"))
                .expect("Unable to parse the config");

        let mut engine = engine::Engine::new(window, &config.engine);
        engine.set_gravity(config.level.gravity);
        engine.set_average_luminosity(config.level.average_luminocity);

        let ground_handle = engine.add_object(&config.level.ground);

        Self {
            engine,
            last_physics_update: time::Instant::now(),
            egui_context: egui::Context::default(),
            _ground_handle: ground_handle,
        }
    }

    fn destroy(&mut self) {
        self.engine.destroy();
    }

    fn redraw(
        &mut self,
        raw_input: egui::RawInput,
        physical_size: winit::dpi::PhysicalSize<u32>,
    ) -> (egui::PlatformOutput, time::Duration) {
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
                    self.engine.populate_hud(ui);
                });
        });

        let engine_dt = self.last_physics_update.elapsed().as_secs_f32();
        self.last_physics_update = time::Instant::now();
        self.engine.update(engine_dt);

        let primitives = self.egui_context.tessellate(egui_output.shapes);
        self.engine.render(
            &primitives,
            &egui_output.textures_delta,
            physical_size,
            self.egui_context.pixels_per_point(),
        );

        (egui_output.platform_output, egui_output.repaint_after)
    }
}

fn main() {
    env_logger::init();
    //let _ = profiling::tracy_client::Client::start();

    let event_loop = winit::event_loop::EventLoop::new();
    let window = winit::window::WindowBuilder::new()
        .with_title("RayCraft")
        .build(&event_loop)
        .unwrap();

    let mut game = Game::new(&window);
    let mut egui_winit = egui_winit::State::new(&event_loop);
    let mut last_event = time::Instant::now();

    event_loop.run(move |event, _, control_flow| {
        *control_flow = winit::event_loop::ControlFlow::Poll;
        let _delta = last_event.elapsed().as_secs_f32();
        last_event = time::Instant::now();

        match event {
            winit::event::Event::RedrawEventsCleared => {
                window.request_redraw();
            }
            winit::event::Event::WindowEvent { event, .. } => match event {
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
                        *control_flow = winit::event_loop::ControlFlow::Exit;
                    }
                    _ => {}
                },
                winit::event::WindowEvent::CloseRequested => {
                    *control_flow = winit::event_loop::ControlFlow::Exit;
                }
                _ => {}
            },
            winit::event::Event::RedrawRequested(_) => {
                let raw_input = egui_winit.take_egui_input(&window);
                let (raw_output, wait) = game.redraw(raw_input, window.inner_size());
                profiling::finish_frame!();

                egui_winit.handle_platform_output(&window, &game.egui_context, raw_output);

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
