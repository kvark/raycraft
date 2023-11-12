#![allow(irrefutable_let_patterns)]
#![cfg(not(target_arch = "wasm32"))]

use blade_graphics as gpu;
use std::{fs, path::Path, sync::Arc, time};

const MAX_DEPTH: f32 = 1e9;

fn default_vec() -> mint::Vector3<f32> {
    [0.0; 3].into()
}
fn default_scale() -> f32 {
    1.0
}

#[derive(serde::Deserialize)]
struct VisualConfig {
    mesh: String,
    #[serde(default = "default_vec")]
    pos: mint::Vector3<f32>,
    #[serde(default = "default_vec")]
    rot: mint::Vector3<f32>,
    #[serde(default = "default_scale")]
    scale: f32,
}

#[derive(serde::Deserialize)]
struct ObjectConfig {
    #[serde(default)]
    name: String,
    visuals: Vec<VisualConfig>,
}

#[derive(serde::Deserialize)]
struct EngineConfig {
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

#[derive(Default)]
struct Physics {
    rigid_bodies: rapier3d::dynamics::RigidBodySet,
    integration_params: rapier3d::dynamics::IntegrationParameters,
    island_manager: rapier3d::dynamics::IslandManager,
    impulses: rapier3d::dynamics::ImpulseJointSet,
    joints: rapier3d::dynamics::MultibodyJointSet,
    solver: rapier3d::dynamics::CCDSolver,
    colliders: rapier3d::geometry::ColliderSet,
    broad_phase: rapier3d::geometry::BroadPhase,
    narrow_phase: rapier3d::geometry::NarrowPhase,
    gravity: rapier3d::math::Vector<f32>,
    pipeline: rapier3d::pipeline::PhysicsPipeline,
}

impl Physics {
    fn step(&mut self, dt: f32) {
        self.integration_params.dt = dt;
        let physics_hooks = ();
        let event_handler = ();
        self.pipeline.step(
            &self.gravity,
            &self.integration_params,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_bodies,
            &mut self.colliders,
            &mut self.impulses,
            &mut self.joints,
            &mut self.solver,
            None, // query pipeline
            &physics_hooks,
            &event_handler,
        );
    }
}

struct Object {
    name: String,
    rigid_body: rapier3d::dynamics::RigidBodyHandle,
    visuals: Vec<blade_render::Object>,
}

struct Engine {
    pacer: blade_render::util::FramePacer,
    renderer: blade_render::Renderer,
    physics: Physics,
    scene_load_task: Option<choir::RunningTask>,
    gui_painter: blade_egui::GuiPainter,
    asset_hub: blade_render::AssetHub,
    gpu_context: Arc<gpu::Context>,
    environment_map: Option<blade_asset::Handle<blade_render::Texture>>,
    objects: slab::Slab<Object>,
    render_objects: Vec<blade_render::Object>,
    camera: blade_render::Camera,
    debug: blade_render::DebugConfig,
    need_accumulation_reset: bool,
    is_debug_drawing: bool,
    ray_config: blade_render::RayConfig,
    denoiser_enabled: bool,
    denoiser_config: blade_render::DenoiserConfig,
    post_proc_config: blade_render::PostProcConfig,
    track_hot_reloads: bool,
    workers: Vec<choir::WorkerHandle>,
    _choir: Arc<choir::Choir>,
}

impl Engine {
    fn make_surface_config(physical_size: winit::dpi::PhysicalSize<u32>) -> gpu::SurfaceConfig {
        gpu::SurfaceConfig {
            size: gpu::Extent {
                width: physical_size.width,
                height: physical_size.height,
                depth: 1,
            },
            usage: gpu::TextureUsage::TARGET,
            frame_count: 3,
        }
    }

    #[profiling::function]
    fn new(window: &winit::window::Window, config: &EngineConfig) -> Self {
        log::info!("Initializing");

        let gpu_context = Arc::new(unsafe {
            gpu::Context::init_windowed(
                window,
                gpu::ContextDesc {
                    validation: cfg!(debug_assertions),
                    capture: false,
                },
            )
            .unwrap()
        });

        let surface_config = Self::make_surface_config(window.inner_size());
        let screen_size = surface_config.size;
        let surface_format = gpu_context.resize(surface_config);

        let num_workers = num_cpus::get_physical().max((num_cpus::get() * 3 + 2) / 4);
        log::info!("Initializing Choir with {} workers", num_workers);
        let choir = choir::Choir::new();
        let workers = (0..num_workers)
            .map(|i| choir.add_worker(&format!("Worker-{}", i)))
            .collect();

        let asset_hub = blade_render::AssetHub::new(Path::new("asset-cache"), &choir, &gpu_context);
        let (shaders, shader_task) =
            blade_render::Shaders::load(config.shader_path.as_ref(), &asset_hub);

        log::info!("Spinning up the renderer");
        shader_task.join();
        let mut pacer = blade_render::util::FramePacer::new(&gpu_context);
        let (command_encoder, _) = pacer.begin_frame();

        let render_config = blade_render::RenderConfig {
            screen_size,
            surface_format,
            max_debug_lines: 1000,
        };
        let renderer = blade_render::Renderer::new(
            command_encoder,
            &gpu_context,
            shaders,
            &asset_hub.shaders,
            &render_config,
        );

        pacer.end_frame(&gpu_context);

        let gui_painter = blade_egui::GuiPainter::new(surface_format, &gpu_context);

        Self {
            pacer,
            renderer,
            physics: Physics::default(),
            scene_load_task: None,
            gui_painter,
            asset_hub,
            gpu_context,
            environment_map: None,
            objects: slab::Slab::new(),
            render_objects: Vec::new(),
            camera: blade_render::Camera {
                pos: [0.0; 3].into(),
                rot: mint::Quaternion {
                    v: [0.0; 3].into(),
                    s: 1.0,
                },
                fov_y: 1.0,
                depth: MAX_DEPTH,
            },
            debug: blade_render::DebugConfig::default(),
            need_accumulation_reset: true,
            is_debug_drawing: false,
            ray_config: blade_render::RayConfig {
                num_environment_samples: 1,
                environment_importance_sampling: false,
                temporal_history: 10,
                spatial_taps: 1,
                spatial_tap_history: 5,
                spatial_radius: 10,
            },
            denoiser_enabled: true,
            denoiser_config: blade_render::DenoiserConfig {
                num_passes: 3,
                temporal_weight: 0.1,
            },
            post_proc_config: blade_render::PostProcConfig {
                average_luminocity: 1.0,
                exposure_key_value: 1.0 / 9.6,
                white_level: 1.0,
            },
            track_hot_reloads: false,
            workers,
            _choir: choir,
        }
    }

    fn destroy(&mut self) {
        self.workers.clear();
        self.pacer.destroy(&self.gpu_context);
        self.gui_painter.destroy(&self.gpu_context);
        self.renderer.destroy(&self.gpu_context);
        self.asset_hub.destroy();
    }

    #[profiling::function]
    fn update(&mut self, dt: f32) {
        self.physics.step(dt);
    }

    #[profiling::function]
    fn render(
        &mut self,
        gui_primitives: &[egui::ClippedPrimitive],
        gui_textures: &egui::TexturesDelta,
        physical_size: winit::dpi::PhysicalSize<u32>,
        scale_factor: f32,
    ) {
        if self.track_hot_reloads {
            self.need_accumulation_reset |= self.renderer.hot_reload(
                &self.asset_hub,
                &self.gpu_context,
                self.pacer.last_sync_point().unwrap(),
            );
        }

        // Note: the resize is split in 2 parts because `wait_for_previous_frame`
        // wants to borrow `self` mutably, and `command_encoder` blocks that.
        let surface_config = Self::make_surface_config(physical_size);
        let new_render_size = surface_config.size;
        if new_render_size != self.renderer.get_screen_size() {
            log::info!("Resizing to {}", new_render_size);
            self.pacer.wait_for_previous_frame(&self.gpu_context);
            self.gpu_context.resize(surface_config);
        }

        let (command_encoder, temp) = self.pacer.begin_frame();
        if new_render_size != self.renderer.get_screen_size() {
            self.renderer
                .resize_screen(new_render_size, command_encoder, &self.gpu_context);
            self.need_accumulation_reset = true;
        }

        self.gui_painter
            .update_textures(command_encoder, gui_textures, &self.gpu_context);

        self.asset_hub.flush(command_encoder, &mut temp.buffers);

        if let Some(ref task) = self.scene_load_task {
            if task.is_done() {
                log::info!("Scene is loaded");
                self.scene_load_task = None;
            }
        }

        // We should be able to update TLAS and render content
        // even while it's still being loaded.
        if self.scene_load_task.is_none() {
            self.render_objects.clear();
            for object in self.objects.iter() {
                // TODO: get transform
                // populate self.render_objects
            }

            // Rebuilding every frame
            self.renderer.build_scene(
                command_encoder,
                &self.render_objects,
                self.environment_map,
                &self.asset_hub,
                &self.gpu_context,
                temp,
            );

            self.renderer.prepare(
                command_encoder,
                &self.camera,
                self.is_debug_drawing,
                self.debug.mouse_pos.is_some(),
                self.need_accumulation_reset,
            );
            self.need_accumulation_reset = false;

            if !self.objects.is_empty() {
                self.renderer
                    .ray_trace(command_encoder, self.debug, self.ray_config);
                if self.denoiser_enabled {
                    self.renderer.denoise(command_encoder, self.denoiser_config);
                }
            }
        }

        let frame = self.gpu_context.acquire_frame();
        command_encoder.init_texture(frame.texture());

        if let mut pass = command_encoder.render(gpu::RenderTargetSet {
            colors: &[gpu::RenderTarget {
                view: frame.texture_view(),
                init_op: gpu::InitOp::Clear(gpu::TextureColor::TransparentBlack),
                finish_op: gpu::FinishOp::Store,
            }],
            depth_stencil: None,
        }) {
            let screen_desc = blade_egui::ScreenDescriptor {
                physical_size: (physical_size.width, physical_size.height),
                scale_factor,
            };
            if self.scene_load_task.is_none() {
                self.renderer
                    .post_proc(&mut pass, self.debug, self.post_proc_config, &[]);
            }
            self.gui_painter
                .paint(&mut pass, gui_primitives, &screen_desc, &self.gpu_context);
        }

        command_encoder.present(frame);
        let sync_point = self.pacer.end_frame(&self.gpu_context);
        self.gui_painter.after_submit(sync_point);
    }

    #[profiling::function]
    fn populate_hud(&mut self, _ui: &mut egui::Ui) {
        //TODO
    }
}

struct Game {
    engine: Engine,
    last_physics_update: time::Instant,
    egui_context: egui::Context,
    ground_handle: usize,
}

impl Game {
    fn new(window: &winit::window::Window) -> Self {
        log::info!("Initializing");

        let config: GameConfig =
            ron::de::from_bytes(&fs::read("data/config.ron").expect("Unable to open the config"))
                .expect("Unable to parse the config");

        let mut engine = Engine::new(window, &config.engine);
        engine.physics.gravity.y = -config.level.gravity;
        engine.post_proc_config.average_luminocity = config.level.average_luminocity;

        let ground_handle = 0; //TODO

        Self {
            engine,
            last_physics_update: time::Instant::now(),
            egui_context: egui::Context::default(),
            ground_handle,
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
