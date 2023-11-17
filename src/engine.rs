use blade_graphics as gpu;
use std::{path::Path, sync::Arc};

pub use rapier3d::dynamics::RigidBodyType as BodyType;

const MAX_DEPTH: f32 = 1e9;

#[derive(Default)]
struct PhysicsVisualization {
    bounding_boxes: bool,
    contacts: bool,
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
    visualization: PhysicsVisualization,
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

    fn collect_debug_lines(&self) -> Vec<blade_render::DebugLine> {
        let mut lines = Vec::new();

        if self.visualization.bounding_boxes {
            for (_handle, collider) in self.colliders.iter() {
                let color = 0xFFFFFF;
                let (vertices, edges) = collider.compute_aabb().to_outline();
                for pair in edges {
                    lines.push(blade_render::DebugLine {
                        a: blade_render::DebugPoint {
                            pos: vertices[pair[0] as usize].into(),
                            color,
                        },
                        b: blade_render::DebugPoint {
                            pos: vertices[pair[1] as usize].into(),
                            color,
                        },
                    });
                }
            }
        }
        if self.visualization.contacts {
            for contact_pair in self.narrow_phase.contact_graph().interactions() {
                for manifold in contact_pair.manifolds.iter() {
                    for solver_contact in manifold.data.solver_contacts.iter() {
                        let end = solver_contact.point + solver_contact.dist * manifold.data.normal;
                        let color = if solver_contact.dist > 0.0 {
                            0x00FF00
                        } else {
                            0xFF0000
                        };
                        lines.push(blade_render::DebugLine {
                            a: blade_render::DebugPoint {
                                pos: solver_contact.point.into(),
                                color,
                            },
                            b: blade_render::DebugPoint {
                                pos: end.into(),
                                color,
                            },
                        });
                    }
                }
            }
        }

        lines
    }
}

struct Visual {
    model: blade_asset::Handle<blade_render::Model>,
    similarity: nalgebra::geometry::Similarity3<f32>,
}

struct Object {
    name: String,
    rigid_body: rapier3d::dynamics::RigidBodyHandle,
    _colliders: Vec<rapier3d::geometry::ColliderHandle>,
    visuals: Vec<Visual>,
}

pub struct Camera {
    pub isometry: nalgebra::Isometry3<f32>,
    pub fov_y: f32,
}

pub struct Engine {
    pacer: blade_render::util::FramePacer,
    renderer: blade_render::Renderer,
    physics: Physics,
    load_tasks: Vec<choir::RunningTask>,
    gui_painter: blade_egui::GuiPainter,
    asset_hub: blade_render::AssetHub,
    gpu_context: Arc<gpu::Context>,
    environment_map: Option<blade_asset::Handle<blade_render::Texture>>,
    objects: slab::Slab<Object>,
    render_objects: Vec<blade_render::Object>,
    debug: blade_render::DebugConfig,
    need_accumulation_reset: bool,
    is_debug_drawing: bool,
    ray_config: blade_render::RayConfig,
    denoiser_enabled: bool,
    denoiser_config: blade_render::DenoiserConfig,
    post_proc_config: blade_render::PostProcConfig,
    track_hot_reloads: bool,
    workers: Vec<choir::WorkerHandle>,
    choir: Arc<choir::Choir>,
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
    pub fn new(window: &winit::window::Window, config: &super::EngineConfig) -> Self {
        log::info!("Initializing the engine");

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
            load_tasks: Vec::new(),
            gui_painter,
            asset_hub,
            gpu_context,
            environment_map: None,
            objects: slab::Slab::new(),
            render_objects: Vec::new(),
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
            choir,
        }
    }

    pub fn destroy(&mut self) {
        self.workers.clear();
        self.pacer.destroy(&self.gpu_context);
        self.gui_painter.destroy(&self.gpu_context);
        self.renderer.destroy(&self.gpu_context);
        self.asset_hub.destroy();
    }

    #[profiling::function]
    pub fn update(&mut self, dt: f32) {
        self.choir.check_panic();
        self.physics.step(dt);
    }

    #[profiling::function]
    pub fn render(
        &mut self,
        camera: &Camera,
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

        self.load_tasks.retain(|task| !task.is_done());

        // We should be able to update TLAS and render content
        // even while it's still being loaded.
        if self.load_tasks.is_empty() {
            self.render_objects.clear();
            for (_, object) in self.objects.iter() {
                let isometry = self
                    .physics
                    .rigid_bodies
                    .get(object.rigid_body)
                    .unwrap()
                    .position();
                for visual in object.visuals.iter() {
                    let m = (isometry * visual.similarity).to_homogeneous();
                    self.render_objects.push(blade_render::Object {
                        transform: blade_graphics::Transform {
                            x: m.column(0).into(),
                            y: m.column(1).into(),
                            z: m.column(2).into(),
                        },
                        model: visual.model,
                    });
                }
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
                &blade_render::Camera {
                    pos: camera.isometry.translation.vector.into(),
                    rot: camera.isometry.rotation.into(),
                    fov_y: camera.fov_y,
                    depth: MAX_DEPTH,
                },
                self.is_debug_drawing,
                self.debug.mouse_pos.is_some(),
                self.need_accumulation_reset,
            );
            self.need_accumulation_reset = false;

            if !self.render_objects.is_empty() {
                self.renderer
                    .ray_trace(command_encoder, self.debug, self.ray_config);
                if self.denoiser_enabled {
                    self.renderer.denoise(command_encoder, self.denoiser_config);
                }
            }
        }

        let debug_lines = self.physics.collect_debug_lines();

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
            if self.load_tasks.is_empty() {
                self.renderer.post_proc(
                    &mut pass,
                    self.debug,
                    self.post_proc_config,
                    &debug_lines,
                    &[],
                );
            }
            self.gui_painter
                .paint(&mut pass, gui_primitives, &screen_desc, &self.gpu_context);
        }

        command_encoder.present(frame);
        let sync_point = self.pacer.end_frame(&self.gpu_context);
        self.gui_painter.after_submit(sync_point);
    }

    #[profiling::function]
    pub fn populate_hud(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Debug").show(ui, |ui| {
            ui.checkbox(
                &mut self.physics.visualization.bounding_boxes,
                "Visualize bounding boxes",
            );
            ui.checkbox(
                &mut self.physics.visualization.contacts,
                "Visualize contacts",
            );
        });
        egui::CollapsingHeader::new("Objects")
            .default_open(true)
            .show(ui, |ui| {
                let mut selected_object_index = None;
                for (handle, object) in self.objects.iter() {
                    ui.selectable_value(&mut selected_object_index, Some(handle), &object.name);
                }
            });
    }

    pub fn add_object(
        &mut self,
        config: &super::ObjectConfig,
        isometry: nalgebra::Isometry3<f32>,
        body_type: BodyType,
    ) -> usize {
        let mut visuals = Vec::new();
        for visual in config.visuals.iter() {
            let (model, task) = self.asset_hub.models.load(
                format!("data/{}", visual.model),
                blade_render::model::Meta {
                    generate_tangents: true,
                },
            );
            visuals.push(Visual {
                model,
                similarity: nalgebra::geometry::Similarity3::from_parts(
                    nalgebra::Vector3::from(visual.pos).into(),
                    nalgebra::geometry::UnitQuaternion::from_euler_angles(
                        visual.rot.x,
                        visual.rot.y,
                        visual.rot.z,
                    ),
                    visual.scale,
                ),
            });
            self.load_tasks.push(task.clone());
        }

        let rigid_body = rapier3d::dynamics::RigidBodyBuilder::new(body_type)
            .position(isometry)
            .build();
        let rb_handle = self.physics.rigid_bodies.insert(rigid_body);

        let mut colliders = Vec::new();
        for cc in config.colliders.iter() {
            let isometry = nalgebra::geometry::Isometry3::from_parts(
                nalgebra::Vector3::from(cc.pos).into(),
                nalgebra::geometry::UnitQuaternion::from_euler_angles(cc.rot.x, cc.rot.y, cc.rot.z),
            );
            let builder = match cc.shape {
                super::Shape::Ball { radius } => rapier3d::geometry::ColliderBuilder::ball(radius),
                super::Shape::Cylinder {
                    half_height,
                    radius,
                } => rapier3d::geometry::ColliderBuilder::cylinder(half_height, radius),
                super::Shape::Cuboid { half } => {
                    rapier3d::geometry::ColliderBuilder::cuboid(half.x, half.y, half.z)
                }
                super::Shape::ConvexHull { ref points } => {
                    let pv = points
                        .iter()
                        .map(|p| nalgebra::Vector3::from(*p).into())
                        .collect::<Vec<_>>();
                    rapier3d::geometry::ColliderBuilder::convex_hull(&pv)
                        .expect("Unable to build convex full")
                }
            };
            let collider = builder.mass(cc.mass).position(isometry).build();
            let c_handle = self.physics.colliders.insert_with_parent(
                collider,
                rb_handle,
                &mut self.physics.rigid_bodies,
            );
            colliders.push(c_handle);
        }

        self.objects.insert(Object {
            name: config.name.clone(),
            rigid_body: rb_handle,
            _colliders: colliders,
            visuals,
        })
    }

    pub fn get_object_isometry(&self, handle: usize) -> &nalgebra::Isometry3<f32> {
        let object = &self.objects[handle];
        let body = &self.physics.rigid_bodies[object.rigid_body];
        body.position()
    }

    pub fn apply_impulse(&mut self, handle: usize, impulse: nalgebra::Vector3<f32>) {
        let object = &self.objects[handle];
        let body = &mut self.physics.rigid_bodies[object.rigid_body];
        body.apply_impulse(impulse, false)
    }

    pub fn set_environment_map(&mut self, path: &str) {
        if path.is_empty() {
            self.environment_map = None;
        } else {
            let full = format!("data/{}", path);
            let (handle, task) = self.asset_hub.textures.load(
                full,
                blade_render::texture::Meta {
                    format: blade_graphics::TextureFormat::Rgba32Float,
                    generate_mips: false,
                    y_flip: false,
                },
            );
            self.environment_map = Some(handle);
            self.load_tasks.push(task.clone());
        }
    }

    pub fn set_gravity(&mut self, force: f32) {
        self.physics.gravity.y = -force;
    }

    pub fn set_average_luminosity(&mut self, avg_lum: f32) {
        self.post_proc_config.average_luminocity = avg_lum;
    }
}
