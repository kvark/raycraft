#[derive(serde::Deserialize)]
pub struct Body {
    pub visual: blade::config::Visual,
    pub collider: blade::config::Collider,
    pub height: f32,
}

#[derive(serde::Deserialize)]
pub struct Wheel {
    pub visual: blade::config::Visual,
    pub collider: blade::config::Collider,
}

#[derive(Default, serde::Deserialize)]
pub struct Motor {
    pub limit: f32,
    pub stiffness: f32,
    pub damping: f32,
}

#[derive(serde::Deserialize)]
pub struct Axle {
    pub z: f32,
    pub xs: Vec<f32>,
    #[serde(default)]
    pub additional_mass: Option<blade::config::AdditionalMass>,
    #[serde(default)]
    pub suspension: Motor,
    #[serde(default)]
    pub steering: Motor,
}

#[derive(serde::Deserialize)]
pub struct Vehicle {
    pub body: Body,
    pub wheel: Wheel,
    pub drive_factor: f32,
    pub jump_impulse: f32,
    pub roll_impulse: f32,
    pub axles: Vec<Axle>,
}

#[derive(serde::Deserialize)]
pub struct Level {
    #[serde(default)]
    pub environment: String,
    pub gravity: f32,
    pub average_luminocity: f32,
    pub spawn_pos: [f32; 3],
    pub ground: blade::config::Object,
}

#[derive(serde::Deserialize)]
pub struct Camera {
    pub azimuth: f32,
    pub altitude: f32,
    pub distance: f32,
    pub speed: f32,
    pub target: [f32; 3],
    pub fov: f32,
}

#[derive(serde::Deserialize)]
pub struct Game {
    pub engine: blade::config::Engine,
    pub camera: Camera,
    pub level: String,
    pub vehicle: String,
}
