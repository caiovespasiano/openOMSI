//! Road vehicle dynamics.
//!
//! This is a deliberately simple first model (longitudinal forces + kinematic steering +
//! ground following) that speaks the original script interface: the scripts provide
//! `M_Wheel` (drive torque at the driven wheels, Nm), `Axle_Brakeforce_i_L/R` (N per wheel)
//! and `Axle_Springfactor_*`; the engine provides `Velocity` (km/h), `n_Wheel` (rpm of the
//! driven axle), `Wheel_RotationSpeed_i_*` (rpm), `Wheel_Rotation_i_*` (degrees),
//! `Axle_Steering_i_*` (degrees), `Axle_Suspension_i_*` (m), `A_Trans_*` (m/s²), `Throttle`,
//! `Brake`, `Clutch`. A rigid-body model with wheel contacts replaces it later.

use glam::{DVec3, Vec3};
use omsi_vehicle::Vehicle;

/// Driver inputs, all in `0..1` except steering in `-1..1` (positive = right).
#[derive(Debug, Clone, Copy, Default)]
pub struct Controls {
    pub throttle: f32,
    pub brake: f32,
    pub clutch: f32,
    pub steering: f32,
}

#[derive(Debug, Clone)]
pub struct WheelState {
    /// Longitudinal axle position (m, forward positive) and lateral offset (m, right positive).
    pub long: f32,
    pub lat: f32,
    pub radius: f32,
    pub driven: bool,
    pub rotation_deg: f32,
    pub rpm: f32,
    pub suspension: f32,
}

#[derive(Debug, Clone)]
pub struct VehiclePhysics {
    pub mass_kg: f32,
    pub rolling_resistance: f32,
    pub inv_min_turn_radius: f32,
    pub rot_pnt_long: f32,
    pub wheelbase: f32,
    /// Per axle: (left, right) wheels.
    pub wheels: Vec<[WheelState; 2]>,
    /// Longitudinal speed, m/s (forward positive).
    pub speed: f32,
    pub accel: Vec3,
    /// Current steering angle of the front wheels, degrees, positive = right.
    pub steer_deg: f32,
    pub max_steer_deg: f32,
    pub controls: Controls,
    /// Steering wheel input rate for keyboard steering (fraction per second).
    pub steer_rate: f32,
    /// Steering-wheel angle, degrees (`input * half_lock`, half
    /// `1700/2 = 850` unless the definition says otherwise).
    pub wheel_deg: f32,
    /// Self-aligning torque, N·m (`-0.025*delta*v^2`, `±15`).
    /// Single source of truth for the self-aligning torque (authoritative
    /// over any preview/host estimate).
    pub ffb_nm: f32,
    /// Left/right road-wheel angles of the steered axle, radians
    /// (Ackermann inner/outer). Positive = right.
    pub ackermann_rad: (f32, f32),
    /// Bicycle effective radius, m (`L/(tan(delta)*f_curva)`).
    pub bicycle_radius_m: f32,
}

impl VehiclePhysics {
    pub fn from_definition(def: &Vehicle) -> VehiclePhysics {
        let mass_kg = if def.mass < 100.0 { def.mass * 1000.0 } else { def.mass };
        let wheels: Vec<[WheelState; 2]> = def
            .axles
            .iter()
            .map(|a| {
                let mk = |lat: f32| WheelState { long: a.long, lat, radius: (a.wheel_diameter / 2.0).max(0.1), driven: a.driven, rotation_deg: 0.0, rpm: 0.0, suspension: 0.0 };
                [mk(-a.max_width / 2.0), mk(a.max_width / 2.0)]
            })
            .collect();
        let front = wheels.iter().map(|w| w[0].long).fold(f32::MIN, f32::max);
        let rear = wheels.iter().map(|w| w[0].long).fold(f32::MAX, f32::min);
        let wheelbase = (front - rear).abs().max(1.0);
        // inv_min_turnradius = tan(alpha_max) / s  →  alpha_max
        let s = (front - def.rot_pnt_long).abs().max(1.0);
        let max_steer_deg = (def.inv_min_turn_radius * s).atan().to_degrees().clamp(10.0, 60.0);
        VehiclePhysics { mass_kg: mass_kg.max(500.0), rolling_resistance: def.rolling_resistance, inv_min_turn_radius: def.inv_min_turn_radius, rot_pnt_long: def.rot_pnt_long, wheelbase, wheels, speed: 0.0, accel: Vec3::ZERO, steer_deg: 0.0, max_steer_deg, controls: Controls::default(), steer_rate: 0.8, wheel_deg: 0.0, ffb_nm: 0.0, ackermann_rad: (0.0, 0.0), bicycle_radius_m: f32::INFINITY }
    }

    /// Advance one step. `drive_torque` is `M_Wheel`, `brake_forces` the per-wheel brake
    /// forces in the same order as `wheels`. Returns the displacement along the vehicle's
    /// forward axis and the heading change in degrees.
    pub fn step(&mut self, dt: f32, drive_torque: f32, brake_forces: &[f32]) -> (f32, f32) {
        let dt = dt.clamp(0.0, 0.1);
        // steering
        let target = self.controls.steering.clamp(-1.0, 1.0) * self.max_steer_deg;
        let rate = self.max_steer_deg * 2.5 * dt;
        self.steer_deg += (target - self.steer_deg).clamp(-rate, rate);
        // Wheel angle, single-writer FFB and Ackermann/bicycle state.
        let input = self.controls.steering.clamp(-1.0, 1.0);
        self.wheel_deg = input * 850.0;
        let steer_rad = self.steer_deg.to_radians();
        self.ffb_nm = Self::ffb_nm(steer_rad, self.speed);
        let max_rad = self.max_steer_deg.to_radians().max(0.1);
        let l0 = self.wheelbase.max(1.5);
        let track = self
            .wheels
            .first()
            .map(|a| (a[1].lat - a[0].lat).abs())
            .unwrap_or(2.0)
            .max(1.2);
        let (inner, outer) = Self::ackermann(steer_rad, max_rad, l0, track);
        self.ackermann_rad = if steer_rad >= 0.0 { (outer, inner) } else { (inner, outer) };
        self.bicycle_radius_m = Self::bicycle_radius(l0, steer_rad, 1.0);

        // forces along the forward axis
        let driven_radius = self.wheels.iter().find(|w| w[0].driven).or(self.wheels.first()).map(|w| w[0].radius).unwrap_or(0.5);
        let f_drive = drive_torque / driven_radius;
        let f_brake: f32 = brake_forces.iter().sum::<f32>().max(0.0);
        let f_roll = self.rolling_resistance.max(0.0);
        let v = self.speed;
        let resist = f_brake + f_roll;
        let mut f = f_drive;
        // resistances oppose motion; at rest they can only cancel the drive force
        if v.abs() > 0.05 {
            f -= resist * v.signum();
        } else if f_drive.abs() <= resist {
            f = 0.0;
            self.speed = 0.0;
        } else {
            f -= resist * f_drive.signum();
        }
        let a = f / self.mass_kg;
        let new_speed = v + a * dt;
        // resistances must not reverse the direction of travel
        if v.abs() > 0.05 && new_speed.signum() != v.signum() && f_drive.abs() <= resist {
            self.speed = 0.0;
        } else {
            self.speed = new_speed;
        }
        self.accel = Vec3::new(0.0, a, 0.0);
        let ds = self.speed * dt;

        // kinematic turning about the rotation point
        let curvature = self.steer_deg.to_radians().tan() / self.wheelbase;
        let dheading = (ds * curvature).to_degrees();

        // wheels
        for axle in self.wheels.iter_mut() {
            for w in axle.iter_mut() {
                let omega = self.speed / w.radius; // rad/s
                w.rpm = omega * 60.0 / std::f32::consts::TAU;
                w.rotation_deg = (w.rotation_deg + omega.to_degrees() * dt).rem_euclid(360.0);
            }
        }
        (ds, dheading)
    }

    pub fn velocity_kmh(&self) -> f32 {
        self.speed * 3.6
    }

    /// FFB, N·m: `T = -0.025*delta*v^2`, clamp `±15`.
    pub fn ffb_nm(steer_rad: f32, v_mps: f32) -> f32 {
        (-0.025 * steer_rad * v_mps * v_mps).clamp(-15.0, 15.0)
    }

    /// Ackermann magnitudes, radians: `r_in = L0/tan(inner)`,
    /// `r_out = r_in + track`, `outer = atan(L0/r_out)`.
    pub fn ackermann(delta_cmd: f32, max_steer_rad: f32, l0_m: f32, track_m: f32) -> (f32, f32) {
        let max = max_steer_rad.max(0.1);
        let l0 = l0_m.max(1.5);
        let track = track_m.max(1.2);
        let inner = (delta_cmd / max).clamp(-1.0, 1.0).abs() * max;
        if inner <= 1e-5 {
            return (0.0, 0.0);
        }
        let r_in = l0 / inner.tan();
        (inner, (l0 / (r_in + track)).atan())
    }

    /// Bicycle radius, m: `R = L/(tan(delta)*f_curva)`.
    pub fn bicycle_radius(l_m: f32, delta_rad: f32, f_curva: f32) -> f32 {
        let t = delta_rad.tan() * f_curva;
        if t.abs() < 1e-6 { f32::INFINITY } else { l_m / t }
    }

    /// Curve authority factor by chassis layout (table).
    pub fn curve_authority(layout: &str) -> f32 {
        match layout {
            "Rigid4x2" => 1.0,
            "Rigid6x2" => 0.70,
            "Rigid6x2Dir" => 0.95,
            "ArtPusher" => 0.90,
            "ArtMda" => 0.88,
            "ArtPuller" => 0.85,
            _ => 1.0,
        }
    }

    /// Directional rear steer: `ratio*delta*fade`, fade `25→40 km/h`.
    pub fn directional(delta: f32, v_kmh: f32, ratio: f32, max_rad: f32) -> f32 {
        let fade = ((40.0 - v_kmh) / 15.0).clamp(0.0, 1.0);
        (ratio * delta * fade).clamp(-max_rad, max_rad)
    }

    /// Ground height under the vehicle, used to follow the terrain.
    pub fn place_on_ground(position: &mut DVec3, ground: f64) {
        position.z = ground;
    }
}
