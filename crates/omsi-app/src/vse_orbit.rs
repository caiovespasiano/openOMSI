//! F3/F4 cameras rebuilt 1:1 from the VSE (`D:/Programação/Engine Simulador`).
//!
//! Nothing of the previous openOMSI chase/free implementation is reused here
//! (`camera_look` outside branch, `camera_clipped`, `SpringArm`, orbit field):
//! every formula below is the VSE chain with the same constants, signs,
//! clamps and order. openOMSI services used as-is (never reimplemented):
//! `Camera::ray` (ray construction), `World::ground_height` (terrain query),
//! `ray_mesh` (mesh query), `winit` input.
//!
//! Conventions: VSE world frame x = right, y = forward, z = up, heading with
//! forward = (sin, cos) — identical to the openOMSI vehicle frame, so the
//! formulas transfer verbatim. Angles that the openOMSI `Camera` needs
//! (yaw = atan2(dx, dy), pitch = asin(dz)) are DERIVED from the VSE
//! position/target points, never converted by sign hacks.

use crate::vse::{
    VSE_CAM_EASE_SECS, VSE_CHASE_DIST_MAX, VSE_CHASE_DIST_MIN, VSE_CHASE_FOV,
    VSE_CHASE_OFF_Z, VSE_CHASE_PITCH_MAX, VSE_CHASE_PITCH_MIN, VSE_CHASE_SENS,
    VSE_CHASE_TAU_Z, VSE_CHASE_TGT_Z, VSE_CHASE_ZOOM_K, VSE_F4_SMOOTH_K,
    VSE_PICK_BISECT, VSE_PICK_T_MAX, VSE_PICK_T_MIN,
};

/// Chase camera (VSE `ExteriorChase` state: `m_chaseYawDeg`,
/// `m_chasePitchDeg`, `m_chaseDistance`, `m_chaseCamZFilt`).
#[derive(Debug, Clone)]
pub struct VseChase {
    pub yaw_deg: f32,
    pub pitch_deg: f32,
    pub dist: f32,
    pub pivot_z: f64,
    pub pivot_valid: bool,
    pub bus_wb: f32,
}

impl VseChase {
    /// VSE `Start`: yaw 0, pitch 12, `dist = wb*1.5+4`, filter unset.
    pub fn new(wheelbase_m: f32) -> Self {
        let wb = if wheelbase_m > 1.0 { wheelbase_m } else { 6.08 };
        Self {
            yaw_deg: 0.0,
            pitch_deg: 12.0,
            dist: wb * 1.5 + 4.0,
            pivot_z: 0.0,
            pivot_valid: false,
            bus_wb: wb,
        }
    }

    /// A new bus is a new `Start`: re-derive everything from its wheelbase.
    /// The driver's own zoom is never touched afterwards.
    pub fn ensure_bus(&mut self, wheelbase_m: f32) {
        let wb = if wheelbase_m > 1.0 { wheelbase_m } else { 6.08 };
        if (wb - self.bus_wb).abs() > 1e-6 {
            *self = Self::new(wb);
        }
    }

    /// VSE `SetCameraMode(ExteriorChase)`: the Z filter restarts; angles and
    /// distance persist.
    pub fn enter(&mut self) {
        self.pivot_valid = false;
    }

    /// VSE `ResetCameraCenter` chase part: yaw 0, pitch 12, filter restarts.
    /// Distance is KEPT (the VSE never zeroes it).
    pub fn reset(&mut self) {
        self.yaw_deg = 0.0;
        self.pitch_deg = 12.0;
        self.pivot_valid = false;
    }

    /// VSE `OnChaseOrbit(dx, dy)` with raw mouse pixels: `yaw -= dx*0.35`
    /// (inverted), `pitch += dy*0.35`, clamp `-10..75`.
    pub fn on_orbit_px(&mut self, dx: f32, dy: f32) {
        self.yaw_deg -= dx * VSE_CHASE_SENS;
        self.pitch_deg = (self.pitch_deg + dy * VSE_CHASE_SENS)
            .clamp(VSE_CHASE_PITCH_MIN, VSE_CHASE_PITCH_MAX);
    }

    /// VSE `OnZoom(delta)`: `dist -= delta*1.5`, clamp `4..40`.
    pub fn on_zoom(&mut self, delta: f32) {
        self.dist = (self.dist - delta * VSE_CHASE_ZOOM_K)
            .clamp(VSE_CHASE_DIST_MIN, VSE_CHASE_DIST_MAX);
    }

    /// Degree sources without pixel gain (keys, scripts, pad): applied
    /// directly, same clamp. Not a VSE path — keeps one writer per state.
    pub fn nudge_deg(&mut self, dyaw: f32, dpitch: f32) {
        self.yaw_deg += dyaw;
        self.pitch_deg = (self.pitch_deg + dpitch)
            .clamp(VSE_CHASE_PITCH_MIN, VSE_CHASE_PITCH_MAX);
    }

    /// VSE `UpdateCameraPose ExteriorChase`: yaw-only basis, smoothed pivot Z
    /// (`tau 0.30`, snap while invalid), then the shared pose math below.
    /// `pos`/`center` combine through `pivot_base` (VSE `yr/yf` basis).
    pub fn pose(
        &mut self,
        pos: [f64; 3],
        heading_rad: f64,
        center: [f32; 3],
        dt: f32,
    ) -> ([f64; 3], [f64; 3]) {
        let base = pivot_base(pos, heading_rad, center);
        if !self.pivot_valid {
            self.pivot_z = base[2];
            self.pivot_valid = true;
        } else {
            let safe = if dt > 1e-4 { dt } else { 1.0 / 60.0 };
            let lambda = 1.0 - (-safe / VSE_CHASE_TAU_Z).exp();
            self.pivot_z += lambda as f64 * (base[2] - self.pivot_z);
        }
        chase_points(base[0], base[1], self.pivot_z, heading_rad, self.yaw_deg, self.pitch_deg, self.dist)
    }

    /// Seed values for `InitFreeCamFromChase`.
    pub fn seed(&self) -> (f32, f32, f32) {
        (self.yaw_deg, self.pitch_deg, self.dist)
    }
}

/// Free orbit camera (VSE `EditorCameraController` in Play `FreeCam` mode).
#[derive(Debug, Clone)]
pub struct VseFreeOrbit {
    pub target: [f64; 3],
    pub desired: [f64; 3],
    pub interpolating: bool,
    pub position: [f64; 3],
    pub dist: f32,
    pub yaw_deg: f32,
    pub pitch_deg: f32,
    pub fov_deg: f32,
}

impl Default for VseFreeOrbit {
    fn default() -> Self {
        Self {
            target: [0.0; 3],
            desired: [0.0; 3],
            interpolating: false,
            position: [0.0; 3],
            dist: 8.0,
            yaw_deg: 45.0,
            pitch_deg: 25.0,
            fov_deg: VSE_CHASE_FOV,
        }
    }
}

impl VseFreeOrbit {
    /// VSE `Update(dt)`: glide the target (`blend = 1−exp(−10dt)`, snap below
    /// `1e-5`), then reposition from orbit (mode is always Orbit in Play).
    pub fn update(&mut self, dt: f32) {
        if self.interpolating {
            let blend = 1.0 - (-VSE_F4_SMOOTH_K * dt.max(0.001)).exp();
            let mut dist_sq = 0.0f64;
            for i in 0..3 {
                let diff = self.desired[i] - self.target[i];
                dist_sq += diff * diff;
                self.target[i] += diff * blend as f64;
            }
            if dist_sq < 1e-5 {
                self.target = self.desired;
                self.interpolating = false;
            }
        }
        self.reposition();
    }

    /// VSE `OnOrbit(deltaPitchDeg, deltaYawDeg)` — Play calls it as
    /// `OnOrbit(dy, dx)`: drag down raises elevation, drag right turns left.
    /// Damping past 70°, clamp ±88°, yaw wraps 0..360.
    pub fn on_orbit(&mut self, dy_px: f32, dx_px: f32) {
        let atten = if self.pitch_deg.abs() > 70.0 {
            ((88.0 - self.pitch_deg.abs()) / 18.0).max(0.20)
        } else {
            1.0
        };
        self.pitch_deg += dy_px * 0.35 * atten;
        self.yaw_deg -= dx_px * 0.35;
        self.pitch_deg = self.pitch_deg.clamp(-88.0, 88.0);
        while self.yaw_deg >= 360.0 {
            self.yaw_deg -= 360.0;
        }
        while self.yaw_deg < 0.0 {
            self.yaw_deg += 360.0;
        }
        self.reposition();
    }

    /// Degree nudge (keys, scripts, pad — no pixel gain): same clamp and wrap
    /// as `on_orbit`, then reposition. Not a VSE path; keeps every degree
    /// source alive with one writer.
    pub fn nudge(&mut self, dyaw_deg: f32, dpitch_deg: f32) {
        self.pitch_deg = (self.pitch_deg + dpitch_deg).clamp(-88.0, 88.0);
        self.yaw_deg += dyaw_deg;
        while self.yaw_deg >= 360.0 {
            self.yaw_deg -= 360.0;
        }
        while self.yaw_deg < 0.0 {
            self.yaw_deg += 360.0;
        }
        self.reposition();
    }

    /// VSE `OnPan(dx, dy)`: exact 1:1 pixel-to-world at the target plane.
    pub fn on_pan(
        &mut self,
        dx_px: f32,
        dy_px: f32,
        right: [f64; 3],
        up: [f64; 3],
        vp_height_px: f32,
    ) {
        let vp = vp_height_px.max(1.0) as f64;
        let half_tan = ((self.fov_deg.to_radians() * 0.5).tan()) as f64;
        let wpp = 2.0 * self.dist as f64 * half_tan / vp;
        let dxw = -(dx_px as f64) * wpp;
        let dyw = dy_px as f64 * wpp;
        for i in 0..3 {
            let shift = right[i] * dxw + up[i] * dyw;
            self.target[i] += shift;
            self.position[i] += shift;
        }
    }

    /// VSE `OnZoom(wheelDelta)`: up (`>0`) ×0.88, else ×1.14, clamp 0.1..2000.
    pub fn on_zoom(&mut self, wheel: f32) {
        let factor = if wheel > 0.0 { 0.88 } else { 1.14 };
        self.dist = (self.dist * factor).clamp(0.1, 2000.0);
        self.reposition();
    }

    /// VSE `OnPrecisionDolly(dy)`: up closes in, down recedes.
    pub fn on_dolly(&mut self, dy_px: f32) {
        let factor = (1.0 - dy_px * 0.0028).clamp(0.80, 1.20);
        self.dist = (self.dist * factor).clamp(0.05, 2500.0);
        self.reposition();
    }

    /// VSE `SetOrbitTarget`: hard target + resync dist/yaw/pitch from the
    /// kept position (first MMB drag orbits the NEW target, never snaps back).
    pub fn set_target(&mut self, x: f64, y: f64, z: f64) {
        self.target = [x, y, z];
        self.desired = [x, y, z];
        self.interpolating = false;
        let dx = self.position[0] - x;
        let dy = self.position[1] - y;
        let dz = self.position[2] - z;
        let len = (dx * dx + dy * dy + dz * dz).sqrt();
        if len > 0.01 {
            self.dist = len as f32;
            self.pitch_deg = ((dz / len).clamp(-0.999, 0.999).asin().to_degrees()) as f32;
            self.yaw_deg = (dx.atan2(-dy).to_degrees()) as f32;
            self.pitch_deg = self.pitch_deg.clamp(-88.0, 88.0);
            while self.yaw_deg >= 360.0 {
                self.yaw_deg -= 360.0;
            }
            while self.yaw_deg < 0.0 {
                self.yaw_deg += 360.0;
            }
        }
    }

    /// VSE `SetOrbitTargetSmooth`: desired only; `update` glides there.
    pub fn set_target_smooth(&mut self, x: f64, y: f64, z: f64) {
        self.desired = [x, y, z];
        self.interpolating = true;
    }

    /// VSE `SetLookAtDirect`: pose as given, target settled.
    pub fn set_look_at(&mut self, pos: [f64; 3], tgt: [f64; 3]) {
        self.position = pos;
        self.target = tgt;
        self.desired = tgt;
        self.interpolating = false;
    }

    /// VSE `InitFreeCamFromChase`: same pose math as the chase (yaw-only
    /// basis, chase yaw/pitch/dist, smoothed pivot Z), then detach — after
    /// this the vehicle is never read again. Orbit internals resynced from
    /// the pose so the first drag orbits the new target. `base` is the
    /// pivot base (already bus-centred); `pivot_z` the filtered height.
    pub fn seed_from_chase(
        &mut self,
        base: [f64; 3],
        pivot_z: f64,
        heading_rad: f64,
        chase_yaw_deg: f32,
        chase_pitch_deg: f32,
        chase_dist: f32,
    ) {
        let (cam, tgt) = chase_points(
            base[0], base[1], pivot_z, heading_rad, chase_yaw_deg, chase_pitch_deg,
            chase_dist,
        );
        self.fov_deg = VSE_CHASE_FOV;
        self.set_look_at(cam, tgt);
        self.set_target(tgt[0], tgt[1], tgt[2]);
    }

    /// VSE `UpdateCameraPositionFromOrbit`: spherical around target (Z-up,
    /// pitch = elevation, yaw 0 = facing +Y from −Y).
    fn reposition(&mut self) {
        let pr = (self.pitch_deg.to_radians()) as f64;
        let yr = (self.yaw_deg.to_radians()) as f64;
        let d = self.dist as f64;
        self.position = [
            self.target[0] + d * pr.cos() * yr.sin(),
            self.target[1] - d * pr.cos() * yr.cos(),
            self.target[2] + d * pr.sin(),
        ];
    }

    /// View direction target→camera inverted (camera→target), as openOMSI
    /// yaw (`atan2(x, y)`) / pitch (`asin(z)`) — derived from the VSE points,
    /// never sign-hacked.
    pub fn view_angles(&self) -> (f32, f32) {
        let dx = self.target[0] - self.position[0];
        let dy = self.target[1] - self.position[1];
        let dz = self.target[2] - self.position[2];
        let len = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-9);
        (
            dx.atan2(dy).to_degrees() as f32,
            (dz / len).clamp(-1.0, 1.0).asin().to_degrees() as f32,
        )
    }

    /// Right/up basis around the view direction (VSE `GetRightVector` /
    /// `GetUpVector`, ±Y fallback at the poles) for `on_pan`.
    pub fn basis(&self) -> ([f64; 3], [f64; 3]) {
        let dx = self.target[0] - self.position[0];
        let dy = self.target[1] - self.position[1];
        let dz = self.target[2] - self.position[2];
        let len = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-9);
        let f = [dx / len, dy / len, dz / len];
        let (wx, wy, wz) = if f[2].abs() > 0.999 {
            (0.0, if f[2] > 0.0 { -1.0 } else { 1.0 }, 0.0)
        } else {
            (0.0, 0.0, 1.0)
        };
        let mut r = [
            f[1] * wz - f[2] * wy,
            f[2] * wx - f[0] * wz,
            f[0] * wy - f[1] * wx,
        ];
        let rl = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt().max(1e-9);
        r = [r[0] / rl, r[1] / rl, r[2] / rl];
        let mut u = [
            r[1] * f[2] - r[2] * f[1],
            r[2] * f[0] - r[0] * f[2],
            r[0] * f[1] - r[1] * f[0],
        ];
        let ul = (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt().max(1e-9);
        u = [u[0] / ul, u[1] / ul, u[2] / ul];
        (r, u)
    }
}

/// VSE terrain march (`PathTool::ScreenPointToTerrain`): from `t = 0.05`,
/// step `clamp(0.75/horiz, 0.05, 2.0)` to `4000`, then 14 bisections.
/// `ground(x, y)` is the terrain service (`SampleGround` over there).
pub fn vse_pick_ground(
    o: [f64; 3],
    d: [f64; 3],
    ground: &dyn Fn(f64, f64) -> Option<f64>,
) -> Option<[f64; 3]> {
    let above = |t: f64| -> Option<bool> {
        let (x, y, z) = (o[0] + d[0] * t, o[1] + d[1] * t, o[2] + d[2] * t);
        ground(x, y).map(|g| z > g)
    };
    let horiz = (d[0] * d[0] + d[1] * d[1]).sqrt().max(1e-9);
    let step = (0.75 / horiz).clamp(0.05, 2.0);
    let mut t = VSE_PICK_T_MIN;
    let mut last = 0.0;
    while t < VSE_PICK_T_MAX {
        if above(t) == Some(false) {
            let (mut a, mut b) = (last, t);
            for _ in 0..VSE_PICK_BISECT {
                let m = (a + b) * 0.5;
                if above(m).unwrap_or(true) {
                    a = m;
                } else {
                    b = m;
                }
            }
            let (x, y) = (o[0] + d[0] * b, o[1] + d[1] * b);
            return ground(x, y).map(|g| [x, y, g]);
        }
        last = t;
        t += step;
    }
    None
}

/// VSE `Z = 0` fallback when the march misses and the ray points down.
pub fn vse_plane_fallback(o: [f64; 3], d: [f64; 3]) -> Option<[f64; 3]> {
    if d[2].abs() <= 1e-5 {
        return None;
    }
    let t = -o[2] / d[2];
    if t <= 0.0 {
        return None;
    }
    Some([o[0] + d[0] * t, o[1] + d[1] * t, 0.0])
}

/// Shared chase pose math (stateless): `off = [D·sinCY·cosCP,
/// −D·cosCY·cosCP, 1.6 + D·sinCP]`, `cam = veh + yr·offX + yf·offY`,
/// `tgt = (vehX, vehY, pivotZ + 1.4)`. Used by `VseChase::pose` (filtered)
/// and by stateless callers (offscreen first frame = filter snap).
pub fn chase_points(
    px: f64,
    py: f64,
    pivot_z: f64,
    heading_rad: f64,
    chase_yaw_deg: f32,
    chase_pitch_deg: f32,
    dist: f32,
) -> ([f64; 3], [f64; 3]) {
    let (cy, cp) = (
        chase_yaw_deg.to_radians() as f64,
        chase_pitch_deg.to_radians() as f64,
    );
    let d = dist as f64;
    let off = [
        d * cy.sin() * cp.cos(),
        -d * cy.cos() * cp.cos(),
        VSE_CHASE_OFF_Z as f64 + d * cp.sin(),
    ];
    let (sy, cyaw) = heading_rad.sin_cos();
    let cam = [
        px + cyaw * off[0] + sy * off[1],
        py - sy * off[0] + cyaw * off[1],
        pivot_z + off[2],
    ];
    let tgt = [px, py, pivot_z + VSE_CHASE_TGT_Z as f64];
    (cam, tgt)
}

/// Pivot base (VSE yaw-only basis `yr/yf`): the bus-authored centre rotated
/// by heading alone — never by body pitch/bank, so suspension bounce and
/// body roll never reach the camera. `position.z` is averaged terrain height
/// (smooth); body motion lives in the attitude, which is excluded here.
pub fn pivot_base(pos: [f64; 3], heading_rad: f64, center: [f32; 3]) -> [f64; 3] {
    let (sy, cyaw) = heading_rad.sin_cos();
    [
        pos[0] + cyaw * center[0] as f64 + sy * center[1] as f64,
        pos[1] - sy * center[0] as f64 + cyaw * center[1] as f64,
        pos[2] + center[2] as f64,
    ]
}

/// Fly-mode wheel: throttle for the free camera (up faster, down slower).
pub fn vse_fly_speed(speed_ms: f32, wheel: f32) -> f32 {
    let factor = if wheel > 0.0 { 1.15 } else { 1.0 / 1.15 };
    (speed_ms * factor).clamp(1.0, 200.0)
}

/// Chase ease length shared with the head glide.
pub fn chase_ease_secs() -> f32 {
    VSE_CAM_EASE_SECS
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chase() -> VseChase {
        VseChase::new(6.08)
    }

    #[test]
    fn chase_starts_like_vse_start() {
        let c = chase();
        assert_eq!((c.yaw_deg, c.pitch_deg), (0.0, 12.0));
        assert!((c.dist - 13.12).abs() < 1e-4, "{}", c.dist);
        assert!(!c.pivot_valid);
        // a new bus restarts everything; same bus keeps the driver's angles.
        let mut c2 = chase();
        c2.yaw_deg = 30.0;
        c2.ensure_bus(6.08);
        assert_eq!(c2.yaw_deg, 30.0);
        c2.ensure_bus(12.0);
        assert_eq!((c2.yaw_deg, c2.pitch_deg), (0.0, 12.0));
        assert!((c2.dist - 22.0).abs() < 1e-4, "{}", c2.dist);
    }

    #[test]
    fn chase_orbit_zoom_reset_match_vse() {
        let mut c = chase();
        c.on_orbit_px(10.0, 10.0);
        assert!((c.yaw_deg + 3.5).abs() < 1e-4 && (c.pitch_deg - 15.5).abs() < 1e-4);
        c.on_orbit_px(0.0, 10000.0);
        assert_eq!(c.pitch_deg, 75.0);
        c.on_orbit_px(0.0, -10000.0);
        assert_eq!(c.pitch_deg, -10.0);
        c.on_zoom(1.0);
        assert!((c.dist - 11.62).abs() < 1e-4, "{}", c.dist);
        c.on_zoom(100.0);
        assert_eq!(c.dist, 4.0);
        c.on_zoom(-100.0);
        assert_eq!(c.dist, 40.0);
        // reset keeps the distance (VSE never zeroes it).
        c.reset();
        assert_eq!((c.yaw_deg, c.pitch_deg, c.dist), (0.0, 12.0, 40.0));
        // degree sources apply directly with the same clamp.
        c.nudge_deg(400.0, -100.0);
        assert_eq!(c.pitch_deg, -10.0);
    }

    #[test]
    fn pivot_base_uses_yaw_only() {
        // heading 0: centre passes through; heading 90°: forward swings east.
        // No body pitch/bank input exists — bounce cannot enter by construction.
        let p = pivot_base([10.0, 20.0, 5.0], 0.0, [0.0, 0.0, 1.2]);
        assert!((p[0] - 10.0).abs() < 1e-9 && (p[1] - 20.0).abs() < 1e-9);
        assert!((p[2] - 6.2).abs() < 1e-6, "{p:?}");
        let q = pivot_base([0.0, 0.0, 3.0], std::f64::consts::FRAC_PI_2, [1.0, 2.0, 1.2]);
        assert!((q[0] - 2.0).abs() < 1e-6 && (q[1] + 1.0).abs() < 1e-6);
        assert!((q[2] - 4.2).abs() < 1e-6, "{q:?}");
    }

    #[test]
    fn chase_pose_matches_vse_update() {
        // bus at (100, 200, 50) + centre (0,0,1.2), heading 0, defaults:
        // pivot snaps to 51.2, tgt = (100, 200, 52.6); cam looks down ~12.8°.
        let mut c = chase();
        let (cam, tgt) = c.pose([100.0, 200.0, 50.0], 0.0, [0.0, 0.0, 1.2], 1.0 / 60.0);
        assert!(c.pivot_valid && (c.pivot_z - 51.2).abs() < 1e-6);
        assert!((cam[1] - 187.167).abs() < 1e-2, "{cam:?}");
        assert!((cam[2] - 55.528).abs() < 1e-2, "{cam:?}");
        assert!((tgt[2] - 52.6).abs() < 1e-6, "{tgt:?}");
        assert!(cam[2] > tgt[2]);
        // second frame filters toward the new height (tau 0.30), never snaps.
        let (_, _) = c.pose([100.0, 200.0, 60.0], 0.0, [0.0, 0.0, 1.2], 1.0 / 60.0);
        assert!(c.pivot_z > 51.2 && c.pivot_z < 61.2, "{}", c.pivot_z);
        // reset invalidates: next pose snaps again.
        c.reset();
        let (_, _) = c.pose([0.0, 0.0, 10.0], 0.0, [0.0, 0.0, 0.0], 1.0 / 60.0);
        assert!((c.pivot_z - 10.0).abs() < 1e-9);
    }

    #[test]
    fn free_orbit_repositions_like_vse() {
        let mut o = VseFreeOrbit::default();
        o.set_target(0.0, 0.0, 10.0);
        o.dist = 10.0;
        o.yaw_deg = 0.0;
        o.pitch_deg = 0.0;
        o.update(1.0 / 60.0);
        assert!((o.position[0] - 0.0).abs() < 1e-6);
        assert!((o.position[1] + 10.0).abs() < 1e-6);
        assert!((o.position[2] - 10.0).abs() < 1e-6);
        // drag down raises, drag right turns left; clamp ±88; yaw wraps.
        o.on_orbit(10.0, 0.0);
        assert!((o.pitch_deg - 3.5).abs() < 1e-4, "{}", o.pitch_deg);
        o.on_orbit(0.0, 10.0);
        assert!((o.yaw_deg - 356.5).abs() < 1e-4, "{}", o.yaw_deg);
        o.pitch_deg = 80.0;
        o.on_orbit(1000.0, 0.0);
        assert_eq!(o.pitch_deg, 88.0);
    }

    #[test]
    fn free_target_sync_never_snaps_back() {
        // VSE `SetOrbitTarget`: keep position, resync angles from the pose.
        let mut o = VseFreeOrbit::default();
        o.set_look_at([0.0, -10.0, 5.0], [0.0, 0.0, 5.0]);
        o.set_target(0.0, 0.0, 5.0);
        assert!((o.dist - 10.0).abs() < 1e-4);
        assert!((o.pitch_deg - 0.0).abs() < 1e-4);
        assert!((o.yaw_deg - 0.0).abs() < 1e-4);
        // smooth retarget glides (blend 1−exp(−10dt)) then settles exactly.
        o.set_target_smooth(10.0, 0.0, 5.0);
        assert!(o.interpolating);
        for _ in 0..600 {
            o.update(1.0 / 60.0);
        }
        assert!(!o.interpolating);
        assert!((o.target[0] - 10.0).abs() < 1e-9);
    }

    #[test]
    fn free_zoom_dolly_match_vse_factors() {
        let mut o = VseFreeOrbit::default();
        o.dist = 10.0;
        o.on_zoom(1.0);
        assert!((o.dist - 8.8).abs() < 1e-4);
        o.on_zoom(-1.0);
        assert!((o.dist - 8.8 * 1.14).abs() < 1e-3);
        // multiplicative: repeated wheel-up walks to the floor, never past it.
        for _ in 0..200 {
            o.on_zoom(1.0);
        }
        assert_eq!(o.dist, 0.1);
        o.dist = 10.0;
        o.dist = 10.0;
        o.on_dolly(-100.0);
        assert!(o.dist > 10.0, "{}", o.dist);
        for _ in 0..200 {
            o.on_dolly(10000.0);
        }
        assert_eq!(o.dist, 0.05);
    }

    #[test]
    fn free_seed_matches_chase_pose() {
        // seed from a chase state == chase pose, then detached (vehicle args
        // reused only as numbers; later updates never read the vehicle).
        let mut c = VseChase::new(6.08);
        let base = pivot_base([100.0, 200.0, 50.0], 0.0, [0.0, 0.0, 1.2]);
        // same numbers the pose path produces (shared core, no drift).
        let (cam, _) = c.pose([100.0, 200.0, 50.0], 0.0, [0.0, 0.0, 1.2], 1.0 / 60.0);
        let (cy, cp, d) = c.seed();
        let mut o = VseFreeOrbit::default();
        o.seed_from_chase(base, c.pivot_z, 0.0, cy, cp, d);
        assert!((o.position[0] - cam[0]).abs() < 1e-6);
        assert!((o.position[1] - cam[1]).abs() < 1e-6);
        assert!((o.position[2] - cam[2]).abs() < 1e-6);
        assert_eq!(o.fov_deg, 60.0);
        assert!(!o.interpolating);
        // view angles derived from the points (openOMSI convention).
        let (yaw, pitch) = o.view_angles();
        assert!(yaw.abs() < 1e-3, "{yaw}");
        assert!(pitch < -11.0 && pitch > -14.0, "{pitch}");
    }

    #[test]
    fn ground_march_matches_vse_terrain() {
        let g = |x: f64, y: f64| Some(0.0 + 0.001 * (x + y));
        // straight down onto near-flat ground.
        let hit = vse_pick_ground([0.0, 0.0, 10.0], [0.0, 0.0, -1.0], &g).unwrap();
        assert!((hit[2] - 0.0).abs() < 0.05, "{hit:?}");
        // parallel ray never lands.
        assert!(vse_pick_ground([0.0, 0.0, 10.0], [1.0, 0.0, 0.0], &g).is_none());
        // Z=0 fallback only pointing down with positive distance.
        assert!(vse_plane_fallback([1.0, 2.0, 5.0], [0.0, 0.0, -1.0]).unwrap()[2] == 0.0);
        assert!(vse_plane_fallback([0.0, 0.0, 5.0], [0.0, 0.0, 1.0]).is_none());
        assert!(vse_plane_fallback([0.0, 0.0, 5.0], [1.0, 0.0, 0.0]).is_none());
    }

    #[test]
    fn chase_ease_secs_matches_head_glide() {
        assert!((chase_ease_secs() - 0.54).abs() < 1e-6);
    }

    #[test]
    fn fly_wheel_throttles_both_ways_with_floor_and_ceiling() {
        assert!((vse_fly_speed(30.0, 1.0) - 34.5).abs() < 1e-4);
        assert!((vse_fly_speed(30.0, -1.0) - 30.0 / 1.15).abs() < 1e-4);
        assert_eq!(vse_fly_speed(1.0, -10.0), 1.0);
        assert_eq!(vse_fly_speed(200.0, 10.0), 200.0);
    }
}
