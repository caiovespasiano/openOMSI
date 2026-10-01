//! Camera (F1-F4), mouse-drive steering, keyboard steering,
//! return-to-centre, FFB and Ackermann helpers.
//!
//! What this module implements (see `docs/VSE_TO_OPENOMSI_CAMERA_STEERING_PORT.md`):
//! - head glide `0.54s`, `s = 1-(1-t)^3`
//! - interior orbit/zoom
//! - `FOV = base/(1+5.5*z)`, min 4
//! - F3 chase `offZ 1.6 / tgt 1.4 / tauZ 0.30`
//! - F4 `blend = 1-exp(-10*dt)`
//! - terrain march + 14 bisections, fallback `Z=0`
//! - mouse `edge/corner/slew tau 0.12`
//! - keyboard rates + return-in-motion
//! - FFB `-0.025*delta*v^2`, clamp `±15`
//! - Ackermann `r_in = L/tan(inner)`
//!
//! Everything here is pure and desktop-only in use (called from `omsi-app`,
//! never from `omsi-sim`/`omsi-render`), so mobile/touch paths are untouched.

/// F1 camera switch glide, seconds (was 0.45s).
pub const VSE_CAM_EASE_SECS: f32 = 0.54;
/// F1 look sensitivity, deg/px (`0.35 * 0.70`).
pub const VSE_LOOK_SENS: f32 = 0.35 * 0.70;
/// F1 look clamps, degrees.
pub const VSE_LOOK_YAW_CLAMP: f32 = 90.0;
pub const VSE_LOOK_PITCH_MIN: f32 = -35.0;
pub const VSE_LOOK_PITCH_MAX: f32 = 35.0;
/// F1/F2 zoom drag: pixels for full zoom, delivered intent.
pub const VSE_ZOOM_PX: f32 = 364.0;
pub const VSE_ZOOM_INTENT: f32 = 0.70;
/// Interior FOV: `base / (1 + 5.5*z)`, min 4 (full zoom 6.5x).
pub const VSE_FOV_ZOOM_K: f32 = 5.5;
pub const VSE_FOV_MIN: f32 = 4.0;

/// F3 chase: sensitivity deg/px, clamps, FOV, offsets, smoothing.
pub const VSE_CHASE_SENS: f32 = 0.35;
pub const VSE_CHASE_DIST_MIN: f32 = 4.0;
pub const VSE_CHASE_DIST_MAX: f32 = 40.0;
pub const VSE_CHASE_PITCH_MIN: f32 = -10.0;
pub const VSE_CHASE_PITCH_MAX: f32 = 75.0;
pub const VSE_CHASE_FOV: f32 = 60.0;
pub const VSE_CHASE_OFF_Z: f32 = 1.6;
pub const VSE_CHASE_TGT_Z: f32 = 1.4;
pub const VSE_CHASE_TAU_Z: f32 = 0.30;
pub const VSE_CHASE_ZOOM_K: f32 = 1.5;

/// F4 free target smoothing: `blend = 1-exp(-10*dt)`.
pub const VSE_F4_SMOOTH_K: f32 = 10.0;
/// F4 terrain pick: march `t = 0.05..4000`, step `clamp(0.75/horiz,0.05,2.0)`.
pub const VSE_PICK_T_MIN: f64 = 0.05;
pub const VSE_PICK_T_MAX: f64 = 4000.0;
pub const VSE_PICK_BISECT: u32 = 14;

/// Keyboard steering: `base = 0.336*sens`, `min = 0.145*sens`, sens default 0.70.
pub const VSE_STEER_BASE: f32 = 0.336;
pub const VSE_STEER_MIN: f32 = 0.145;
pub const VSE_STEER_SENS_DEFAULT: f32 = 0.70;
/// Mouse steering slew: `tau 0.12s`, `slew 2.5/s`.
pub const VSE_MOUSE_TAU: f32 = 0.12;
pub const VSE_MOUSE_SLEW: f32 = 2.5;
/// Mouse pedals deadzone.
pub const VSE_PEDAL_DEAD: f32 = 0.10;
/// Return-to-centre base `0.22 * v_ramp * exp(-0.020*v_kmh)`.
pub const VSE_RETURN_BASE: f32 = 0.22;
/// FFB `T = -0.025*delta*v^2`, clamp `±15 N·m`.
pub const VSE_FFB_K: f32 = 0.025;
pub const VSE_FFB_MAX: f32 = 15.0;
/// Steering wheel half lock fallback, degrees (1700/2).
pub const VSE_WHEEL_HALF_DEFAULT: f32 = 850.0;
/// Max road-wheel angle fallback, degrees.
pub const VSE_MAX_WHEEL_DEG_DEFAULT: f32 = 50.0;
/// Directional (rear-steer) ratio, fade 25→40 km/h.
pub const VSE_DIRECTIONAL_RATIO: f32 = -0.3;

/// Ease-out cubic `s = 1-(1-t)^3`.
pub fn vse_ease_out(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    let u = 1.0 - t;
    1.0 - u * u * u
}

/// Wrap camera index.
pub fn vse_wrap_index(current: usize, count: usize, dir: i32) -> usize {
    if count == 0 {
        return 0;
    }
    let next = current as i64 + dir as i64;
    next.rem_euclid(count as i64) as usize
}

/// Shortest yaw delta in degrees (`std::remainder(d, 360)`).
pub fn vse_shortest_yaw(from_deg: f32, to_deg: f32) -> f32 {
    (to_deg - from_deg + 180.0).rem_euclid(360.0) - 180.0
}

/// Interior display FOV.
pub fn vse_interior_fov(base_deg: f32, zoom: f32) -> f32 {
    let z = zoom.clamp(0.0, 1.0);
    (base_deg / (1.0 + VSE_FOV_ZOOM_K * z)).max(VSE_FOV_MIN)
}

/// Interior orbit: F1 only, ignored while transitioning.
pub fn vse_interior_orbit(yaw: f32, pitch: f32, dx: f32, dy: f32, zoom: f32) -> (f32, f32) {
    let stab = 1.0 - 0.25 * zoom.clamp(0.0, 1.0);
    let y = (yaw + dx * VSE_LOOK_SENS * stab).clamp(-VSE_LOOK_YAW_CLAMP, VSE_LOOK_YAW_CLAMP);
    let p = (pitch - dy * VSE_LOOK_SENS * stab).clamp(VSE_LOOK_PITCH_MIN, VSE_LOOK_PITCH_MAX);
    (y, p)
}

/// Interior zoom drag: drag down (`dy>0`) zooms in.
pub fn vse_interior_zoom(zoom: f32, dy: f32) -> f32 {
    (zoom + dy * VSE_ZOOM_INTENT / VSE_ZOOM_PX).clamp(0.0, 1.0)
}

/// F3 initial distance (`wb*1.5+4`, `wb>1?wb:6.08`).
pub fn vse_chase_dist0(wheelbase_m: f32) -> f32 {
    let wb = if wheelbase_m > 1.0 { wheelbase_m } else { 6.08 };
    wb * 1.5 + 4.0
}

/// F3 orbit: `yaw -= dx*0.35`, `pitch += dy*0.35`.
pub fn vse_chase_orbit(yaw: f32, pitch: f32, dx: f32, dy: f32) -> (f32, f32) {
    let y = yaw - dx * VSE_CHASE_SENS;
    let p = (pitch + dy * VSE_CHASE_SENS).clamp(VSE_CHASE_PITCH_MIN, VSE_CHASE_PITCH_MAX);
    (y, p)
}

/// F3 zoom: `dist -= delta*1.5`, clamp `4..40`.
pub fn vse_chase_zoom(dist: f32, delta: f32) -> f32 {
    (dist - delta * VSE_CHASE_ZOOM_K).clamp(VSE_CHASE_DIST_MIN, VSE_CHASE_DIST_MAX)
}

/// F3 chase offset in vehicle-yaw frame.
pub fn vse_chase_offset(dist: f32, yaw_deg: f32, pitch_deg: f32) -> [f32; 3] {
    let (cy, cp) = (yaw_deg.to_radians(), pitch_deg.to_radians());
    [
        dist * cy.sin() * cp.cos(),
        -dist * cy.cos() * cp.cos(),
        VSE_CHASE_OFF_Z + dist * cp.sin(),
    ]
}

/// Low-pass pivot height (`tau 0.30`): `filt += (1-exp(-dt/tau)) * (z - filt)`.
pub fn vse_chase_z_filter(current: Option<f32>, z: f32, dt: f32) -> f32 {
    match current {
        None => z,
        Some(f) => {
            let safe = if dt > 1e-4 { dt } else { 1.0 / 60.0 };
            let lambda = 1.0 - (-safe / VSE_CHASE_TAU_Z).exp();
            f + lambda * (z - f)
        }
    }
}

/// F4 target smoothing.
pub fn vse_f4_blend(dt: f32) -> f32 {
    1.0 - (-VSE_F4_SMOOTH_K * dt.max(0.0)).exp()
}

/// Mouse steering target:
/// edge `0.75*|nx|`, corner ramps `lin(0.20,1,|nx|)*lin(0.10,1,|ny|)`.
pub fn vse_mouse_target(nx: f32, ny: f32) -> f32 {
    let ax = nx.abs().clamp(0.0, 1.0);
    let ay = ny.abs().clamp(0.0, 1.0);
    let edge = 0.75 * ax;
    let lin = |a: f32, b: f32, x: f32| ((x - a) / (b - a)).clamp(0.0, 1.0);
    let corner = lin(0.20, 1.0, ax) * lin(0.10, 1.0, ay);
    let mag = edge + (1.0 - edge) * corner;
    if nx >= 0.0 {
        mag
    } else {
        -mag
    }
}

/// Mouse pedals: throttle deadzone `0.10`/full top,
/// brake deadzone `0.10`/full at 70% down.
pub fn vse_mouse_pedals(ny: f32) -> (f32, f32) {
    let ay = ny.clamp(-1.0, 1.0);
    let thr = if ay <= VSE_PEDAL_DEAD {
        0.0
    } else {
        ((ay - VSE_PEDAL_DEAD) / 0.90).min(1.0)
    };
    let dn = -ay;
    let brk = if dn <= VSE_PEDAL_DEAD {
        0.0
    } else {
        ((dn - VSE_PEDAL_DEAD) / 0.60).min(1.0)
    };
    (thr, brk)
}

/// Mouse slew without teleport (`tau 0.12 s`, `slew 2.5/s`, `dt` seconds).
pub fn vse_mouse_slew(sm: f32, target: f32, dt: f32) -> f32 {
    let lagged = sm + (target - sm) * (dt / VSE_MOUSE_TAU).min(1.0);
    let max_step = VSE_MOUSE_SLEW * dt;
    (sm + (lagged - sm).clamp(-max_step, max_step)).clamp(-1.0, 1.0)
}

/// Neutral-until-moved gate: while the cursor
/// still sits where driving was switched on, target and pedals stay neutral —
/// no first-frame jump from wherever the cursor happened to be.
pub fn vse_anchor_neutral(anchor: Option<(f32, f32)>, cursor: (f32, f32)) -> bool {
    anchor.is_some_and(|a| a == cursor)
}

/// Corner-boost ratchet:
/// only the excess over `0.75*|nx|` latches; the hold clears at centre, on
/// side change or when `|nx|<0.5` — but the TARGET is always recomputed as
/// `sign*(lateral + |hold|)`, so it passes through intact even where no hold
/// latches (no deadzone anywhere: steering is proportional from zero, `0.10`
/// is pedals-only). While latched the boost persists (`lateral + |hold|`)
/// even when the raw target falls back to the edge — the wheel never unwinds
/// a corner visit on its own.
pub fn vse_boost_hold(hold: f32, nx: f32, target: f32) -> (f32, f32) {
    let ax = nx.abs();
    let lateral = 0.75 * ax;
    let sgn = if target > 0.0 {
        1
    } else if target < 0.0 {
        -1
    } else {
        0
    };
    let sgn_hold = if hold > 0.0 {
        1
    } else if hold < 0.0 {
        -1
    } else {
        0
    };
    // (the inner `if (sgnHold==0||sgn!=sgnHold) hold=0` inside the
    // boost branch is dead: side-change with a live hold is cleared above, a
    // zero hold is already zero.)
    let mut h = hold;
    if sgn == 0 || ax < 0.5 || (sgn_hold != 0 && sgn != sgn_hold) {
        h = 0.0;
    } else if target.abs() > lateral {
        let boost = target.abs() - lateral;
        let held = h.abs().max(boost);
        h = if sgn > 0 { held } else { -held };
    }
    let t = if sgn > 0 {
        lateral + h.abs()
    } else if sgn < 0 {
        -(lateral + h.abs())
    } else {
        0.0
    };
    (h, t.clamp(-1.0, 1.0))
}

/// Whether an analog stick actually drives the wheel this frame: flagged as a
/// stick AND deflected. A connected-but-centred pad (`poll` deadzones it to
/// exactly `0.0`) must not enter the 1.2 s chase in `app_events` — otherwise
/// it drags the wheel toward zero while the keys integrate up, and the merge
/// gate flips winner every few ms (the A/D "fight").
pub fn vse_stick_chase(stick: bool, x: Option<f32>) -> bool {
    stick && x.is_some_and(|v| v != 0.0)
}

fn smoothstep01(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Keyboard steer/unsteer rates per second.
pub fn vse_keyboard_rates(v_kmh: f32, input_mag: f32, sens: f32) -> (f32, f32, f32) {
    let sens = sens.clamp(0.20, 2.00);
    let base = VSE_STEER_BASE * sens;
    let min = VSE_STEER_MIN * sens;
    let turn = 0.38 + 0.62 * (-0.045 * v_kmh).exp();
    let unwind_speed = 0.48 + 0.52 * (-0.032 * v_kmh).exp();
    let low = 2.0 - smoothstep01((v_kmh - 20.0) / 25.0);
    let steer = (base * low * turn).clamp(min, base * 2.2);
    let unwind_base = (base * low * unwind_speed).clamp(min, base * 2.4);
    let mag = input_mag.clamp(0.0, 1.0);
    let pitch = vse_hermite_pitch(mag);
    let v_ramp = (v_kmh / 1.5).clamp(0.0, 1.0);
    let stopped = pitch.min(1.25);
    let mult = stopped + (pitch - stopped) * v_ramp;
    let blend = ((mult - 1.0) / 1.55).clamp(0.0, 1.0);
    let fast = (unwind_base * mult).max(base * mult);
    let unsteer = unwind_base * (1.0 - blend) + fast * blend;
    (steer, unsteer, mult)
}

/// Hermite C1 unwind pitch `1.0/1.35/2.15/2.75/3.25`.
pub fn vse_hermite_pitch(mag: f32) -> f32 {
    let hermite = |t: f32, p0: f32, p1: f32, m0: f32, m1: f32, h: f32| {
        let (t2, t3) = (t * t, t * t * t);
        (2.0 * t3 - 3.0 * t2 + 1.0) * p0
            + (-2.0 * t3 + 3.0 * t2) * p1
            + (t3 - 2.0 * t2 + t) * (h * m0)
            + (t3 - t2) * (h * m1)
    };
    if mag <= 0.20 {
        hermite(mag / 0.20, 1.0, 1.35, 1.2, 2.0, 0.20)
    } else if mag <= 0.50 {
        hermite((mag - 0.20) / 0.30, 1.35, 2.15, 2.0, 2.2, 0.30)
    } else if mag <= 0.75 {
        hermite((mag - 0.50) / 0.25, 2.15, 2.75, 2.2, 2.0, 0.25)
    } else {
        2.75 + 2.0 * (mag - 0.75)
    }
}

/// Return-to-centre step for `dt` seconds.
/// Returns the new input. Zero rate while standing (`v_kmh == 0`).
pub fn vse_return_step(input: f32, v_kmh: f32, half_deg: f32, pitch_mult: f32, dt: f32) -> f32 {
    if input.abs() <= 0.0001 || dt <= 0.0 {
        return input;
    }
    let v_ramp = (v_kmh / 5.0).clamp(0.0, 1.0);
    let base = VSE_RETURN_BASE * v_ramp * (-0.020 * v_kmh).exp();
    if base <= 1e-5 {
        return input;
    }
    let half = if half_deg > 100.0 {
        half_deg
    } else {
        VSE_WHEEL_HALF_DEFAULT
    };
    let angle = input.abs() * half;
    let factor = if angle > 360.0 {
        1.0
    } else if angle > 45.0 {
        0.20 + 0.80 * smoothstep01((angle - 45.0) / 315.0)
    } else if angle > 5.0 {
        0.05 + 0.15 * smoothstep01((angle - 5.0) / 40.0)
    } else {
        (angle / 100.0).max(0.015)
    };
    let auto = 0.33 + 0.67 * pitch_mult;
    let step = base * factor * auto * dt;
    if input.abs() <= step || angle <= 0.05 {
        0.0
    } else if input > 0.0 {
        input - step
    } else {
        input + step
    }
}

/// Self-aligning torque, N·m (single source of truth).
pub fn vse_ffb_nm(steer_rad: f32, v_mps: f32) -> f32 {
    (-VSE_FFB_K * steer_rad * v_mps * v_mps).clamp(-VSE_FFB_MAX, VSE_FFB_MAX)
}

/// Ackermann inner/outer magnitudes.
/// `r_in = L0/tan(inner)`, `r_out = r_in + track`, `outer = atan(L0/r_out)`.
pub fn vse_ackermann(delta_cmd: f32, max_steer_rad: f32, l0_m: f32, track_m: f32) -> (f32, f32) {
    let max = max_steer_rad.max(0.1);
    let l0 = l0_m.max(1.5);
    let track = track_m.max(1.2);
    let u = (delta_cmd / max).clamp(-1.0, 1.0);
    let inner = u.abs() * max;
    if inner <= 1e-5 {
        return (0.0, 0.0);
    }
    let r_in = l0 / inner.tan();
    let r_out = r_in + track;
    (inner, (l0 / r_out).atan())
}

/// Bicycle effective radius `R = L/(tan(delta)*f)`.
pub fn vse_bicycle_radius(l_m: f32, delta_rad: f32, f_curva: f32) -> f32 {
    let t = delta_rad.tan() * f_curva;
    if t.abs() < 1e-6 {
        f32::INFINITY
    } else {
        l_m / t
    }
}

/// Curve authority table.
pub fn vse_curve_authority(layout: &str) -> f32 {
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

/// Directional rear steer `delta_inv = ratio*delta*fade`, fade `25→40 km/h`.
pub fn vse_directional(delta: f32, v_kmh: f32, ratio: f32, max_rad: f32) -> f32 {
    let fade = ((40.0 - v_kmh) / 15.0).clamp(0.0, 1.0);
    (ratio * delta * fade).clamp(-max_rad, max_rad)
}

/// Yaw target `omega = (v/L1)*tan(delta)*f`, clamp `±0.75*g/max(1,|v|)`.
pub fn vse_yaw_target(v_mps: f32, l1_m: f32, delta_rad: f32, f_curva: f32) -> f32 {
    let raw = (v_mps / l1_m.max(1.0)) * delta_rad.tan() * f_curva;
    let lim = 0.75 * 9.80665 / v_mps.abs().max(1.0);
    raw.clamp(-lim, lim)
}

// ---------------------------------------------------------------------------
// Runtime routing tables.
//
// These capture the dispatch decisions in `app_events.rs`
// (`DeviceEvent::MouseMotion`, `App::wheel`) so the "which system consumes this
// input in this context" question is answered by tests, not by code reading.
// Changing behavior means changing these tables AND their call sites together.
// ---------------------------------------------------------------------------

/// Which mouse button holds a drag (`None` = no button).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VseDragButton {
    Middle,
    Right,
}

/// What a mouse-motion burst does in a view.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VseDragAction {
    /// Precision zoom: vertical drag, down = in.
    /// F1/F3/F4. Never teleports; `Space` eases it back (see `vse_reset_blend`).
    ZoomPrecision,
    /// Precision zoom, legacy multiplier path — F2 only, byte-identical;
    /// frozen F2 behavior.
    ZoomLegacy,
    /// Orbit/look.
    Look,
    /// Past-the-edge nudge while mouse-driving.
    MouseEdge,
    /// Swallowed (menu open, no button, no drive).
    Nothing,
}

/// Drag routing: MMB+drag orbits every mode;
/// RMB+drag vertical is precision zoom in `driver`/`outside`/`free`;
/// never in `pax`, never global.
pub fn vse_drag_action(
    btn: Option<VseDragButton>,
    view: &str,
    mouse_drive: bool,
    menu_open: bool,
) -> VseDragAction {
    match btn {
        Some(VseDragButton::Right) if view == "driver" => VseDragAction::ZoomPrecision,
        Some(VseDragButton::Right) if view == "pax" => VseDragAction::ZoomLegacy,
        Some(VseDragButton::Right) if matches!(view, "outside" | "free") => {
            VseDragAction::ZoomPrecision
        }
        Some(_) => VseDragAction::Look,
        None if mouse_drive && !menu_open => VseDragAction::MouseEdge,
        None => VseDragAction::Nothing,
    }
}

/// What the wheel does in the views the chase machine does not own
/// (after editor/placing/menu/map/chat/switch).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VseWheelAction {
    /// Interior with a bus: nothing (F1 zoom is RMB+drag).
    Nothing,
    /// F2's legacy wheel zoom (`zoom_by`), preserved byte-identical.
    ZoomLegacy,
    /// Free/foot dolly along the view direction.
    Dolly,
}

/// Routing for the views the chase machine does not own (driver/pax/free-fly
/// and the player-less camera). `outside` is owned by `VseChase::on_zoom`,
/// orbit `free` by `VseFreeOrbit::on_zoom`.
pub fn vse_wheel_action(view: &str, _ctrl: bool, has_player: bool) -> VseWheelAction {
    if view == "pax" && has_player {
        VseWheelAction::ZoomLegacy
    } else if matches!(view, "driver" | "pax") && has_player {
        VseWheelAction::Nothing
    } else {
        VseWheelAction::Dolly
    }
}

/// F1 zoom intent: `0.70` minus the user-tested 20% (`0.56`).
/// F3/F4 keep `0.70`. Explicit per-mode value, not a guess.
pub const VSE_F1_ZOOM_INTENT: f32 = 0.56;

/// Precision-zoom step: the
/// multiplier `m` is the zoom state `z` seen through `m = 1/(1+5.5z)`,
/// so the same drag deltas take the same FOV path for the same
/// base FOV. `dy > 0` (drag down) zooms in. Result stays `<= 1.0`
/// (never into negative zoom); callers clamp the floor.
pub fn vse_precision_zoom_step(mult: f32, dy: f32, intent: f32) -> f32 {
    let z = ((1.0 / mult.max(0.154) - 1.0) / VSE_FOV_ZOOM_K).clamp(0.0, 1.0);
    let z2 = (z + dy * intent / VSE_ZOOM_PX).clamp(0.0, 1.0);
    1.0 / (1.0 + VSE_FOV_ZOOM_K * z2)
}

/// Zoom-dependent downward pitch margin (user-tested, geometrically
/// derived): at zoom the
/// visible window shrinks, so it may travel inside the normal frame's envelope
/// before hitting the physical stop. `margin = half(base) − half(base·m)` in
/// degrees, `>= 0` (zoom-out never grants extra). At `m = 1` this is `0` and
/// `-35°` stands exactly.
pub fn vse_zoom_pitch_margin(base_fov_deg: f32, mult: f32) -> f32 {
    let half0 = (base_fov_deg.to_radians() * 0.5).tan();
    let margin = half0.atan().to_degrees() - (half0 * mult).atan().to_degrees();
    margin.max(0.0)
}

/// Effective downward pitch limit for the F1 head: `-35°` minus the
/// zoom margin above.
pub fn vse_pitch_min_for_zoom(base_fov_deg: f32, mult: f32) -> f32 {
    VSE_LOOK_PITCH_MIN - vse_zoom_pitch_margin(base_fov_deg, mult)
}

/// Steering merge precedence (`Player::tick`): one slot, strict priority —
/// an engaged analog (mouse `O`, gamepad, touch) wins, else the keys. A tiny
/// analog against a held key still yields to the keys; a centered wheel
/// yields to the analog. Exactly one value leaves this function per frame.
pub fn vse_pick_steering(analog: Option<f32>, keys: f32) -> f32 {
    match analog {
        Some(s) if s.abs() > 0.02 || keys == 0.0 => s,
        _ => keys,
    }
}
/// bus-authored centre rotated by heading alone — never by the body's
/// pitch/bank, so suspension bounce and body roll never reach the camera.
/// `position.z` is averaged terrain height (smooth); body motion lives in the
/// attitude, which is deliberately excluded here.
pub fn vse_orbit_pivot(
    veh_pos: [f64; 3],
    heading_rad: f64,
    center: [f32; 3],
) -> [f64; 3] {
    let (sy, cy) = heading_rad.sin_cos();
    [
        veh_pos[0] + cy as f64 * center[0] as f64 + sy as f64 * center[1] as f64,
        veh_pos[1] - sy as f64 * center[0] as f64 + cy as f64 * center[1] as f64,
        veh_pos[2] + center[2] as f64,
    ]
}

/// Drive-head glide switch (menu "Drive Head Smooth Movement", default ON):
/// `true` = `0.54s` ease, `false` = instant cut, no interpolation.
pub fn vse_glide_active(smooth_setting: bool) -> bool {
    smooth_setting && VSE_CAM_EASE_SECS > 0.0
}

/// F4 press state machine: first press from another view snapshots the
/// chase pose (`snap = true`, orbit mode); further presses toggle fly mode.
/// Returns `(snapshot_now, fly_afterwards)`.
pub fn vse_free_press(in_free: bool, fly: bool) -> (bool, bool) {
    if !in_free {
        (true, false)
    } else {
        (false, !fly)
    }
}

/// F3 chase pose in the yaw frame (
/// x = right, y = forward, z = up): `off = [D·sinCY·cosCP, -D·cosCY·cosCP,
/// 1.6 + D·sinCP]`, camera = vehicle + yaw-basis·off at the pivot height,
/// target = vehicle + 1.4. Axis-preserving.
pub fn vse_chase_pose(
    veh: [f64; 3],
    pivot_z: f64,
    yaw_rad: f64,
    chase_yaw_deg: f32,
    chase_pitch_deg: f32,
    dist: f32,
) -> ([f64; 3], [f64; 3]) {
    let (cy, cp) = (
        chase_yaw_deg.to_radians() as f64,
        chase_pitch_deg.to_radians() as f64,
    );
    let off = [
        dist as f64 * cy.sin() * cp.cos(),
        -(dist as f64) * cy.cos() * cp.cos(),
        VSE_CHASE_OFF_Z as f64 + dist as f64 * cp.sin(),
    ];
    let (sy, cyaw) = yaw_rad.sin_cos();
    let cam = [
        veh[0] + cyaw * off[0] + sy * off[1],
        veh[1] - sy * off[0] + cyaw * off[1],
        pivot_z + off[2],
    ];
    let tgt = [veh[0], veh[1], pivot_z + VSE_CHASE_TGT_Z as f64];
    (cam, tgt)
}

/// Eased Space return (same `0.54s` ease-out as the head glide): look and zoom
/// multiplier ease
/// from their current values to `(0,0)`/`1.0` — never a teleport.
/// Returns `(look, zoom_or_remove, done)`.
pub fn vse_reset_blend(
    look_from: (f32, f32),
    zoom_from: f32,
    t: f32,
) -> ((f32, f32), Option<f32>, bool) {
    let s = vse_ease_out(t / VSE_CAM_EASE_SECS);
    let look = (look_from.0 * (1.0 - s), look_from.1 * (1.0 - s));
    if s >= 1.0 {
        (look, None, true)
    } else {
        (look, Some(zoom_from + (1.0 - zoom_from) * s), false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ease_is_cubic_out_over_054() {
        assert!((VSE_CAM_EASE_SECS - 0.54).abs() < 1e-6);
        assert_eq!(vse_ease_out(0.0), 0.0);
        assert_eq!(vse_ease_out(1.0), 1.0);
        assert!((vse_ease_out(0.5) - 0.875).abs() < 1e-6);
    }

    #[test]
    fn wrap_loops_both_ends() {
        assert_eq!(vse_wrap_index(0, 3, -1), 2);
        assert_eq!(vse_wrap_index(2, 3, 1), 0);
        assert_eq!(vse_wrap_index(1, 1, 1), 0);
    }

    #[test]
    fn shortest_yaw_takes_the_short_way() {
        assert!((vse_shortest_yaw(170.0, -170.0) - 20.0).abs() < 1e-4);
        assert!((vse_shortest_yaw(-170.0, 170.0) + 20.0).abs() < 1e-4);
    }

    #[test]
    fn interior_fov_is_65x() {
        assert!((vse_interior_fov(60.0, 0.0) - 60.0).abs() < 1e-4);
        assert!((vse_interior_fov(60.0, 1.0) - 60.0 / 6.5).abs() < 1e-3);
        assert!(vse_interior_fov(60.0, 1.0) >= VSE_FOV_MIN);
    }

    #[test]
    fn interior_orbit_clamps_and_zoom_stabilises() {
        let (y, _) = vse_interior_orbit(0.0, 0.0, 1000.0, 0.0, 0.0);
        assert_eq!(y, 90.0);
        let (y0, _) = vse_interior_orbit(0.0, 0.0, 10.0, 0.0, 0.0);
        let (y1, _) = vse_interior_orbit(0.0, 0.0, 10.0, 0.0, 1.0);
        assert!(y1 < y0);
        let (_, p) = vse_interior_orbit(0.0, 0.0, 0.0, -1000.0, 0.0);
        assert_eq!(p, 35.0);
        assert!((vse_interior_zoom(0.0, 364.0) - 0.70).abs() < 1e-4);
    }

    #[test]
    fn chase_consts_match_vse() {
        assert!((vse_chase_dist0(6.08) - (6.08 * 1.5 + 4.0)).abs() < 1e-4);
        let (y, p) = vse_chase_orbit(0.0, 12.0, 10.0, 10.0);
        assert!((y + 3.5).abs() < 1e-4 && (p - 15.5).abs() < 1e-4);
        assert_eq!(vse_chase_zoom(10.0, 1.0), 8.5);
        assert_eq!(vse_chase_zoom(4.0, 10.0), 4.0);
        let off = vse_chase_offset(10.0, 0.0, 0.0);
        assert!((off[1] + 10.0).abs() < 1e-4 && (off[2] - 1.6).abs() < 1e-4);
        assert!((vse_f4_blend(0.1) - (1.0 - (-1.0f32).exp())).abs() < 1e-4);
    }

    #[test]
    fn mouse_target_edge_corner_and_pedals() {
        assert!((vse_mouse_target(1.0, 0.0) - 0.75).abs() < 1e-4);
        assert!((vse_mouse_target(1.0, 1.0) - 1.0).abs() < 1e-4);
        assert!((vse_mouse_target(-0.5, 0.0) + 0.375).abs() < 1e-4);
        assert_eq!(vse_mouse_target(0.0, 0.0), 0.0);
        assert_eq!(vse_mouse_pedals(0.05), (0.0, 0.0));
        assert!((vse_mouse_pedals(1.0).0 - 1.0).abs() < 1e-4);
        assert!((vse_mouse_pedals(-1.0).1 - 1.0).abs() < 1e-4);
        let s = vse_mouse_slew(0.0, 1.0, 0.01);
        assert!((s - 0.025).abs() < 1e-4);
    }

    /// No steering deadzone in mouse drive (deadzone is pedals-only
    /// `0.10`; the steer map is proportional from zero). The target is
    /// strictly increasing in |nx| and zero only at the centre.
    #[test]
    fn mouse_target_has_no_center_deadzone() {
        assert_eq!(vse_mouse_target(0.0, 0.0), 0.0);
        let mut last = 0.0f32;
        let mut x = 0.01f32;
        while x <= 1.0 {
            let m = vse_mouse_target(x, 0.0);
            assert!(m > last, "flat spot at nx={x}: {m} <= {last}");
            last = m;
            x += 0.01;
        }
        // symmetric to the left, corner adds (never a step at the ramp start).
        assert!(vse_mouse_target(-0.3, 0.0) < 0.0);
        assert!(vse_mouse_target(0.21, 0.5) > vse_mouse_target(0.20, 0.0));
    }

    /// State separation: a stale past-edge accumulation never survives the
    /// ratchet — near the centre the HOLD always clears, while the small
    /// target itself passes through intact (no steering deadzone: the old
    /// `(0,0)` return here was the O centre-deadzone bug).
    #[test]
    fn stale_edge_never_latches_the_ratchet() {
        let approx = |got: (f32, f32), want: (f32, f32)| {
            assert!((got.0 - want.0).abs() < 1e-6 && (got.1 - want.1).abs() < 1e-6, "{got:?} != {want:?}");
        };
        approx(vse_boost_hold(1.5, 0.1, 0.075), (0.0, 0.075));
        approx(vse_boost_hold(-1.5, -0.1, -0.075), (0.0, -0.075));
        assert_eq!(vse_boost_hold(1.5, 0.0, 0.0), (0.0, 0.0));
        // small lateral target passes through: `lateral + 0`.
        approx(vse_boost_hold(0.0, 0.3, vse_mouse_target(0.3, 0.0)), (0.0, 0.225));
        // side change: hold cleared, new-side target recomputed, no zero frame.
        approx(vse_boost_hold(0.25, -0.6, -0.45), (0.0, -0.45));
    }

    /// Ratchet persistence (`target = lateral + |hold|` always): after a
    /// corner visit the latched boost holds the wheel even when the raw
    /// target falls back to the edge — it never unwinds on its own.
    #[test]
    fn corner_boost_persists_until_centre_or_side_change() {
        // corner visit latches the excess over the edge.
        let (h, t) = vse_boost_hold(0.0, 1.0, 1.0);
        assert!((h - 0.25).abs() < 1e-6 && (t - 1.0).abs() < 1e-6, "{h} {t}");
        // back to mid-lateral: raw 0.45, latched 0.25 stays on top.
        let (h, t) = vse_boost_hold(0.25, 0.6, 0.45);
        assert!((h - 0.25).abs() < 1e-6 && (t - 0.70).abs() < 1e-6, "{h} {t}");
    }

    /// Stick arbitration: only a deflected stick drives the chase — a
    /// centred pad (dead `0.0`) leaves the wheel to the keys, so an idle
    /// gamepad can never fight A/D through the merge gate.
    #[test]
    fn idle_stick_never_drives_the_chase() {
        assert!(!vse_stick_chase(true, None));
        assert!(!vse_stick_chase(true, Some(0.0)));
        assert!(!vse_stick_chase(false, Some(0.5)));
        assert!(vse_stick_chase(true, Some(0.5)));
        assert!(vse_stick_chase(true, Some(-0.09)));
    }

    /// Anchor gate: neutral exactly while the cursor sits on the enable
    /// point (float equality is intended — same stored cursor value).
    #[test]
    fn anchor_holds_until_first_move() {
        assert!(vse_anchor_neutral(Some((800.0, 450.0)), (800.0, 450.0)));
        assert!(!vse_anchor_neutral(Some((800.0, 450.0)), (801.0, 450.0)));
        assert!(!vse_anchor_neutral(None, (800.0, 450.0)));
    }

    #[test]
    fn keyboard_rates_fall_with_speed() {
        let (slow, _, _) = vse_keyboard_rates(0.0, 0.0, 0.70);
        let (mid, _, _) = vse_keyboard_rates(40.0, 0.0, 0.70);
        let (fast, _, _) = vse_keyboard_rates(80.0, 0.0, 0.70);
        assert!(slow > mid && mid > fast);
        assert!((vse_hermite_pitch(0.0) - 1.0).abs() < 1e-4);
        assert!((vse_hermite_pitch(1.0) - 3.25).abs() < 1e-3);
    }

    #[test]
    fn return_only_rolls_and_settles() {
        assert_eq!(vse_return_step(0.5, 0.0, 850.0, 1.0, 0.01), 0.5);
        let back = vse_return_step(0.5, 30.0, 850.0, 1.0, 0.01);
        assert!(back < 0.5 && back > 0.49);
        assert_eq!(vse_return_step(1e-5, 30.0, 850.0, 1.0, 0.01), 1e-5);
    }

    #[test]
    fn ffb_and_ackermann_match_vse() {
        assert!((vse_ffb_nm(0.2, 22.22) + 0.025 * 0.2 * 22.22 * 22.22).abs() < 1e-3);
        assert_eq!(vse_ffb_nm(10.0, 100.0), -15.0);
        let (inner, outer) = vse_ackermann(0.5, 0.8726, 6.0, 2.0);
        assert!(inner > outer && outer > 0.0);
        assert!((vse_curve_authority("Rigid6x2") - 0.70).abs() < 1e-6);
        assert!((vse_directional(0.5, 0.0, -0.3, 0.8) + 0.15).abs() < 1e-4);
        assert_eq!(vse_directional(0.5, 50.0, -0.3, 0.8), 0.0);
    }

    /// Behavioral routing matrix: MMB+drag orbits every mode; RMB+drag is
    /// precision zoom in driver/outside/free, legacy zoom in pax
    /// (frozen F2); with no button only mouse-drive sees motion, never with
    /// the menu open.
    #[test]
    fn drag_routing_single_consumer_per_context() {
        use VseDragAction as A;
        use VseDragButton as B;
        for view in ["driver", "pax", "outside", "free", "foot"] {
            assert_eq!(
                vse_drag_action(Some(B::Middle), view, false, false),
                A::Look,
                "{view}"
            );
        }
        assert_eq!(
            vse_drag_action(Some(B::Right), "driver", false, false),
            A::ZoomPrecision
        );
        assert_eq!(
            vse_drag_action(Some(B::Right), "pax", false, false),
            A::ZoomLegacy
        );
        for view in ["outside", "free"] {
            assert_eq!(
                vse_drag_action(Some(B::Right), view, false, false),
                A::ZoomPrecision,
                "{view}"
            );
        }
        assert_eq!(
            vse_drag_action(Some(B::Right), "foot", false, false),
            A::Look
        );
        assert_eq!(vse_drag_action(None, "driver", true, false), A::MouseEdge);
        assert_eq!(vse_drag_action(None, "driver", true, true), A::Nothing);
        assert_eq!(vse_drag_action(None, "driver", false, false), A::Nothing);
        assert_eq!(vse_drag_action(None, "free", true, false), A::MouseEdge);
    }

    /// Behavioral routing matrix: the wheel serves F2's frozen path, is a
    /// no-op in F1 with a bus, and dollies free-fly/foot/player-less views.
    /// `outside` and orbit `free` never reach the table (machines own
    /// their distance).
    #[test]
    fn wheel_routing_only_f3_zooms() {
        use VseWheelAction as W;
        assert_eq!(vse_wheel_action("outside", false, true), W::Dolly);
        assert_eq!(vse_wheel_action("driver", false, true), W::Nothing);
        assert_eq!(vse_wheel_action("driver", true, true), W::Nothing);
        assert_eq!(vse_wheel_action("pax", false, true), W::ZoomLegacy);
        for view in ["driver", "pax"] {
            assert_eq!(vse_wheel_action(view, false, false), W::Dolly, "{view}");
        }
        assert_eq!(vse_wheel_action("free", false, true), W::Dolly);
        assert_eq!(vse_wheel_action("foot", false, false), W::Dolly);
    }

    /// Precision zoom follows the state curve: a full 364 px drag takes
    /// `z` 0→0.70 (`m` 1.0→0.206), drag down zooms in, drag up undoes it, and
    /// the multiplier never exceeds 1.0 (never negative zoom). F1 runs the
    /// same curve 20% slower (`VSE_F1_ZOOM_INTENT`).
    #[test]
    fn precision_zoom_matches_vse_curve() {
        let full = |m: f32, dy: f32| vse_precision_zoom_step(m, dy, VSE_ZOOM_INTENT);
        assert!((full(1.0, 364.0) - 1.0 / (1.0 + 5.5 * 0.70)).abs() < 1e-4);
        let m = full(1.0, 100.0);
        assert!(m < 1.0 && m > 0.45, "{m}");
        assert!((full(m, -100.0) - 1.0).abs() < 1e-4);
        assert_eq!(full(1.0, -50.0), 1.0);
        assert!(full(1.0, 10000.0) >= 0.153);
        // F1: same curve, 20% less gain per pixel.
        let slow = vse_precision_zoom_step(1.0, 100.0, VSE_F1_ZOOM_INTENT);
        assert!(slow > m && slow < 1.0, "{slow} vs {m}");
        assert!((VSE_F1_ZOOM_INTENT - VSE_ZOOM_INTENT * 0.80).abs() < 1e-6);
    }

    /// Zoom-dependent pitch margin: zero at rest (`-35°` stands), growing
    /// with zoom — normal, intermediate, maximum.
    #[test]
    fn zoom_pitch_margin_grows_with_zoom() {
        assert_eq!(vse_zoom_pitch_margin(60.0, 1.0), 0.0);
        assert_eq!(vse_pitch_min_for_zoom(60.0, 1.0), -35.0);
        let mid = vse_zoom_pitch_margin(60.0, 0.5);
        assert!(mid > 5.0 && mid < 15.0, "{mid}");
        let max = vse_zoom_pitch_margin(60.0, 1.0 / 6.5);
        assert!(max > 20.0 && max < 26.0, "{max}");
        assert!(vse_pitch_min_for_zoom(60.0, 1.0 / 6.5) < -55.0);
        // zoom-out never grants extra downward room.
        assert_eq!(vse_zoom_pitch_margin(60.0, 1.6), 0.0);
    }

    /// Stable pivot: heading-only rotation — body pitch/bank cannot move it
    /// (the function takes none), height rides the smooth position plus the
    /// authored centre.
    #[test]
    fn orbit_pivot_ignores_body_attitude() {
        let p = vse_orbit_pivot([10.0, 20.0, 5.0], 0.0, [0.0, 0.0, 1.2]);
        assert!((p[0] - 10.0).abs() < 1e-9 && (p[1] - 20.0).abs() < 1e-9);
        assert!((p[2] - 6.2).abs() < 1e-6, "{p:?}");
        // heading 90°: the forward offset swings east, height untouched.
        let q = vse_orbit_pivot([0.0, 0.0, 3.0], std::f64::consts::FRAC_PI_2, [1.0, 2.0, 1.2]);
        assert!((q[0] - 2.0).abs() < 1e-6 && (q[1] + 1.0).abs() < 1e-6);
        assert!((q[2] - 4.2).abs() < 1e-6, "{q:?}");
    }

    /// Glide switch: setting ON (the default) eases, OFF cuts instantly.
    #[test]
    fn glide_switch_matches_menu_default() {
        assert!(vse_glide_active(true));
        assert!(!vse_glide_active(false));
        assert!(crate::settings::Settings::default().driverview_smooth);
    }

    /// F4 press machine: first press snapshots (VSE mode), further presses
    /// toggle fly — pose untouched by the toggle itself.
    #[test]
    fn free_press_cycles_vse_then_fly() {
        assert_eq!(vse_free_press(false, false), (true, false));
        assert_eq!(vse_free_press(false, true), (true, false));
        assert_eq!(vse_free_press(true, false), (false, true));
        assert_eq!(vse_free_press(true, true), (false, false));
    }

    /// Merge precedence: exactly one steering value per frame — analog wins
    /// when engaged, keys otherwise. No summing, no `max()` across writers.
    #[test]
    fn steering_merge_has_one_winner() {
        assert_eq!(vse_pick_steering(Some(0.5), -0.3), 0.5);
        assert_eq!(vse_pick_steering(None, -0.3), -0.3);
        assert_eq!(vse_pick_steering(Some(0.01), -0.3), -0.3);
        assert_eq!(vse_pick_steering(Some(0.01), 0.0), 0.01);
        assert_eq!(vse_pick_steering(Some(-0.5), 0.0), -0.5);
    }

    /// §7 numeric proof (hand-derived):
    /// bus at (100, 200, 50), ride 0.3 → pivot 49.7, heading 0, chase yaw 0,
    /// pitch 12°, dist = 6.08·1.5+4 = 13.12.
    /// off = [0, -13.12·cos12°, 1.6+13.12·sin12°] = [0, -12.833, 4.328];
    /// cam = (100, 187.167, 54.028), tgt = (100, 200, 51.1);
    /// |cam−tgt| = √(12.833² + 2.928²) = 13.163; cam looks down ~12.8°.
    #[test]
    fn chase_pose_matches_vse_geometry() {
        let (cam, tgt) = vse_chase_pose([100.0, 200.0, 50.0], 49.7, 0.0, 0.0, 12.0, 13.12);
        assert!((cam[0] - 100.0).abs() < 1e-6);
        assert!((cam[1] - 187.167).abs() < 1e-2, "{}", cam[1]);
        assert!((cam[2] - 54.028).abs() < 1e-2, "{}", cam[2]);
        for (a, b) in tgt.iter().zip([100.0, 200.0, 51.1]) {
            assert!((a - b).abs() < 1e-6, "{tgt:?}");
        }
        let d = ((cam[0] - tgt[0]).powi(2) + (cam[1] - tgt[1]).powi(2) + (cam[2] - tgt[2]).powi(2))
            .sqrt();
        assert!((d - 13.163).abs() < 1e-2, "{d}");
        assert!(
            cam[2] > tgt[2],
            "camera above the target, never under the bus"
        );
        // yaw 90°: the camera sits to the side, same height relation.
        let (cam2, tgt2) = vse_chase_pose([0.0, 0.0, 10.0], 9.7, 0.0, 90.0, 12.0, 13.12);
        assert!((cam2[0] - 12.833).abs() < 1e-2 && (cam2[1] - 0.0).abs() < 1e-6);
        for (a, b) in tgt2.iter().zip([0.0, 0.0, 11.1]) {
            assert!((a - b).abs() < 1e-6, "{tgt2:?}");
        }
    }

    /// Space return eases with the head-glide curve (`0.54s` ease-out):
    /// halfway is ~87.5% home, the end lands exactly on normal, never past it.
    #[test]
    fn reset_blend_eases_like_the_head_glide() {
        let (look, zoom, done) = vse_reset_blend((30.0, -10.0), 0.5, 0.0);
        assert_eq!((look, zoom, done), ((30.0, -10.0), Some(0.5), false));
        let (look, zoom, done) = vse_reset_blend((30.0, -10.0), 0.5, 0.27);
        assert!((look.0 - 30.0 * 0.125).abs() < 1e-4, "{look:?}");
        assert!(zoom.is_some_and(|z| z > 0.5 && z < 1.0));
        assert!(!done);
        let (look, zoom, done) = vse_reset_blend((30.0, -10.0), 0.5, 0.54);
        assert_eq!(look, (0.0, 0.0));
        assert_eq!((zoom, done), (None, true));
    }
}
