//! Input actions: the names from `Inputs/keyboard.cfg` and how they reach a vehicle.
//!
//! Vehicle actions are script triggers with the same name; on key release the trigger
//! `<name>_off` fires. A few actions are handled by the engine itself (throttle, brake,
//! clutch, steering, views).

/// Actions the engine handles instead of forwarding to the script.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EngineAction {
    Throttle,
    ThrottleAmplify,
    Brake,
    Clutch,
    SteeringLeft,
    SteeringRight,
    SteeringNeutral,
}

pub fn engine_action(name: &str) -> Option<EngineAction> {
    Some(match name.to_ascii_lowercase().as_str() {
        "throttle" => EngineAction::Throttle,
        "throttle_amplify" => EngineAction::ThrottleAmplify,
        "brake" => EngineAction::Brake,
        "clutch" => EngineAction::Clutch,
        "steering_left" => EngineAction::SteeringLeft,
        "steering_right" => EngineAction::SteeringRight,
        "steering_neutral" => EngineAction::SteeringNeutral,
        _ => return None,
    })
}

/// Keyboard-driven analogue inputs, integrated per frame like the original (keys ramp the
/// pedal/steering position instead of setting it).
#[derive(Debug, Clone)]
pub struct KeyboardAxes {
    pub throttle_key: bool,
    pub amplify_key: bool,
    pub brake_key: bool,
    pub clutch_key: bool,
    pub left_key: bool,
    pub right_key: bool,
    pub neutral_key: bool,
    pub throttle: f32,
    pub brake: f32,
    pub clutch: f32,
    pub steering: f32,
    /// Road speed in km/h: the steering returns to centre by itself only while rolling.
    pub speed_kmh: f32,
    /// Rate of the steering wheel (fraction per second) while it swings back on its own.
    pub steer_vel: f32,
    /// Keyboard sensitivity slider: default `0.70`, clamp `0.20..2.00`.
    /// Scales both turn and return.
    pub steering_sens: f32,
    /// "Steering linearity": the keys turn the wheel as Omsi.exe does (0x7e64c6): the
    /// curvature changes by 0.00005 per millisecond whatever the speed, so the wheel goes at
    /// one steady pace; `lock_curvature` (`[inv_min_turnradius]`) turns that into a share
    /// of the lock.
    pub linear: bool,
    /// "Old Steering": let go, the wheel stays where it is and is turned back by hand - OMSI
    /// without `[autoCenter]`.
    pub old_steering: bool,
    /// While the mouse owns the wheel (`O` drive), the keyboard
    /// contributes nothing to steering or pedals.
    /// Key states are kept, so releasing `O` resumes seamlessly.
    pub mouse_owned: bool,
    /// Handed back from the mouse: stays until steered by hand
    /// (`hold_steer`; any steering key or centering clears it).
    pub hold_steer: bool,
    /// Pedal plant state below (press/release edges and hold timers).
    pub prev_thr_held: bool,
    pub thr_since_release_s: f32,
    pub thr_fast_rise: bool,
    pub prev_brk_held: bool,
    pub brk_press_s: f32,
    pub brk_hold_mode: bool,
    pub brk_raw_at_press: f32,
    pub prev_clu_held: bool,
    /// The gearbox has a real clutch pedal (only for manual gearboxes;
    /// default true here because no drivetrain type reaches the input layer
    /// — without it the TAB clutch would die for every bus).
    pub clutch_has_pedal: bool,
    pub lock_curvature: f32,
    /// OMSI's held brake (the default): let go, the brake stays where the key left it until
    /// the throttle key is pressed - tap the brake and it keeps that pressure. Off, the brake
    /// comes off with its key, as in most games. (The throttle never stays: see `update`.)
    pub pedal_hold: bool,
    /// The steering is on its way back to the middle (Omsi.exe +0x5d6): set by the
    /// `steering_neutral` key, cleared by a steering key.
    pub centering: bool,
}

impl Default for KeyboardAxes {
    fn default() -> Self {
        KeyboardAxes {
            throttle_key: false,
            amplify_key: false,
            brake_key: false,
            clutch_key: false,
            left_key: false,
            right_key: false,
            neutral_key: false,
            throttle: 0.0,
            brake: 0.0,
            clutch: 0.0,
            steering: 0.0,
            speed_kmh: 0.0,
            steer_vel: 0.0,
            steering_sens: 0.0,
            mouse_owned: false,
            hold_steer: false,
            prev_thr_held: false,
            thr_since_release_s: 10.0,
            thr_fast_rise: false,
            prev_brk_held: false,
            brk_press_s: 0.0,
            brk_hold_mode: false,
            brk_raw_at_press: 0.0,
            prev_clu_held: false,
            clutch_has_pedal: true,
            linear: false,
            old_steering: false,
            lock_curvature: 0.0,
            pedal_hold: false,
            centering: false,
        }
    }
}

impl KeyboardAxes {
    pub fn set(&mut self, action: EngineAction, pressed: bool) {
        match action {
            EngineAction::Throttle => self.throttle_key = pressed,
            EngineAction::ThrottleAmplify => self.amplify_key = pressed,
            EngineAction::Brake => self.brake_key = pressed,
            EngineAction::Clutch => self.clutch_key = pressed,
            EngineAction::SteeringLeft => self.left_key = pressed,
            EngineAction::SteeringRight => self.right_key = pressed,
            EngineAction::SteeringNeutral => self.neutral_key = pressed,
        }
    }

    /// Let go of every key: the window losing focus (alt-tab, a click outside it, an OS
    /// dialog) never delivers the matching key-up, so without this a throttle or steering
    /// key held at that moment stayed "pressed" forever (and, with a modifier key stuck the
    /// same way, a later plain key press could be misread as held with that modifier).
    pub fn release_all(&mut self) {
        *self = KeyboardAxes {
            throttle: self.throttle,
            brake: self.brake,
            clutch: self.clutch,
            steering: self.steering,
            speed_kmh: self.speed_kmh,
            linear: self.linear,
            old_steering: self.old_steering,
            lock_curvature: self.lock_curvature,
            pedal_hold: self.pedal_hold,
            centering: self.centering,
            steering_sens: if self.steering_sens > 0.0 { self.steering_sens } else { 0.70 },
            mouse_owned: self.mouse_owned,
            hold_steer: self.hold_steer,
            ..Default::default()
        };
    }

    /// Keyboard step: `input ±= rate*dt` with speed turn/unwind, low-speed
    /// agility and Hermite unwind pitch. Returns `(steer_rate, unsteer_rate,
    /// pitch_mult)`. `v_kmh` absolute road speed, `dt` seconds, integrated
    /// in fixed `0.01 s` substeps.
    pub fn vse_keyboard_step(&mut self, dt: f32, v_kmh: f32) -> (f32, f32, f32) {
        let sens = if self.steering_sens > 0.0 { self.steering_sens } else { 0.70 };
        let (steer, unsteer, pitch) = vse_keyboard_rates_inner(v_kmh, self.steering.abs(), sens);
        let n = ((dt / 0.01).round() as usize).clamp(1, 5);
        let h = dt / n as f32;
        for _ in 0..n {
            let (s, u, _) = vse_keyboard_rates_inner(v_kmh, self.steering.abs(), sens);
            if self.left_key && !self.right_key {
                let unwinding = self.steering > 0.005;
                let rate = if unwinding { u } else { s };
                self.steering = (self.steering - rate * h).max(-1.0);
                self.steer_vel = 0.0;
                self.centering = false;
            } else if self.right_key && !self.left_key {
                let unwinding = self.steering < -0.005;
                let rate = if unwinding { u } else { s };
                self.steering = (self.steering + rate * h).min(1.0);
                self.steer_vel = 0.0;
                self.centering = false;
            }
        }
        self.steering = self.steering.clamp(-1.0, 1.0);
        (steer, unsteer, pitch)
    }

    /// Return-to-centre: only while rolling (`v_kmh > 0`), with angular
    /// ease-out, in fixed `0.01 s` substeps. No-op while a steering key is
    /// held. Returns the applied step (signed).
    pub fn vse_return_to_centre(&mut self, dt: f32, v_kmh: f32, half_deg: f32, pitch_mult: f32) -> f32 {
        if self.left_key || self.right_key || self.steering.abs() <= 0.0001 || dt <= 0.0 {
            return 0.0;
        }
        let before = self.steering;
        let n = ((dt / 0.01).round() as usize).clamp(1, 5);
        let h = dt / n as f32;
        for _ in 0..n {
            self.steering = vse_return_step_inner(self.steering, v_kmh, half_deg, pitch_mult, h);
        }
        self.steering - before
    }

    pub fn update(&mut self, dt: f32) {
        // While the mouse owns the wheel the keyboard is ignored
        // entirely; the stored key states are kept for resume.
        let (throttle_key, brake_key, left_key, right_key) = if self.mouse_owned {
            (false, false, false, false)
        } else {
            (self.throttle_key, self.brake_key, self.left_key, self.right_key)
        };
        // Pedal plant: keyboard held / analog bypass feed temporal ramps,
        // never direct positions. `dt` clamped (no hitch jumps).
        let dt = dt.clamp(0.0, 0.05);
        let v_kmh = self.speed_kmh.abs();
        // --- throttle (tip-in, double-tap kickdown, rise/fall) ---
        // `amplify` (OMSI key) lifts the cap to 1.0; the curve is unchanged.
        let thr_cap = if self.amplify_key { 1.0 } else { 0.80 };
        if throttle_key {
            self.brake = 0.0;
            if !self.prev_thr_held {
                self.thr_fast_rise = self.thr_since_release_s <= 0.40;
                if !self.thr_fast_rise && self.throttle < 0.16 {
                    self.throttle = 0.16;
                }
            }
            let (rise_s, cap): (f32, f32) = if self.thr_fast_rise {
                (0.16, 1.0)
            } else {
                (0.80, thr_cap)
            };
            self.throttle = cap.min(self.throttle + (cap / rise_s) * dt);
        } else {
            if self.prev_thr_held {
                self.thr_since_release_s = 0.0;
                self.thr_fast_rise = false;
            } else {
                self.thr_since_release_s += dt;
            }
            self.throttle = (self.throttle - dt / 0.35).max(0.0);
        }
        self.prev_thr_held = throttle_key;
        // --- brake (tap steps latch, hold tangent, W cancel) ---
        // `pedal_hold == false` keeps the legacy fast release instead.
        if self.pedal_hold {
            if brake_key && !self.prev_brk_held {
                self.brk_press_s = 0.0;
                self.brk_hold_mode = false;
                self.brk_raw_at_press = self.brake;
            }
            if brake_key {
                self.brk_press_s += dt;
                if !self.brk_hold_mode && self.brk_press_s >= 0.22 {
                    self.brk_hold_mode = true;
                }
                if self.brk_hold_mode && self.brake < 1.0 - 1e-4 {
                    let t = ((v_kmh - 20.0) / 70.0).clamp(0.0, 1.0);
                    let tau = 0.28 + (0.85 - 0.28) * t;
                    let alpha = 1.0 - (-dt / tau).exp();
                    self.brake = (self.brake + (1.0 - self.brake) * alpha).min(1.0);
                }
            } else if self.prev_brk_held {
                if !self.brk_hold_mode && self.brk_press_s < 0.22 {
                    self.brake = next_brake_step(self.brk_raw_at_press);
                }
                self.brk_press_s = 0.0;
                self.brk_hold_mode = false;
            }
            if !brake_key
                && (throttle_key)
                && self.brake > 1e-4
            {
                self.brake = (self.brake - dt / 0.125).max(0.0);
            }
            self.prev_brk_held = brake_key;
        } else if brake_key {
            self.throttle = 0.0;
            self.brake = (self.brake + dt).min(1.0);
        } else {
            self.brake = (self.brake - 3.0 * dt).max(0.0);
        }
        // --- clutch (TAB, bite floor, slow rise / fast fall) ---
        // Authority note: the drivetrain type decides this upstream; no
        // gearbox type reaches this layer here, so `clutch_has_pedal`
        // (default true) preserves the working clutch instead of killing it
        // for every bus.
        if !self.clutch_has_pedal {
            self.clutch = 0.0;
            self.prev_clu_held = false;
        } else if self.clutch_key {
            if !self.prev_clu_held && self.clutch < 0.20 {
                self.clutch = 0.20;
            }
            self.clutch = (self.clutch + dt / 0.45).min(1.0);
            self.prev_clu_held = true;
        } else {
            self.clutch = (self.clutch - dt / 0.18).max(0.0);
            self.prev_clu_held = false;
        }
        // Steering (single writer): A/D integrate with the rates
        // (base/exp/low/Hermite) and return only while rolling. The
        // `linear` / `centering` / `old_steering` opt-ins keep their OMSI
        // paths; the default path is the rate/return integrator below, so no
        // second integrator fights it.
        // While the mouse owns the wheel none of this runs.
        let v = self.speed_kmh.abs();
        let sens = if self.steering_sens > 0.0 { self.steering_sens } else { 0.70 };
        if !self.mouse_owned {
        if self.linear {
            // OMSI: 0.05 of curvature a second, from the middle to the lock in
            // `[inv_min_turnradius]` / 0.05 seconds (2 s for a bus with a 10 m radius); it
            // comes back (unless Old Steering) at the same pace, as `[autoCenter]` does
            let r = (0.05 / self.lock_curvature.max(0.01)).clamp(0.05, 5.0);
            if self.neutral_key {
                self.centering = true;
            }
            if left_key || right_key {
                self.centering = false;
            }
            if left_key {
                self.steering = (self.steering - r * dt).max(-1.0);
                self.steer_vel = 0.0;
            } else if right_key {
                self.steering = (self.steering + r * dt).min(1.0);
                self.steer_vel = 0.0;
            } else if self.centering {
                let step = r * dt;
                self.steering -= self.steering.clamp(-step, step);
                self.steer_vel = 0.0;
            } else if self.old_steering {
                self.steer_vel = 0.0;
            } else {
                let ease = (self.steering.abs() / 0.08).clamp(0.3, 1.0);
                let step = r * ease * dt;
                self.steering -= self.steering.clamp(-step, step);
                self.steer_vel = 0.0;
                if self.steering.abs() < 0.0003 {
                    self.steering = 0.0;
                }
            }
            return;
        }
        if self.neutral_key {
            self.centering = true;
        }
        // Fixed 0.01 s substeps, up to 5 a frame: frame-rate independent.
        let n = ((dt / 0.01).round() as usize).clamp(1, 5);
        let h = dt / n as f32;
        // (inside the outer `!mouse_owned` guard: no integration, no return,
        // no centering while the mouse drives)
        if left_key || right_key {
            self.centering = false;
        }
        if left_key && !right_key {
            self.hold_steer = false;
            for _ in 0..n {
                // Rates for this speed and wheel position (the unwind
                // pitch is also the return's angle factor input).
                let (steer_rate_v, unsteer_rate_v, _) =
                    vse_keyboard_rates_inner(v, self.steering.abs(), sens);
                let unwinding = self.steering > 0.005;
                let rate = if unwinding { unsteer_rate_v } else { steer_rate_v };
                self.steering = (self.steering - rate * h).max(-1.0);
                self.steer_vel = 0.0;
            }
        } else if right_key && !left_key {
            self.hold_steer = false;
            for _ in 0..n {
                let (steer_rate_v, unsteer_rate_v, _) =
                    vse_keyboard_rates_inner(v, self.steering.abs(), sens);
                let unwinding = self.steering < -0.005;
                let rate = if unwinding { unsteer_rate_v } else { steer_rate_v };
                self.steering = (self.steering + rate * h).min(1.0);
                self.steer_vel = 0.0;
            }
        } else if self.centering {
            // steering_neutral as in Omsi.exe (sub_7d5124 at 0x7d55d6): the wheel goes back
            // to the middle at the pace the keys turn it in OMSI - 0.05 of curvature a second -
            // in a straight line, and stays there until a steering key is pressed; it used to
            // jump to the middle, a jerk of the whole bus at speed
            self.hold_steer = false;
            let r = (0.05 / self.lock_curvature.max(0.01)).clamp(0.05, 5.0);
            let step = r * dt;
            self.steering -= self.steering.clamp(-step, step);
            self.steer_vel = 0.0;
        } else if self.old_steering {
            // Old Steering: the wheel stays where the hands left it
            self.steer_vel = 0.0;
        } else if self.hold_steer {
            // handed back from the mouse: stays until steered by hand
            // (`hold_steer`; any steering key above clears it).
            self.steer_vel = 0.0;
        } else {
            // Return-to-centre: only while rolling; standing still the
            // wheel rests where the hands left it. Pitch recomputed every
            // substep (it falls with the wheel).
            for _ in 0..n {
                let (_, _, pitch) =
                    vse_keyboard_rates_inner(v, self.steering.abs(), sens);
                self.steering = vse_return_step_inner(self.steering, v, 850.0, pitch, h);
            }
            self.steer_vel = 0.0;
        }
        }
    }
}

/// Shared steering math (mirrors `omsi-app::vse`; duplicated here so `omsi-sim`
/// stays platform-independent and desktop/mobile-free).
fn vse_smooth01(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Brake detent steps: first latch step above `current` (+2% epsilon).
fn next_brake_step(current: f32) -> f32 {
    const STEPS: [f32; 5] = [0.10, 0.25, 0.50, 0.80, 1.00];
    for s in STEPS {
        if current + 0.02 < s {
            return s;
        }
    }
    STEPS[4]
}

fn vse_hermite_pitch_inner(mag: f32) -> f32 {
    let h = |t: f32, p0: f32, p1: f32, m0: f32, m1: f32, hh: f32| {
        let (t2, t3) = (t * t, t * t * t);
        (2.0 * t3 - 3.0 * t2 + 1.0) * p0
            + (-2.0 * t3 + 3.0 * t2) * p1
            + (t3 - 2.0 * t2 + t) * (hh * m0)
            + (t3 - t2) * (hh * m1)
    };
    if mag <= 0.20 {
        h(mag / 0.20, 1.0, 1.35, 1.2, 2.0, 0.20)
    } else if mag <= 0.50 {
        h((mag - 0.20) / 0.30, 1.35, 2.15, 2.0, 2.2, 0.30)
    } else if mag <= 0.75 {
        h((mag - 0.50) / 0.25, 2.15, 2.75, 2.2, 2.0, 0.25)
    } else {
        2.75 + 2.0 * (mag - 0.75)
    }
}

fn vse_keyboard_rates_inner(v_kmh: f32, input_mag: f32, sens: f32) -> (f32, f32, f32) {
    let sens = sens.clamp(0.20, 2.00);
    let base = 0.336 * sens;
    let min = 0.145 * sens;
    let turn = 0.38 + 0.62 * (-0.045 * v_kmh).exp();
    let unwind_speed = 0.48 + 0.52 * (-0.032 * v_kmh).exp();
    let low = 2.0 - vse_smooth01((v_kmh - 20.0) / 25.0);
    let steer = (base * low * turn).clamp(min, base * 2.2);
    let unwind_base = (base * low * unwind_speed).clamp(min, base * 2.4);
    let pitch = vse_hermite_pitch_inner(input_mag.clamp(0.0, 1.0));
    let v_ramp = (v_kmh / 1.5).clamp(0.0, 1.0);
    let stopped = pitch.min(1.25);
    let mult = stopped + (pitch - stopped) * v_ramp;
    let blend = ((mult - 1.0) / 1.55).clamp(0.0, 1.0);
    let fast = (unwind_base * mult).max(base * mult);
    (steer, unwind_base * (1.0 - blend) + fast * blend, mult)
}

fn vse_return_step_inner(input: f32, v_kmh: f32, half_deg: f32, pitch_mult: f32, dt: f32) -> f32 {
    if input.abs() <= 0.0001 || dt <= 0.0 {
        return input;
    }
    let v_ramp = (v_kmh / 5.0).clamp(0.0, 1.0);
    let base = 0.22 * v_ramp * (-0.020 * v_kmh).exp();
    if base <= 1e-5 {
        return input;
    }
    let half = if half_deg > 100.0 { half_deg } else { 850.0 };
    let angle = input.abs() * half;
    let factor = if angle > 360.0 {
        1.0
    } else if angle > 45.0 {
        0.20 + 0.80 * vse_smooth01((angle - 45.0) / 315.0)
    } else if angle > 5.0 {
        0.05 + 0.15 * vse_smooth01((angle - 5.0) / 40.0)
    } else {
        (angle / 100.0).max(0.015)
    };
    let step = base * factor * (0.33 + 0.67 * pitch_mult) * dt;
    if input.abs() <= step || angle <= 0.05 {
        0.0
    } else if input > 0.0 {
        input - step
    } else {
        input + step
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear_and_old_steering_as_omsi() {
        let mut a = KeyboardAxes { linear: true, old_steering: true, lock_curvature: 0.1, speed_kmh: 50.0, ..Default::default() };
        a.right_key = true;
        for _ in 0..100 {
            a.update(0.01);
        }
        // 0.05 1/m a second against a lock of 0.1 1/m: half the lock in a second, at any speed
        assert!((a.steering - 0.5).abs() < 1e-3, "{}", a.steering);
        a.right_key = false;
        for _ in 0..100 {
            a.update(0.01);
        }
        assert!((a.steering - 0.5).abs() < 1e-3, "old steering: it stays");
        a.old_steering = false;
        for _ in 0..50 {
            a.update(0.01);
        }
        assert!((a.steering - 0.25).abs() < 0.02, "it comes back at the same pace: {}", a.steering);
    }

    /// Pedal plant: tip-in, double-tap kickdown, rise/fall ramps, tap
    /// steps with latch, hold tangent by speed, W-cancel.
    #[test]
    fn pedals_as_vse_ramps_them() {
        // throttle tip-in on the press edge, then 0.80 cap in 0.8 s.
        let mut a = KeyboardAxes::default();
        a.throttle_key = true;
        a.update(0.01);
        assert!((a.throttle - 0.17).abs() < 1e-6, "tip-in + first rise: {}", a.throttle);
        assert_eq!(a.brake, 0.0);
        for _ in 0..79 {
            a.update(0.01);
        }
        assert!((a.throttle - 0.80).abs() < 1e-3, "{}", a.throttle);
        // release: falls back at 1/0.35 a second.
        a.throttle_key = false;
        for _ in 0..10 {
            a.update(0.01);
        }
        assert!((a.throttle - 0.514).abs() < 0.02, "{}", a.throttle);
        // double-tap inside 0.40 s: kickdown to 1.0 in 0.16 s.
        for _ in 0..10 {
            a.update(0.01);
        }
        a.throttle_key = true;
        for _ in 0..20 {
            a.update(0.01);
        }
        assert_eq!(a.throttle, 1.0);
        a.throttle_key = false;
        // brake tap (< 0.22 s): latches the first step above press value.
        let mut b = KeyboardAxes { pedal_hold: true, ..Default::default() };
        b.brake_key = true;
        for _ in 0..10 {
            b.update(0.01);
        }
        b.brake_key = false;
        b.update(0.01);
        assert_eq!(b.brake, 0.10);
        // hold 1 s standing: tangent tau 0.28 s climbs past 0.9.
        let mut c = KeyboardAxes { pedal_hold: true, ..Default::default() };
        c.brake_key = true;
        for _ in 0..100 {
            c.update(0.01);
        }
        assert!(c.brake > 0.9 && c.brake <= 1.0, "{}", c.brake);
        // W (without S) cancels latched brake in 0.125 s.
        let mut d = KeyboardAxes { pedal_hold: true, brake: 0.5, ..Default::default() };
        d.throttle_key = true;
        for _ in 0..15 {
            d.update(0.01);
        }
        assert_eq!(d.brake, 0.0);
        // amplify lifts the cap to 1.0; pedal_hold=false keeps fast release.
        let mut e = KeyboardAxes::default();
        e.amplify_key = true;
        e.throttle_key = true;
        for _ in 0..100 {
            e.update(0.01);
        }
        assert_eq!(e.throttle, 1.0);
        let mut f = KeyboardAxes { pedal_hold: false, brake: 0.6, ..Default::default() };
        for _ in 0..10 {
            f.update(0.01);
        }
        assert!((f.brake - 0.3).abs() < 1e-3, "{}", f.brake);
    }

    /// The centring key brings the wheel back in a straight line at OMSI's pace, not at once.
    #[test]
    fn steering_neutral_brings_the_wheel_back_steadily() {
        let mut a = KeyboardAxes { old_steering: true, lock_curvature: 0.1, steering: 0.8, ..Default::default() };
        a.neutral_key = true;
        for _ in 0..10 {
            a.update(0.01);
        }
        a.neutral_key = false;
        assert!((a.steering - 0.75).abs() < 1e-4, "{}", a.steering);
        for _ in 0..100 {
            a.update(0.01);
        }
        assert!((a.steering - 0.25).abs() < 1e-3, "{}", a.steering);
        for _ in 0..100 {
            a.update(0.01);
        }
        assert_eq!(a.steering, 0.0);
        // a steering key ends it
        a.steering = 0.5;
        a.right_key = true;
        a.update(0.01);
        a.right_key = false;
        a.update(0.5);
        assert!(a.steering > 0.5, "old steering stays: {}", a.steering);
    }

    /// Clutch (TAB, manual only): press edge plants the 0.20 bite floor,
    /// hold climbs 0→100% in 0.45 s, release falls in 0.18 s.
    #[test]
    fn the_clutch_bites_then_climbs_and_lets_go_fast() {
        let mut a = KeyboardAxes::default();
        a.clutch_key = true;
        a.update(0.01);
        assert!((a.clutch - 0.222).abs() < 1e-3, "{}", a.clutch);
        for _ in 0..44 {
            a.update(0.01);
        }
        assert_eq!(a.clutch, 1.0);
        a.clutch_key = false;
        for _ in 0..20 {
            a.update(0.01);
        }
        assert_eq!(a.clutch, 0.0);
        // without a pedal the ramp never runs (safe state).
        let mut b = KeyboardAxes { clutch_has_pedal: false, ..Default::default() };
        b.clutch_key = true;
        for _ in 0..100 {
            b.update(0.01);
        }
        assert_eq!(b.clutch, 0.0);
    }

    /// Keyboard steering: a key turns the wheel at the speed-sensitive rate
    /// (slower with speed, agile at crawl), unwinding against the turn uses
    /// the Hermite pitch, and let go the wheel only comes back while
    /// rolling — standing still it rests where the hands left it, settling
    /// without swinging through the middle.
    #[test]
    fn steering_returns_like_a_spring() {
        let step = 1.0 / 60.0;
        // standing: ~2.1 s middle to full lock (0.336*0.70*2.0/s at 0 km/h)
        let mut s = KeyboardAxes::default();
        s.set(EngineAction::SteeringRight, true);
        let mut t_lock = None;
        for i in 1..=300 {
            s.update(step);
            if s.steering >= 1.0 && t_lock.is_none() {
                t_lock = Some(i as f32 * step);
            }
        }
        let t_lock = t_lock.expect("never reached full lock");
        assert!(
            (1.8..=2.6).contains(&t_lock),
            "middle to full lock standing took {t_lock} s"
        );
        // at 30 km/h a second of the key is about a fifth of a lock, and
        // slower still at 80
        let mut k = KeyboardAxes {
            speed_kmh: 30.0,
            ..Default::default()
        };
        k.set(EngineAction::SteeringLeft, true);
        for _ in 0..60 {
            k.update(step);
        }
        assert!(
            k.steering < -0.15 && k.steering > -0.30,
            "held left for a second at 30 km/h: {}",
            k.steering
        );
        let mut fast = KeyboardAxes {
            speed_kmh: 80.0,
            ..Default::default()
        };
        fast.set(EngineAction::SteeringLeft, true);
        for _ in 0..60 {
            fast.update(step);
        }
        assert!(
            fast.steering > k.steering,
            "turned faster at 80 km/h ({}) than at 30 ({})",
            fast.steering,
            k.steering
        );
        // let go at 30 km/h: eases back at the return rate, never through
        // the middle, and settles (angle factor + auto pitch slow it down)
        k.set(EngineAction::SteeringLeft, false);
        for _ in 0..60 {
            let before = k.steering;
            k.update(step);
            assert!(
                k.steering <= 0.0,
                "swung through the middle: {}",
                k.steering
            );
            assert!(
                k.steering >= before,
                "moved away from the middle: {} -> {}",
                before,
                k.steering
            );
        }
        assert!(
            k.steering > -0.21 && k.steering < -0.10,
            "a second of return at 30 km/h: {}",
            k.steering
        );
        for _ in 0..600 {
            k.update(step);
        }
        assert!(
            k.steering.abs() < 0.05,
            "never settled rolling: {}",
            k.steering
        );
        // standing still it stays where the hands left it (return is 0
        // at 0 km/h — the wheel rests, it is not stuck)
        let mut s = KeyboardAxes {
            steering: -0.8,
            speed_kmh: 0.0,
            ..Default::default()
        };
        for _ in 0..60 {
            s.update(step);
        }
        assert!(
            (s.steering + 0.8).abs() < 1e-6,
            "moved standing still: {}",
            s.steering
        );
        // while the mouse owns the wheel the keyboard is ignored entirely
        let mut m = KeyboardAxes {
            steering: 0.3,
            speed_kmh: 30.0,
            mouse_owned: true,
            ..Default::default()
        };
        m.set(EngineAction::SteeringRight, true);
        m.set(EngineAction::Throttle, true);
        for _ in 0..60 {
            m.update(step);
        }
        assert_eq!(m.steering, 0.3);
        assert_eq!(m.throttle, 0.0);
    }

    /// Handoff freeze (`hold_steer`): handed back from the mouse,
    /// the wheel stays put rolling until a steering key (or centering) moves
    /// it — never an automatic return, never a reset.
    #[test]
    fn handed_back_wheel_stays_until_hand_steered() {
        let step = 1.0 / 60.0;
        let mut h = KeyboardAxes {
            steering: 0.4,
            speed_kmh: 30.0,
            hold_steer: true,
            ..Default::default()
        };
        for _ in 0..120 {
            h.update(step);
        }
        assert_eq!(h.steering, 0.4);
        // a hand on the wheel clears the hold and integrates at once.
        h.set(EngineAction::SteeringLeft, true);
        h.update(step);
        assert!(h.steering < 0.4, "{}", h.steering);
        assert!(!h.hold_steer);
    }

    /// Drive keys: `W` is throttle only, `S` brake only, `A`/`D`
    /// steering only — no channel ever writes another (normal keyboard
    /// = 1 writer, the key's own integrator branch).
    #[test]
    fn vse_drive_keys_isolated() {
        let step = 0.01;
        let mut w = KeyboardAxes::default();
        w.set(EngineAction::Throttle, true);
        w.update(step);
        assert!(w.throttle > 0.0 && w.brake == 0.0 && w.steering == 0.0);
        let mut s = KeyboardAxes::default();
        s.set(EngineAction::Brake, true);
        s.update(step);
        assert!(s.brake > 0.0 && s.throttle == 0.0 && s.steering == 0.0);
        let mut a = KeyboardAxes::default();
        a.set(EngineAction::SteeringLeft, true);
        a.update(step);
        assert!(a.steering < 0.0 && a.throttle == 0.0 && a.brake == 0.0);
        let mut d = KeyboardAxes::default();
        d.set(EngineAction::SteeringRight, true);
        d.update(step);
        assert!(d.steering > 0.0 && d.throttle == 0.0 && d.brake == 0.0);
        // release: nothing sticks from another channel (residue below the
        // `1e-4` guard is the wheel at rest, not a stuck input).
        d.set(EngineAction::SteeringRight, false);
        d.speed_kmh = 30.0;
        for _ in 0..1000 {
            d.update(step);
        }
        assert!(d.steering.abs() < 1e-4, "{}", d.steering);
        assert_eq!(d.throttle, 0.0);
    }
    #[test]
    fn vse_keyboard_rates_and_return_in_motion_only() {
        let (slow, _, _) = vse_keyboard_rates_inner(0.0, 0.0, 0.70);
        let (mid, _, _) = vse_keyboard_rates_inner(40.0, 0.0, 0.70);
        let (fast, _, _) = vse_keyboard_rates_inner(80.0, 0.0, 0.70);
        assert!(slow > mid && mid > fast, "{slow} {mid} {fast}");
        let mut a = KeyboardAxes { steering_sens: 0.70, ..Default::default() };
        a.right_key = true;
        for _ in 0..60 {
            a.vse_keyboard_step(0.01, 0.0);
        }
        assert!(a.steering > 0.2 && a.steering < 0.6, "{}", a.steering);
        a.right_key = false;
        let held = a.steering;
        for _ in 0..60 {
            a.vse_return_to_centre(0.01, 0.0, 850.0, 1.0);
        }
        assert!((a.steering - held).abs() < 1e-6, "standing: no return");
        for _ in 0..600 {
            a.vse_return_to_centre(0.01, 30.0, 850.0, 1.0);
        }
        assert!(a.steering.abs() < 0.05, "rolling: settled {}", a.steering);
    }
}
