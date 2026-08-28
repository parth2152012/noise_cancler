#![no_std]
//! A fixed-memory, confidence-aware IMU derivative filter.
//! Under testing 
//!
//! This crate is designed for a regularly clocked bare-metal control loop.
//! It has no allocator, no operating-system dependencies, and no external
//! math crate. The Gaussian trust calculation uses a bounded `exp(-x)`
//! approximation suitable for the filter's `[0, 1]` confidence output.

/// One filter result, including the filtered value and the input's trust.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FilterOutput {
    /// Confidence-aware, phase-corrected output.
    pub value: f32,
    /// Gaussian confidence in the current input, clamped to `[0.0, 1.0]`.
    pub trust: f32,
}

/// Non-linear derivative filtering engine for real-time IMU noise mitigation.
///
/// `theta` is a jerk threshold in the caller's chosen units, and `delta_t`
/// is the fixed control-loop period in seconds (for example, `0.0025` at
/// 400 Hz). Calibrate `theta` on the target airframe; it is not universal.
pub struct DerivativeFilterEngine {
    theta: f32,
    delta_t: f32,
    timeout_frames: u32,
    buffer: [f32; 3],
    prev_v: f32,
    prev_y_bar: f32,
    consecutive_zeros: u32,
    initialized: bool,
}

impl DerivativeFilterEngine {
    /// Creates an engine with a four-frame prediction timeout.
    ///
    /// Use [`Self::has_valid_configuration`] before arming when configuration
    /// values originate outside firmware constants.
    pub const fn new(theta: f32, delta_t: f32) -> Self {
        Self::with_timeout(theta, delta_t, 4)
    }

    /// Creates an engine with an explicit maximum number of zero-trust frames.
    pub const fn with_timeout(theta: f32, delta_t: f32, timeout_frames: u32) -> Self {
        Self {
            theta,
            delta_t,
            timeout_frames,
            buffer: [0.0; 3],
            prev_v: 0.0,
            prev_y_bar: 0.0,
            consecutive_zeros: 0,
            initialized: false,
        }
    }

    /// Returns whether the threshold and sample interval are safe to use.
    pub fn has_valid_configuration(&self) -> bool {
        self.theta.is_finite() && self.theta > 0.0 && self.delta_t.is_finite() && self.delta_t > 0.0
    }

    /// Processes one IMU-axis sample and returns its filtered value and trust.
    ///
    /// Non-finite input is rejected without changing the engine state.
    pub fn update_with_trust(&mut self, raw_input: f32) -> FilterOutput {
        if !self.has_valid_configuration() || !raw_input.is_finite() {
            return FilterOutput {
                value: self.prev_y_bar,
                trust: 0.0,
            };
        }

        // Start from a real sensor value, avoiding an artificial zero transient.
        if !self.initialized {
            self.buffer = [raw_input; 3];
            self.prev_y_bar = raw_input;
            self.initialized = true;
            return FilterOutput {
                value: raw_input,
                trust: 1.0,
            };
        }

        // Step A: two-sample pre-filter.
        let filtered_y = (raw_input + self.buffer[0]) * 0.5;
        let y_k_1 = self.buffer[0];
        let y_k_2 = self.buffer[1];

        // Step B: discrete first and second derivatives.
        let velocity = (filtered_y - y_k_1) / self.delta_t;
        let jerk = (filtered_y - 2.0 * y_k_1 + y_k_2) / (self.delta_t * self.delta_t);
        self.buffer = [filtered_y, y_k_1, y_k_2];

        // Step C: Gaussian confidence, W = exp(-(|jerk| / theta)^2).
        let ratio = absolute(jerk) / self.theta;
        let mut trust = exp_negative(ratio * ratio);
        if trust < 0.05 {
            trust = 0.0;
            self.consecutive_zeros = self.consecutive_zeros.saturating_add(1);
        } else {
            self.consecutive_zeros = 0;
        }

        // Step D: short-horizon prediction, blend, and bounded re-anchor.
        let mut predicted = self.prev_y_bar + self.prev_v * self.delta_t;
        if self.consecutive_zeros > self.timeout_frames {
            predicted = filtered_y;
        }
        let value = trust * filtered_y + (1.0 - trust) * predicted;
        self.prev_v = velocity;
        self.prev_y_bar = value;
        FilterOutput { value, trust }
    }

    /// Processes one sample and returns only the corrected output value.
    pub fn update(&mut self, raw_input: f32) -> f32 {
        self.update_with_trust(raw_input).value
    }

    /// Clears all state, for example when transitioning from disarmed to armed.
    pub fn reset(&mut self) {
        self.buffer = [0.0; 3];
        self.prev_v = 0.0;
        self.prev_y_bar = 0.0;
        self.consecutive_zeros = 0;
        self.initialized = false;
    }
}

#[inline]
fn absolute(value: f32) -> f32 {
    if value < 0.0 { -value } else { value }
}

/// Approximate `exp(-x)` for non-negative `x` without a `std` or `libm` dependency.
///
/// Range reduction bounds the Taylor polynomial to `0 <= remainder < ln(2)`.
/// The filter hard-clamps confidence below 0.05, so values past `x = 10` do
/// not affect the observable output.
fn exp_negative(x: f32) -> f32 {
    if !x.is_finite() || x >= 10.0 {
        return 0.0;
    }
    if x <= 0.0 {
        return 1.0;
    }

    const LN_2: f32 = 0.693_147_2;
    let exponent = (x / LN_2) as u32;
    let remainder = x - exponent as f32 * LN_2;
    let squared = remainder * remainder;
    let fourth = squared * squared;
    // Sixth-order Taylor series for exp(-remainder).
    let polynomial = 1.0 - remainder + squared * 0.5 - squared * remainder / 6.0 + fourth / 24.0
        - fourth * remainder / 120.0
        + fourth * squared / 720.0;

    // 2^-exponent; exponent is at most 14 because x < 10.
    let mut scale = 1.0;
    let mut index = 0;
    while index < exponent {
        scale *= 0.5;
        index += 1;
    }
    polynomial * scale
}

#[cfg(test)]
mod tests {
    use super::DerivativeFilterEngine;

    #[test]
    fn starts_from_first_measurement() {
        let mut filter = DerivativeFilterEngine::new(6_500.0, 0.0025);
        assert_eq!(filter.update_with_trust(12.5).value, 12.5);
    }

    #[test]
    fn rejects_a_large_discontinuity() {
        let mut filter = DerivativeFilterEngine::new(10.0, 0.01);
        filter.update(0.0);
        assert_eq!(filter.update_with_trust(100.0).trust, 0.0);
    }

    #[test]
    fn reset_restarts_initialization() {
        let mut filter = DerivativeFilterEngine::new(10.0, 0.01);
        filter.update(3.0);
        filter.reset();
        assert_eq!(filter.update_with_trust(-2.0).value, -2.0);
    }
}
