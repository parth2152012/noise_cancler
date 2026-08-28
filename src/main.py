import numpy as np
import matplotlib.pyplot as plt


# =====================================================================
# 1. CORE FILTER ENGINE (Matching your Rust bare-metal architecture)
# =====================================================================
class DerivativeFilterEngine:
    def __init__(self, theta=5500.0, delta_t=0.0025):
        self.theta = theta  # Jerk threshold (Θ)
        self.delta_t = delta_t  # 400 Hz refresh step (Δt)
        self.buffer = [0.0, 0.0, 0.0]  # Sliding window: [y_k, y_k-1, y_k-2]
        self.prev_v = 0.0  # Velocity (v_k-1)
        self.prev_y_bar = 0.0  # Last safe state (y_bar_k-1)
        self.consecutive_zeros = 0  # Emergency timeout tracker

    def update(self, raw_input):
        # --- STEP A: Low-Pass Pre-Filter ---
        # 2-sample window to smooth high-frequency electronic jitter
        filtered_y = (raw_input + self.buffer[0]) / 2.0

        # Cycle the sliding memory buffer
        self.buffer[2] = self.buffer[1]
        self.buffer[1] = self.buffer[0]
        self.buffer[0] = filtered_y

        # --- STEP B: Discrete Time Derivatives ---
        # First derivative (Angular Acceleration baseline)
        v_k = (self.buffer[0] - self.buffer[1]) / self.delta_t

        # Second derivative (Angular Jerk baseline)
        a_k = (self.buffer[0] - 2.0 * self.buffer[1] + self.buffer[2]) / (
            self.delta_t**2
        )

        # --- STEP C: Dynamic Gaussian Trust Factor (W) ---
        jerk_magnitude = abs(a_k)
        ratio = jerk_magnitude / self.theta
        w = np.exp(-1.0 * (ratio**2))

        # --- STEP D: Kinematic Prediction & Bounded Timeout ---
        if w < 0.05:
            w = 0.0
            self.consecutive_zeros += 1
        else:
            self.consecutive_zeros = 0

        # Localized Taylor series trajectory projection
        y_hat_k = self.prev_y_bar + (self.prev_v * self.delta_t)

        # Emergency Timeout Fallback (Hard boundary to stop prediction drift)
        if self.consecutive_zeros > 4:
            y_hat_k = filtered_y

        # Convex Blend Output
        y_bar_k = (w * filtered_y) + ((1.0 - w) * y_hat_k)

        # Cache states for step k+1
        self.prev_v = v_k
        self.prev_y_bar = y_bar_k

        return y_bar_k, w


# =====================================================================
# 2. FLIGHT DATA GENERATION (Simulating a real 5-inch quadcopter environment)
# =====================================================================
np.random.seed(42)  # Deterministic evaluation
duration = 1.5  # Seconds
fs = 400  # 400 Hz Sampling rate
t = np.linspace(0, duration, int(fs * duration), endpoint=False)

# Intentional Pilot Input: Smooth, clean 1.5 Hz flight roll maneuver
true_flight_path = 45.0 * np.sin(2 * np.pi * 1.5 * t)

# 10,000+ RPM Motor Vibrations (Structural noise modeled at ~166Hz + random noise)
motor_vibration_hz = 166.0
motor_harmonics = 12.0 * np.sin(2 * np.pi * motor_vibration_hz * t)
sensor_noise = np.random.normal(0, 3.5, len(t))

# Combine to create raw compromised IMU feedback
raw_imu_stream = true_flight_path + motor_harmonics + sensor_noise

# Inject mathematically "impossible" mechanical/electronic glitch shocks
raw_imu_stream[180:183] += 180.0  # Massive positive spike
raw_imu_stream[420:423] -= 220.0  # Massive negative spike

# =====================================================================
# 3. RUN SIMULATION LOOP
# =====================================================================
engine = DerivativeFilterEngine(theta=6500.0, delta_t=1.0 / fs)
clean_output = []
trust_weights = []

for sample in raw_imu_stream:
    filtered_val, current_w = engine.update(sample)
    clean_output.append(filtered_val)
    trust_weights.append(current_w)

# =====================================================================
# 4. DASHBOARD PRESENTATION FOR INTERVIEW JUDGES
# =====================================================================
fig, (ax1, ax2) = plt.subplots(
    2, 1, figsize=(13, 8), sharex=True, gridspec_kw={"height_ratios": [3, 1]}
)

# Top Plot: Signal Comparison
ax1.plot(
    t,
    raw_imu_stream,
    label="Raw IMU Sensor (Motor Noise + Spikes)",
    color="#E63946",
    alpha=0.5,
    linewidth=1.2,
)
ax1.plot(
    t,
    clean_output,
    label="Cleaned Flight Path (Our Filter Engine)",
    color="#1D3557",
    linewidth=2.5,
)
ax1.plot(
    t,
    true_flight_path,
    label="True Ground Physical Position",
    color="#457B9D",
    linestyle="--",
    alpha=0.8,
)

ax1.set_title(
    "Phase 1 Simulation: Real-Time IMU Mathematical Filtering Verification",
    fontsize=14,
    fontweight="bold",
    pad=15,
)
ax1.set_ylabel("Angular Rate / Attitude (deg/s)", fontsize=11)
ax1.legend(loc="upper right", frameon=True, facecolor="white", edgecolor="none")
ax1.grid(True, linestyle=":", alpha=0.6)

# Bottom Plot: Dynamic Gaussian Trust Weight tracking
ax2.fill_between(
    t, trust_weights, color="#2A9D8F", alpha=0.3, label="Sensor Trust Factor (W)"
)
ax2.plot(t, trust_weights, color="#2A9D8F", linewidth=1.5)
ax2.set_ylim(-0.1, 1.1)
ax2.set_ylabel("Trust Weight (W)", fontsize=11)
ax2.set_xlabel("Time Execution Window (Seconds)", fontsize=11)
ax2.grid(True, linestyle=":", alpha=0.6)
ax2.legend(loc="upper right")

plt.tight_layout()
plt.show()
