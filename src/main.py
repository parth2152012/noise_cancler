"""Deterministic simulation for the zero-lag IMU derivative filter.

Run ``python src/main.py --save artifacts/simulation.png`` to regenerate the
evaluation chart, or omit ``--save`` to open an interactive plot.
"""

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass, field
from pathlib import Path

@dataclass
class DerivativeFilterEngine:
    """Reject implausible second-derivative events without adding phase lag.

    ``theta`` is the jerk threshold and ``delta_t`` is the sample interval in
    seconds. Samples should use one consistent angular-rate unit.
    """

    theta: float = 5_500.0
    delta_t: float = 0.0025
    timeout_frames: int = 4
    buffer: list[float] = field(default_factory=list, init=False)
    prev_v: float = field(default=0.0, init=False)
    prev_y_bar: float = field(default=0.0, init=False)
    consecutive_zeros: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        if self.theta <= 0 or self.delta_t <= 0:
            raise ValueError("theta and delta_t must both be positive")
        if self.timeout_frames < 0:
            raise ValueError("timeout_frames cannot be negative")

    def update(self, raw_input: float) -> tuple[float, float]:
        """Process one sample and return ``(filtered_value, trust_weight)``."""
        if not math.isfinite(raw_input):
            raise ValueError("raw_input must be finite")

        # Warm up from the first sensor value rather than an artificial zero.
        if not self.buffer:
            self.buffer = [float(raw_input)] * 3
            self.prev_y_bar = float(raw_input)
            return float(raw_input), 1.0

        # A two-sample pre-filter suppresses electronic jitter before the
        # derivative calculation. buffer is [y_k-1, y_k-2, y_k-3].
        filtered_y = (float(raw_input) + self.buffer[0]) / 2.0
        y_k_1, y_k_2 = self.buffer[0], self.buffer[1]
        v_k = (filtered_y - y_k_1) / self.delta_t
        jerk_k = (filtered_y - 2.0 * y_k_1 + y_k_2) / self.delta_t**2
        self.buffer = [filtered_y, y_k_1, y_k_2]

        ratio = abs(jerk_k) / self.theta
        trust = math.exp(-(ratio * ratio))
        if trust < 0.05:
            trust = 0.0
            self.consecutive_zeros += 1
        else:
            self.consecutive_zeros = 0

        predicted = self.prev_y_bar + (self.prev_v * self.delta_t)
        if self.consecutive_zeros > self.timeout_frames:
            predicted = filtered_y

        output = trust * filtered_y + (1.0 - trust) * predicted
        self.prev_v = v_k
        self.prev_y_bar = output
        return output, trust


def generate_flight_data(duration: float = 1.5, fs: int = 400, seed: int = 42):
    """Return reproducible synthetic flight, noisy IMU, and time samples."""
    if duration <= 0 or fs <= 0:
        raise ValueError("duration and fs must be positive")
    rng = random.Random(seed)
    sample_count = int(duration * fs)
    time = [index / fs for index in range(sample_count)]
    true_path = [45.0 * math.sin(2 * math.pi * 1.5 * value) for value in time]
    motor_noise = [12.0 * math.sin(2 * math.pi * 166.0 * value) for value in time]
    raw_stream = [path + noise + rng.gauss(0, 3.5) for path, noise in zip(true_path, motor_noise)]
    for start, end, offset in ((180, 183, 180.0), (420, 423, -220.0)):
        if start < len(raw_stream):
            for index in range(start, min(end, len(raw_stream))):
                raw_stream[index] += offset
    return time, true_path, raw_stream


def run_simulation(duration: float = 1.5, fs: int = 400):
    """Run the filter and return time, ground truth, raw data, output, trust."""
    time, true_path, raw_stream = generate_flight_data(duration, fs)
    engine = DerivativeFilterEngine(theta=6_500.0, delta_t=1.0 / fs)
    results = [engine.update(sample) for sample in raw_stream]
    output, trust = map(list, zip(*results))
    return time, true_path, raw_stream, output, trust


def create_figure():
    """Build the evaluation figure without displaying it."""
    import matplotlib.pyplot as plt

    time, true_path, raw_stream, output, trust = run_simulation()
    figure, (signal_ax, trust_ax) = plt.subplots(
        2, 1, figsize=(13, 8), sharex=True, gridspec_kw={"height_ratios": [3, 1]}
    )
    signal_ax.plot(time, raw_stream, label="Raw IMU (motor noise + spikes)", color="#e63946", alpha=0.5)
    signal_ax.plot(time, output, label="Derivative filter output", color="#1d3557", linewidth=2.5)
    signal_ax.plot(time, true_path, label="Synthetic ground truth", color="#457b9d", linestyle="--")
    signal_ax.set(title="Phase 1: real-time IMU filtering simulation", ylabel="Angular rate (deg/s)")
    signal_ax.legend(loc="upper right")
    trust_ax.fill_between(time, trust, color="#2a9d8f", alpha=0.3, label="Sensor trust")
    trust_ax.set(ylim=(-0.1, 1.1), xlabel="Time (seconds)", ylabel="Trust (W)")
    trust_ax.legend(loc="upper right")
    for axis in (signal_ax, trust_ax):
        axis.grid(True, linestyle=":", alpha=0.6)
    figure.tight_layout()
    return figure


def main() -> None:
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(description="Run the IMU filter simulation.")
    parser.add_argument("--save", type=Path, help="write the chart to this path")
    parser.add_argument("--no-show", action="store_true", help="do not open a plot window")
    args = parser.parse_args()
    figure = create_figure()
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.save, dpi=160, bbox_inches="tight")
        print(f"Saved simulation chart to {args.save}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
