# Noise Cancler

> A real-time, confidence-aware IMU noise-mitigation prototype for autonomous quadcopters.

**Prepared for the 1517 Medici Grant application.** Noise Cancler explores a practical question for low-cost drones: how can a flight controller reject severe motor-induced IMU spikes without introducing the control-loop lag associated with broad moving-average filters?

The project is currently a **Phase 1 deterministic simulation**. It is not flight-certified and must not be used as the sole safety mechanism in an aircraft.

## The problem

High-RPM brushless motors can transmit structural vibration into inexpensive IMUs. A conventional low-pass or moving-average filter attenuates that noise by averaging across time, which also delays the signal. In a tightly coupled flight-control loop, that phase delay can make a controller react to stale motion.

## The approach

Each scalar IMU sample passes through a small, stateful pipeline running at **400 Hz** by default:

1. **Pre-filter** — a two-sample rolling average conditions electronic jitter.
2. **Differentiate** — first and second finite differences estimate rate change and jerk.
3. **Score trust** — a Gaussian factor converts jerk magnitude to measurement confidence:

   `W = exp(-(|j| / θ)²)`

4. **Predict and blend** — when confidence falls, the output blends the observed sample with a one-step kinematic prediction:

   `ȳₖ = W · yₖ + (1 − W) · ŷₖ`

When trust reaches zero for too many consecutive frames, a bounded timeout re-anchors the prediction to prevent unbounded drift.

## Repository map

```text
.
├── index.html             # 1517 Medici-oriented project landing page
├── styles.css             # Responsive landing-page styling
├── src/main.py            # Filter engine, data generator, chart CLI
└── tests/test_filter.py   # Unit and simulation checks
```

## Run the simulation

### Requirements

- Python 3.10+
- Packages listed in `requirements.txt`

```bash
python -m pip install -r requirements.txt
python src/main.py --save artifacts/simulation.png --no-show
```

The chart compares synthetic ground truth, a noisy 166 Hz motor-vibration stream, the filtered output, and the live trust weight. Inputs use seeded random generation, so the Phase 1 chart is reproducible.

To open an interactive plot instead:

```bash
python src/main.py
```

## Test

```bash
python -m pytest -q
```

The checks cover clean initialization, trust collapse on a large discontinuity, parameter/input validation, and repeatable simulation alignment.

## Design notes

- `theta` is a calibration parameter, not a universal constant. It should be tuned against the physical platform, sensor range, units, and expected maneuver envelope.
- The model operates on a scalar channel. A flight implementation should evaluate each gyroscope axis deliberately and integrate with sensor calibration, state estimation, actuator limits, failsafes, and hardware-in-the-loop testing.
- The included timeout is a guard against drifting indefinitely while input confidence remains near zero; it is not a substitute for redundancy or flight-safety validation.

## Roadmap

1. Replay recorded IMU logs and establish baseline comparisons against common filters.
2. Tune thresholds against a target airframe and quantify latency/noise trade-offs.
3. Port the state machine to a `#![no_std]` Rust target.
4. Validate in controlled tethered testing before outdoor flight trials.

## License

Released under the [MIT License](LICENSE).
