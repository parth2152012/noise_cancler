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
├── vercel.json            # Static Vercel deployment configuration
├── src/lib.rs             # Embedded `#![no_std]` Rust filter library
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

## Rust `no_std` core

The Rust implementation in `src/lib.rs` is the embedded path for this project. It is an allocator-free `#![no_std]` library: it uses fixed-size state, has no operating-system dependency, and does not pull in a math crate. The Gaussian confidence calculation uses a bounded internal `exp(-x)` approximation, which is sufficient for the engine's `[0, 1]` trust output and its `0.05` hard clamp.

```rust
use noise_cancler::DerivativeFilterEngine;

let mut filter = DerivativeFilterEngine::new(6_500.0, 0.0025);
let result = filter.update_with_trust(gyro_x);

// Send result.value to the estimator and result.trust to telemetry.
```

Build and run the Rust unit tests with a host target:

```bash
cargo test
cargo build --release
```

For a microcontroller, add this crate to a firmware project with the appropriate Rust target and HAL. The caller must schedule `update_with_trust` at the same constant period supplied as `delta_t`; it must also validate and tune `theta` against the actual sensor units and airframe.

## Deploy the landing page

The root landing page is a static Vercel site. Import the repository in Vercel, select the **Other** framework preset, and leave the build command and output directory empty. The included `vercel.json` enables clean static URLs.

You can also deploy from the repository root with:

```bash
npx vercel --prod
```

The Python and Rust simulations are source code for development and validation; neither runs in the deployed static site.

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
