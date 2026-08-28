import math

import pytest

from src.main import DerivativeFilterEngine, generate_flight_data, run_simulation


def test_first_sample_initializes_without_artificial_zero():
    output, trust = DerivativeFilterEngine().update(12.5)
    assert output == 12.5
    assert trust == 1.0


def test_large_discontinuity_drops_sensor_trust():
    engine = DerivativeFilterEngine(theta=10.0, delta_t=0.01)
    engine.update(0.0)
    _, trust = engine.update(100.0)
    assert trust == 0.0


def test_invalid_parameters_and_samples_are_rejected():
    with pytest.raises(ValueError):
        DerivativeFilterEngine(theta=0)
    with pytest.raises(ValueError):
        DerivativeFilterEngine().update(math.nan)


def test_simulation_is_reproducible_and_aligned():
    time, truth, raw = generate_flight_data()
    _, _, _, output, trust = run_simulation()
    assert len(time) == len(truth) == len(raw) == len(output) == len(trust) == 600
    assert min(trust) == 0.0
