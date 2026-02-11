from __future__ import annotations

import numpy as np

from pathplanning.sampling_based_planning.rrt_3d import utils_3d


class _Env:
    def __init__(self) -> None:
        self.boundary = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.0])
        self.blocks = []
        self.OBB = []
        self.balls = []


class _InitParams:
    def __init__(self) -> None:
        self.env = _Env()
        self.xt = (0.5, 0.5, 0.5)


def _within_bounds(sample: np.ndarray, boundary: np.ndarray) -> bool:
    low, high = boundary[0:3], boundary[3:6]
    return bool(np.all(sample >= low) and np.all(sample <= high))


def test_sample_free_within_bounds() -> None:
    initparams = _InitParams()
    sample = utils_3d.sampleFree(initparams, bias=0.0, max_tries=10)
    assert _within_bounds(sample, initparams.env.boundary)


def test_sample_free_bounded_resampling(monkeypatch) -> None:
    initparams = _InitParams()

    monkeypatch.setattr(utils_3d, "isinside", lambda _initparams, _x: True)

    sample = utils_3d.sampleFree(initparams, bias=0.0, max_tries=3)
    assert _within_bounds(sample, initparams.env.boundary)
