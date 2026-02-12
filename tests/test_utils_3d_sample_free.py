from __future__ import annotations

import numpy as np

import pathplanning.utils.sampling3d as utils_3d


class _Env:
    def __init__(self) -> None:
        self.boundary = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.0])
        self.blocks = []
        self.obb = []
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
    sample = utils_3d.sample_free(initparams, bias=0.0, max_tries=10)
    assert _within_bounds(sample, initparams.env.boundary)


def test_sample_free_bounded_resampling(monkeypatch) -> None:
    initparams = _InitParams()

    monkeypatch.setattr(utils_3d, "is_inside", lambda _initparams, _x: True)

    sample = utils_3d.sample_free(initparams, bias=0.0, max_tries=3)
    assert _within_bounds(sample, initparams.env.boundary)
