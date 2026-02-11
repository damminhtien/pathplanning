from __future__ import annotations

from pathplanning.sampling_based_planning.rrt_3d.utils_3d import path


class _InitParams:
    def __init__(self) -> None:
        self.x0 = (0.0, 0.0, 0.0)
        self.xt = (1.0, 0.0, 0.0)
        self.parent_by_node = {self.xt: self.x0}


def test_path_does_not_accumulate_across_calls() -> None:
    initparams = _InitParams()

    first_path, first_dist = path(initparams)
    second_path, second_dist = path(initparams)

    assert first_path is not second_path
    assert len(first_path) == 1
    assert len(second_path) == 1
    assert first_dist == second_dist
