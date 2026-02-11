from __future__ import annotations

from pathplanning.sampling_based_planning.rrt_3d import rrt_star_3d


class _DummyPlot:
    def figure(self, *args, **kwargs):  # noqa: D401 - simple test stub
        return object()

    def title(self, *args, **kwargs):  # noqa: D401 - simple test stub
        return None

    def show(self, *args, **kwargs):  # noqa: D401 - simple test stub
        return None


def test_rrt_star_3d_run_no_starttime_nameerror(monkeypatch) -> None:
    """run() should not rely on a global starttime value."""
    monkeypatch.setattr(rrt_star_3d, "plt", _DummyPlot())
    monkeypatch.setattr(rrt_star_3d, "visualization", lambda *args, **kwargs: None)

    planner = rrt_star_3d.rrtstar()
    planner.maxiter = 0
    planner.run()
