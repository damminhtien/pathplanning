"""Post-planning scenes, static rendering, and trace playback.

Matplotlib is imported only when a renderer or viewer is constructed.
"""

from pathplanning.viz.render import SceneRenderer, render_result
from pathplanning.viz.scene import Box, Circle, Rectangle, Scene, Sphere, scene_from_problem
from pathplanning.viz.viewer import Viewer, view_result

__all__ = [
    "Box",
    "Circle",
    "Rectangle",
    "Scene",
    "Sphere",
    "SceneRenderer",
    "Viewer",
    "render_result",
    "scene_from_problem",
    "view_result",
]
