"""
Plotting utilities for path planning visualizations.

This module provides classes and methods to visualize paths, visited nodes,
and obstacles in a 2D grid environment.

author: damminhtien
"""

from __future__ import annotations

from typing import Iterable, List, Sequence, Tuple

from pathplanning.viz import lazy_import

plt = lazy_import("matplotlib.pyplot")

try:
    from . import env
except ImportError:  # pragma: no cover - script execution fallback
    from utils import env

Point = Tuple[int, int]


class Plotting:
    """Matplotlib-based plotting helpers for 2D grid search visualizations."""

    # Default colors
    _DEFAULT_VISITED_COLOR = "gray"
    _DEFAULT_PATH_COLOR = "r"
    _DEFAULT_BI_FORWARD_COLOR = "gray"
    _DEFAULT_BI_BACKWARD_COLOR = "cornflowerblue"

    # Animation pacing thresholds
    _EARLY_VISITED_STEP = 20
    _MID_VISITED_STEP = 30
    _LATE_VISITED_STEP = 40

    def __init__(self, start: Point, goal: Point) -> None:
        self.start, self.goal = start, goal
        self.env = env.Env()
        self.obstacles = self.env.obs_map()
        self._figure = None  # type: ignore[var-annotated]

    # ---------------------------------------------------------------------- #
    # Public API                                                             #
    # ---------------------------------------------------------------------- #
    def update_obstacles(self, obstacles: Iterable[Point]) -> None:
        """Replace obstacle set used for plotting."""
        self.obstacles = set(obstacles)

    def animate(self, path: Sequence[Point], visited: Sequence[Point], title: str) -> None:
        """Animate a single run (e.g., A*, Dijkstra)."""
        self._ensure_figure()
        self.plot_grid(title)
        self.plot_visited(visited)
        self.plot_path(path)
        plt.show()

    def animate_lrta(
        self,
        paths: Sequence[Sequence[Point]],
        visited_sets: Sequence[Sequence[Point]],
        title: str,
    ) -> None:
        """
        Animate multi-iteration planners (LRTA*, RTAA*).
        Each iteration has its own visited set and partial path.
        """
        self._ensure_figure()
        self.plot_grid(title)
        colors = self.color_list_lrta()
        combined_path: List[Point] = []

        for iteration_index, path_segment in enumerate(paths):
            color = colors[iteration_index % len(colors)]
            self.plot_visited(
                visited_sets[iteration_index], color=color, pause_time=0.2)
            self.plot_path(path_segment, pause_time=0.2)
            combined_path.extend(path_segment)

        # Remove duplicates of start node
        combined_path = [p for p in combined_path if p != self.start]
        self.plot_path(combined_path)
        plt.show()

    def animate_ara_star(
        self,
        paths: Sequence[Sequence[Point]],
        visited_sets: Sequence[Sequence[Point]],
        title: str,
    ) -> None:
        """Animate anytime algorithms (ARA*, AD* variants)."""
        self._ensure_figure()
        self.plot_grid(title)
        visited_colors, path_colors = self.color_list_anytime()

        for iteration_index, path_segment in enumerate(paths):
            self.plot_visited(
                visited_sets[iteration_index],
                color=visited_colors[iteration_index % len(visited_colors)],
                pause_time=0.5,
            )
            self.plot_path(
                path_segment,
                color=path_colors[iteration_index % len(path_colors)],
                use_custom_color=True,
                pause_time=0.5,
            )

        plt.show()

    def animate_bi_astar(
        self,
        path: Sequence[Point],
        visited_forward: Sequence[Point],
        visited_backward: Sequence[Point],
        title: str,
    ) -> None:
        """Animate bidirectional A* with forward and backward frontiers."""
        self._ensure_figure()
        self.plot_grid(title)
        self.plot_visited_bidirectional(visited_forward, visited_backward)
        self.plot_path(path)
        plt.show()

    # ---------------------------------------------------------------------- #
    # Low-level plotters                                                     #
    # ---------------------------------------------------------------------- #
    def plot_grid(self, title: str) -> None:
        """Draw start, goal, and static obstacles."""
        ax = plt.gca()
        ax.set_title(title)
        ax.set_aspect("equal", adjustable="box")

        if self.obstacles:
            ox, oy = zip(*self.obstacles)
            ax.plot(ox, oy, "sk")

        ax.plot(self.start[0], self.start[1], "bs", label="Start")
        ax.plot(self.goal[0], self.goal[1], "gs", label="Goal")

    def plot_visited(
        self,
        visited_nodes: Sequence[Point],
        color: str = _DEFAULT_VISITED_COLOR,
        pause_time: float = 0.01,
    ) -> None:
        """Plot visited nodes with animation pacing."""
        if not visited_nodes:
            return

        ax = plt.gca()
        filtered_nodes = [
            p for p in visited_nodes if p not in {self.start, self.goal}]
        if not filtered_nodes:
            return

        total_nodes = len(filtered_nodes)
        for index, (x, y) in enumerate(filtered_nodes, 1):
            ax.plot(x, y, marker="o", linestyle="None", color=color)

            # Adjust pacing based on progress
            if index < total_nodes / 3:
                step = self._EARLY_VISITED_STEP
            elif index < (2 * total_nodes) / 3:
                step = self._MID_VISITED_STEP
            else:
                step = self._LATE_VISITED_STEP

            if index % step == 0:
                plt.pause(0.001)

        plt.pause(pause_time)

    def plot_path(
        self,
        path: Sequence[Point],
        color: str = _DEFAULT_PATH_COLOR,
        use_custom_color: bool = False,
        pause_time: float = 0.01,
    ) -> None:
        """Plot a path polyline along with start and goal markers."""
        if not path:
            return

        ax = plt.gca()
        px, py = zip(*path)
        ax.plot(px, py, linewidth=3,
                color=color if use_custom_color else self._DEFAULT_PATH_COLOR)

        # Redraw start and goal
        ax.plot(self.start[0], self.start[1], "bs")
        ax.plot(self.goal[0], self.goal[1], "gs")
        plt.pause(pause_time)

    def plot_visited_bidirectional(
        self, visited_forward: Sequence[Point], visited_backward: Sequence[Point]
    ) -> None:
        """Plot interleaved visitation order for bidirectional search."""
        ax = plt.gca()

        forward_nodes = [p for p in visited_forward if p != self.start]
        backward_nodes = [p for p in visited_backward if p != self.goal]

        max_length = max(len(forward_nodes), len(backward_nodes))
        for index in range(max_length):
            if index < len(forward_nodes):
                fx, fy = forward_nodes[index]
                ax.plot(fx, fy, marker="o", linestyle="None",
                        color=self._DEFAULT_BI_FORWARD_COLOR)
            if index < len(backward_nodes):
                bx, by = backward_nodes[index]
                ax.plot(bx, by, marker="o", linestyle="None",
                        color=self._DEFAULT_BI_BACKWARD_COLOR)

            if index % 10 == 0:
                plt.pause(0.001)

        plt.pause(0.01)

    # ---------------------------------------------------------------------- #
    # Color helpers                                                          #
    # ---------------------------------------------------------------------- #
    @staticmethod
    def color_list_anytime() -> Tuple[List[str], List[str]]:
        """(visited_colors, path_colors) for multi-iteration plots (ARA*, etc.)."""
        visited_colors = ["silver", "wheat",
                          "lightskyblue", "royalblue", "slategray"]
        path_colors = ["gray", "orange", "deepskyblue", "red", "m"]
        return visited_colors, path_colors

    @staticmethod
    def color_list_lrta() -> List[str]:
        """Color cycle used by LRTA*/RTAA* iteration animations."""
        return [
            "silver",
            "steelblue",
            "dimgray",
            "cornflowerblue",
            "dodgerblue",
            "royalblue",
            "plum",
            "mediumslateblue",
            "mediumpurple",
            "blueviolet",
        ]

    # ---------------------------------------------------------------------- #
    # Internals                                                              #
    # ---------------------------------------------------------------------- #
    def _ensure_figure(self) -> None:
        """Create a figure and bind the ESC key once per figure."""
        if self._figure is None:
            self._figure = plt.figure()
            self._figure.canvas.mpl_connect(
                "key_release_event",
                lambda event: exit(0) if event.key == "escape" else None,
            )
