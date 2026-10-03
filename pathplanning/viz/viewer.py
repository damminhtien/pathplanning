"""Desktop playback controls for a completed planner trace."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from pathplanning.viz.render import SceneRenderer
from pathplanning.viz.replay import ReplayController
from pathplanning.viz.scene import Scene


class Viewer:
    """Own one Matplotlib window, timer, widgets, and their event handlers."""

    def __init__(
        self,
        scene: Scene,
        result: Any,
        *,
        positions: Mapping[Any, Any] | None = None,
    ) -> None:
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, CheckButtons, Slider

        self.figure = plt.figure(figsize=(11, 8))
        ax = self.figure.add_axes(
            [0.08, 0.30, 0.72, 0.65],
            projection="3d" if scene.dimension == 3 else None,
        )
        self.renderer = SceneRenderer(scene, result, ax=ax)
        self.trace = getattr(result, "trace", None)
        self.controller = ReplayController(self.trace) if self.trace is not None else None
        self.positions = positions
        if self.controller is not None and not self.renderer.path_drawn:
            final_state = self.controller.seek(self.controller.length)
            if final_state.solution_path:
                self.renderer.set_path_from_trace(
                    final_state.solution_path, self.trace, positions=positions
                )
            self.controller.seek(0)
        self.playing = False
        self.speed = 1
        self.closed = False
        self._setting_slider = False

        self._status = self.figure.text(0.08, 0.26, "", fontsize=9)
        self._widgets: list[Any] = []
        self._widget_callbacks: list[tuple[Any, int]] = []
        if self.controller is not None:
            self._play_button = Button(self.figure.add_axes([0.08, 0.19, 0.11, 0.05]), "Play")
            self._back_button = Button(self.figure.add_axes([0.20, 0.19, 0.08, 0.05]), "◀")
            self._next_button = Button(self.figure.add_axes([0.29, 0.19, 0.08, 0.05]), "▶")
            self._position_slider = Slider(
                self.figure.add_axes([0.47, 0.19, 0.32, 0.035]),
                "Event",
                0,
                max(1, self.controller.length),
                valinit=0,
                valstep=1,
                valfmt="%d",
            )
            self._speed_slider = Slider(
                self.figure.add_axes([0.08, 0.115, 0.71, 0.035]),
                "Events/frame",
                1,
                1000,
                valinit=1,
                valstep=1,
                valfmt="%d",
            )
            self._widgets.extend(
                (
                    self._play_button,
                    self._back_button,
                    self._next_button,
                    self._position_slider,
                    self._speed_slider,
                )
            )
            self._widget_callbacks.extend(
                (
                    (
                        self._play_button,
                        self._play_button.on_clicked(lambda _event: self.toggle_play()),
                    ),
                    (self._back_button, self._back_button.on_clicked(lambda _event: self.step(-1))),
                    (self._next_button, self._next_button.on_clicked(lambda _event: self.step(1))),
                    (self._position_slider, self._position_slider.on_changed(self._on_seek)),
                    (self._speed_slider, self._speed_slider.on_changed(self._on_speed)),
                )
            )

        self._layers_widget = CheckButtons(
            self.figure.add_axes([0.82, 0.31, 0.16, 0.46]),
            list(self.renderer.layers),
            [True] * len(self.renderer.layers),
        )
        self._widget_callbacks.append(
            (self._layers_widget, self._layers_widget.on_clicked(self._toggle_layer))
        )
        self._widgets.append(self._layers_widget)

        self._timer = self.figure.canvas.new_timer(interval=50)
        self._timer.add_callback(self._tick)
        self._key_cid = self.figure.canvas.mpl_connect("key_press_event", self._on_key)
        self._close_cid = self.figure.canvas.mpl_connect("close_event", self._on_close)
        self.seek(0)

    @property
    def position(self) -> int:
        return 0 if self.controller is None else self.controller.position

    def _on_seek(self, value: float) -> None:
        if not self._setting_slider:
            self.seek(int(value))

    def _on_speed(self, value: float) -> None:
        self.speed = max(1, int(value))

    def _toggle_layer(self, label: str) -> None:
        labels = list(self.renderer.layers)
        index = labels.index(label)
        self.renderer.set_layer_visible(label, self._layers_widget.get_status()[index])

    def _on_key(self, event: Any) -> None:
        if event.key == "escape":
            self.close()
        elif event.key == " ":
            self.toggle_play()
        elif event.key == "left":
            self.step(-1)
        elif event.key == "right":
            self.step(1)

    def _on_close(self, _event: Any) -> None:
        self._dispose()

    def _tick(self) -> None:
        if not self.playing or self.controller is None:
            return
        self.step(self.speed)
        if self.position >= self.controller.length:
            self.pause()

    def seek(self, position: int) -> None:
        if self.closed or self.controller is None:
            self._status.set_text("Static result; request trace to replay the search")
            self.figure.canvas.draw_idle()
            return
        state = self.controller.seek(position)
        self.renderer.show_state(state, self.trace, positions=self.positions)
        self._setting_slider = True
        try:
            if int(self._position_slider.val) != self.position:
                self._position_slider.set_val(self.position)
        finally:
            self._setting_slider = False
        suffix = " · trace truncated" if getattr(self.trace, "truncated", False) else ""
        self._status.set_text(
            f"Event {self.position}/{self.controller.length} · phase {state.phase}{suffix}"
        )
        self.figure.canvas.draw_idle()

    def step(self, count: int = 1) -> None:
        self.seek(self.position + count)

    def play(self) -> None:
        if self.closed or self.controller is None or self.controller.length == 0:
            return
        if self.position >= self.controller.length:
            self.seek(0)
        self.playing = True
        self._play_button.label.set_text("Pause")
        self._timer.start()

    def pause(self) -> None:
        self.playing = False
        self._timer.stop()
        if self.controller is not None:
            self._play_button.label.set_text("Play")

    def toggle_play(self) -> None:
        if self.playing:
            self.pause()
        else:
            self.play()

    def _dispose(self) -> None:
        if self.closed:
            return
        self.closed = True
        self._timer.stop()
        self._timer.remove_callback(self._tick)
        self.figure.canvas.mpl_disconnect(self._key_cid)
        self.figure.canvas.mpl_disconnect(self._close_cid)
        for widget, callback_id in self._widget_callbacks:
            widget.disconnect(callback_id)
        for widget in self._widgets:
            widget.disconnect_events()
        self._widget_callbacks.clear()
        self._widgets.clear()

    def close(self) -> None:
        import matplotlib.pyplot as plt

        self._dispose()
        plt.close(self.figure)

    def show(self, *, block: bool = False) -> None:
        import matplotlib.pyplot as plt

        plt.show(block=block)


def view_result(
    scene: Scene,
    result: Any,
    *,
    positions: Mapping[Any, Any] | None = None,
) -> Viewer:
    """Open a desktop viewer after the planner returned its result."""
    viewer = Viewer(scene, result, positions=positions)
    viewer.show(block=False)
    return viewer


__all__ = ["Viewer", "view_result"]
