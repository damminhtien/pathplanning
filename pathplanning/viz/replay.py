"""Deterministic, bounded-memory playback of native planner events."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass, field
import math
from typing import Any

DISCOVER = 1
EXPAND = 2
PARENT = 3
SAMPLE = 4
PHASE = 5
SOLUTION = 6
PRUNE = 7
NO_NODE = (1 << 64) - 1


@dataclass(slots=True)
class ReplayState:
    visited: set[tuple[int, int]] = field(default_factory=set)
    frontier: set[tuple[int, int]] = field(default_factory=set)
    samples: set[int] = field(default_factory=set)
    parents: dict[tuple[int, int], int] = field(default_factory=dict)
    solution_node: int | None = None
    solution_path: tuple[int, ...] = ()
    phase: int = 0

    def copy(self) -> ReplayState:
        return ReplayState(
            visited=self.visited.copy(),
            frontier=self.frontier.copy(),
            samples=self.samples.copy(),
            parents=self.parents.copy(),
            solution_node=self.solution_node,
            solution_path=self.solution_path,
            phase=self.phase,
        )

    def bytes_estimate(self) -> int:
        return (
            128
            + 80 * (len(self.visited) + len(self.frontier) + len(self.samples))
            + 112 * len(self.parents)
            + 8 * len(self.solution_path)
        )

    def path_to(self, node: int, side: int) -> tuple[int, ...]:
        chain = [node]
        seen = {node}
        while (side, chain[-1]) in self.parents:
            parent = self.parents[(side, chain[-1])]
            if parent in seen:
                return ()
            chain.append(parent)
            seen.add(parent)
        return tuple(chain)


class ReplayController:
    """Seek by replaying from the nearest cached checkpoint.

    Checkpoints are an optional acceleration and never retain more than
    ``max_cache_bytes``. All event data belongs to the caller's trace object.
    """

    def __init__(
        self,
        trace: Any,
        *,
        checkpoint_stride: int = 512,
        max_cache_bytes: int = 8 * 1024 * 1024,
    ) -> None:
        events = trace.events
        fields = getattr(getattr(events, "dtype", None), "names", None)
        if fields is None or not {"kind", "node", "parent", "side"}.issubset(fields):
            raise TypeError("trace.events must be a structured array with kind/node/parent/side")
        if checkpoint_stride <= 0 or max_cache_bytes < 0:
            raise ValueError("checkpoint limits must be non-negative, with a positive stride")
        self.trace = trace
        self.events = events
        self.checkpoint_stride = checkpoint_stride
        self.max_cache_bytes = max_cache_bytes
        self.position = 0
        self.state = ReplayState()
        self._checkpoints: OrderedDict[int, ReplayState] = OrderedDict()
        self._cache_bytes = 0

    @property
    def length(self) -> int:
        return len(self.events)

    @property
    def cache_bytes(self) -> int:
        return self._cache_bytes

    def _apply(self, event: Any) -> None:
        kind = int(event["kind"])
        node = int(event["node"])
        parent = int(event["parent"])
        side = int(event["side"])
        if node == NO_NODE:
            node = -1
        if parent == NO_NODE:
            parent = -1
        key = (side, node)
        if kind == DISCOVER:
            if node >= 0:
                self.state.frontier.add(key)
                if parent >= 0:
                    self.state.parents[key] = parent
        elif kind == EXPAND:
            if node >= 0:
                self.state.frontier.discard(key)
                self.state.visited.add(key)
        elif kind == PARENT:
            if node >= 0:
                if parent >= 0:
                    self.state.parents[key] = parent
                else:
                    self.state.parents.pop(key, None)
        elif kind == SAMPLE:
            if node >= 0:
                self.state.samples.add(node)
        elif kind == PHASE:
            if getattr(self.trace, "kind", None) == "discrete":
                self.state.phase = node if node >= 0 else self.state.phase + 1
                self.state.visited.clear()
                self.state.frontier.clear()
                self.state.samples.clear()
                self.state.parents.clear()
            else:
                phase_value = float(event["value"])
                self.state.phase = (
                    int(phase_value)
                    if math.isfinite(phase_value) and phase_value >= 0
                    else self.state.phase + 1
                )
        elif kind == SOLUTION:
            self.state.solution_node = node if node >= 0 else None
            if node >= 0:
                if (1, node) in self.state.parents or (2, node) in self.state.parents:
                    forward = self.state.path_to(node, 1)
                    backward = self.state.path_to(node, 2)
                    self.state.solution_path = tuple(reversed(forward)) + backward[1:]
                else:
                    self.state.solution_path = tuple(reversed(self.state.path_to(node, 0)))
        elif kind == PRUNE:
            self.state.frontier.discard(key)
            self.state.visited.discard(key)
            self.state.parents.pop(key, None)

    def _remember(self, position: int) -> None:
        if not self.max_cache_bytes or position in self._checkpoints:
            return
        snapshot = self.state.copy()
        size = snapshot.bytes_estimate()
        if size > self.max_cache_bytes:
            return
        while self._checkpoints and self._cache_bytes + size > self.max_cache_bytes:
            _, evicted = self._checkpoints.popitem(last=False)
            self._cache_bytes -= evicted.bytes_estimate()
        self._checkpoints[position] = snapshot
        self._cache_bytes += size

    def seek(self, position: int) -> ReplayState:
        target = max(0, min(self.length, int(position)))
        if target == self.position:
            return self.state
        if target < self.position or target - self.position > self.checkpoint_stride:
            cached = max((index for index in self._checkpoints if index <= target), default=0)
            self.state = self._checkpoints[cached].copy() if cached else ReplayState()
            self.position = cached
        while self.position < target:
            self._apply(self.events[self.position])
            self.position += 1
            if self.position % self.checkpoint_stride == 0:
                self._remember(self.position)
        return self.state

    def step(self, count: int = 1) -> ReplayState:
        return self.seek(self.position + count)


__all__ = [
    "DISCOVER",
    "EXPAND",
    "PARENT",
    "SAMPLE",
    "PHASE",
    "SOLUTION",
    "PRUNE",
    "ReplayController",
    "ReplayState",
]
