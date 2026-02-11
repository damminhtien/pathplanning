"""Priority queue utilities for 3D sampling-based planners."""

from __future__ import annotations

import heapq
import itertools
from typing import Any


class MinHeapPriorityQueue:
    """Heap-backed priority queue with explicit delete support."""

    def __init__(self) -> None:
        """Initialize an empty queue."""
        self._heap: list[list[Any]] = []
        self._nodes: set[Any] = set()
        self._entry_finder: dict[Any, list[Any]] = {}
        self._counter = itertools.count()
        self._removed_sentinel = "<removed-item>"

    def put(self, item: Any, priority: float) -> None:
        """Insert an item or update its priority."""
        if item in self._entry_finder:
            self.remove(item)
        entry = [priority, next(self._counter), item]
        self._entry_finder[item] = entry
        heapq.heappush(self._heap, entry)
        self._nodes.add(item)

    def put_many(self, priorities: dict[Any, float]) -> None:
        """Insert multiple ``item -> priority`` entries."""
        for item, priority in priorities.items():
            self.put(item, priority)

    def remove(self, item: Any) -> None:
        """Mark an item as removed if it is present."""
        if item not in self._entry_finder:
            return
        entry = self._entry_finder.pop(item)
        entry[-1] = self._removed_sentinel
        self._nodes.remove(item)

    def remove_many(self, items: set[Any]) -> None:
        """Mark all provided items as removed."""
        for item in items:
            self.remove(item)

    def filter_priorities(self, threshold: float, mode: str) -> None:
        """Remove items above/below a threshold.

        Args:
            threshold: Threshold to compare with entry priority.
            mode: ``\"lowpass\"`` removes entries with priority >= threshold.
                ``\"highpass\"`` removes entries with priority <= threshold.
        """
        for entry in self.entries():
            item = entry[2]
            if item == self._removed_sentinel:
                continue
            if mode == "lowpass" and entry[0] >= threshold:
                self.remove(item)
            elif mode == "highpass" and entry[0] <= threshold:
                self.remove(item)

    def get(self) -> Any:
        """Pop and return the lowest-priority item.

        Raises:
            KeyError: If the queue is empty.
        """
        while self._heap:
            _priority, _count, item = heapq.heappop(self._heap)
            if item != self._removed_sentinel:
                del self._entry_finder[item]
                self._nodes.remove(item)
                return item
        raise KeyError("pop from an empty priority queue")

    def top_key(self) -> float:
        """Return the minimum priority currently at the heap top."""
        return float(self._heap[0][0])

    def entries(self) -> list[list[Any]]:
        """Return raw heap entries."""
        return self._heap

    def nodes(self) -> set[Any]:
        """Return active nodes currently present in the queue."""
        return self._nodes
