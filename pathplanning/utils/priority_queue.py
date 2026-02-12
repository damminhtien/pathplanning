"""Deterministic queue and priority-queue utilities shared across planners."""

from __future__ import annotations

from collections import deque
from collections.abc import Hashable
import heapq
import itertools
from typing import Any, Generic, TypeVar, cast

T = TypeVar("T", bound=Hashable)


class QueueFIFO(Generic[T]):
    """First-in-first-out queue."""

    def __init__(self) -> None:
        self._queue: deque[T] = deque()

    def empty(self) -> bool:
        return len(self._queue) == 0

    def put(self, node: T) -> None:
        self._queue.append(node)

    def get(self) -> T:
        return self._queue.popleft()


class QueueLIFO(Generic[T]):
    """Last-in-first-out queue."""

    def __init__(self) -> None:
        self._queue: deque[T] = deque()

    def empty(self) -> bool:
        return len(self._queue) == 0

    def put(self, node: T) -> None:
        self._queue.append(node)

    def get(self) -> T:
        return self._queue.pop()


class QueuePrior(Generic[T]):
    """Deterministic heap priority queue with stable tie-breaking."""

    def __init__(self) -> None:
        self._heap: list[tuple[Any, int, T]] = []
        self._counter = itertools.count()

    def empty(self) -> bool:
        return len(self._heap) == 0

    def put(self, item: T, priority: Any) -> None:
        heapq.heappush(self._heap, (priority, next(self._counter), item))

    def get(self) -> T:
        return heapq.heappop(self._heap)[2]

    def enumerate(self) -> list[tuple[Any, T]]:
        return [(priority, item) for priority, _count, item in self._heap]

    def check_remove(self, item: T) -> None:
        for index, (_priority, _count, queued_item) in enumerate(self._heap):
            if queued_item == item:
                self._heap.pop(index)
                heapq.heapify(self._heap)
                return

    def top_key(self) -> Any:
        return self._heap[0][0]


class _RemovableMinHeap(Generic[T]):
    """Heap-backed priority queue with lazy deletion and stable tie-breaking."""

    def __init__(self) -> None:
        self._heap: list[list[Any]] = []
        self._nodes: set[T] = set()
        self._entry_finder: dict[T, list[Any]] = {}
        self._counter = itertools.count()
        self._removed_sentinel = "<removed-item>"

    def _discard_removed_head(self) -> None:
        while self._heap and self._heap[0][2] == self._removed_sentinel:
            heapq.heappop(self._heap)

    def empty(self) -> bool:
        return len(self._nodes) == 0

    def put(self, item: T, priority: Any) -> None:
        if item in self._entry_finder:
            self.remove(item)
        entry = [priority, next(self._counter), item]
        self._entry_finder[item] = entry
        heapq.heappush(self._heap, entry)
        self._nodes.add(item)

    def remove(self, item: T) -> None:
        if item not in self._entry_finder:
            return
        entry = self._entry_finder.pop(item)
        entry[2] = self._removed_sentinel
        self._nodes.remove(item)

    def get(self) -> T:
        while self._heap:
            _priority, _count, item = heapq.heappop(self._heap)
            if item != self._removed_sentinel:
                active_item = cast(T, item)
                del self._entry_finder[active_item]
                self._nodes.remove(active_item)
                return active_item
        raise KeyError("pop from an empty priority queue")

    def top_key(self) -> Any:
        self._discard_removed_head()
        if not self._heap:
            raise KeyError("pop from an empty priority queue")
        return self._heap[0][0]

    def entries(self) -> list[list[Any]]:
        return self._heap

    def nodes(self) -> set[T]:
        return self._nodes


class MinheapPQ(_RemovableMinHeap[T]):
    """Search-planner min-heap priority queue."""

    def check_remove(self, item: T) -> None:
        self.remove(item)

    def enumerate(self) -> list[list[Any]]:
        return [entry for entry in self._heap if entry[2] != self._removed_sentinel]

    def allnodes(self) -> set[T]:
        return self.nodes()


class MinHeapPriorityQueue(_RemovableMinHeap[T]):
    """Sampling-planner min-heap priority queue."""

    def put_many(self, priorities: dict[T, Any]) -> None:
        for item, priority in priorities.items():
            self.put(item, priority)

    def remove_many(self, items: set[T]) -> None:
        for item in items:
            self.remove(item)

    def filter_priorities(self, threshold: float, mode: str) -> None:
        for entry in list(self.entries()):
            item = entry[2]
            if item == self._removed_sentinel:
                continue
            if mode == "lowpass" and entry[0] >= threshold:
                self.remove(cast(T, item))
            elif mode == "highpass" and entry[0] <= threshold:
                self.remove(cast(T, item))


__all__ = [
    "QueueFIFO",
    "QueueLIFO",
    "QueuePrior",
    "MinheapPQ",
    "MinHeapPriorityQueue",
]
