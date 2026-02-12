from __future__ import annotations

import pytest

from pathplanning.utils.priority_queue import MinHeapPriorityQueue


def test_remove_marks_item_and_get_skips_it() -> None:
    queue = MinHeapPriorityQueue()
    queue.put("keep", 2)
    queue.put("drop", 1)

    queue.remove("drop")

    assert "drop" not in queue.nodes()
    assert queue.get() == "keep"
    with pytest.raises(KeyError):
        queue.get()


def test_put_many_accepts_dict_and_preserves_priority_order() -> None:
    queue = MinHeapPriorityQueue()
    queue.put_many({"c": 3, "a": 1, "b": 2})

    assert [queue.get(), queue.get(), queue.get()] == ["a", "b", "c"]
