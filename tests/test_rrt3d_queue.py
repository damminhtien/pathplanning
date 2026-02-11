from __future__ import annotations

import pytest

from pathplanning.sampling_based_planning.rrt_3d.queue import MinheapPQ


def test_check_remove_marks_item_and_get_skips_it() -> None:
    queue = MinheapPQ()
    queue.put("keep", 2)
    queue.put("drop", 1)

    queue.check_remove("drop")

    assert "drop" not in queue.allnodes()
    assert queue.get() == "keep"
    with pytest.raises(KeyError):
        queue.get()


def test_put_set_accepts_dict_and_preserves_priority_order() -> None:
    queue = MinheapPQ()
    queue.put_set({"c": 3, "a": 1, "b": 2})

    assert [queue.get(), queue.get(), queue.get()] == ["a", "b", "c"]
