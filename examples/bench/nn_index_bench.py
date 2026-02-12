"""Micro-benchmark for nearest-neighbor indices."""

from __future__ import annotations

import time

import numpy as np

from pathplanning.nn.index import KDTreeIndex, NaiveIndex


def _bench(index, queries, repeats: int) -> float:
    start = time.perf_counter()
    for _ in range(repeats):
        for q in queries:
            index.nearest(q)
    return time.perf_counter() - start


def main() -> None:
    rng = np.random.default_rng(7)
    points = rng.uniform(-10.0, 10.0, size=(2000, 3))
    queries = rng.uniform(-10.0, 10.0, size=(200, 3))

    naive = NaiveIndex(dim=3)
    for p in points:
        naive.add(p)

    naive_time = _bench(naive, queries, repeats=3)
    print(f"NaiveIndex: {naive_time:.4f}s")

    try:
        kdtree = KDTreeIndex(dim=3)
    except ImportError:
        print("KDTreeIndex: scipy not installed, skipping")
        return

    for p in points:
        kdtree.add(p)

    kdtree_time = _bench(kdtree, queries, repeats=3)
    print(f"KDTreeIndex: {kdtree_time:.4f}s")


if __name__ == "__main__":
    main()
