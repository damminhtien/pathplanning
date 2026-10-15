# Task Decisions: Performance audit

## D-1: Keep native static geometry exact

- Decision: check native-model line segments analytically against the declared
  static primitives; keep custom callback motion validation unchanged.
- Reason: sampled checks scale with edge length and can miss collisions when the
  sample interval is large. The declared model contains convex static shapes.

## D-2: Preserve direction for custom roadmap callbacks

- Decision: skip reverse validation only for a native static model. Continue
  checking both edge directions for custom callbacks.
- Reason: callback-defined motion may be directed even when roadmap distances
  are Euclidean.

## D-3: Keep diagnostics bounded

- Decision: use fixed BARN cases and a handful of repeats for before/after
  profiles; keep the existing compact test cohort.
- Reason: the goal is to verify each root-cause fix without repeating large
  benchmark campaigns during ordinary development.
