"""Core contracts used by planners and reusable planning APIs."""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass
from typing import (
    TYPE_CHECKING,
    Any,
    Generic,
    Protocol,
    TypeAlias,
    TypeVar,
    cast,
    runtime_checkable,
)

from pathplanning.core.types import RNG, Float, N, S, Vec

if TYPE_CHECKING:
    from pathplanning.native.graph import NativeGraph

State: TypeAlias = Vec
N_contra = TypeVar("N_contra", contravariant=True)
S_contra = TypeVar("S_contra", contravariant=True)


# ---------------------------------------------------------------------------
# Discrete / graph search contracts
# ---------------------------------------------------------------------------
@runtime_checkable
class DiscreteGraph(Protocol[N]):
    """Minimal graph contract for discrete search-based planners."""

    def neighbors(self, n: N) -> Iterable[N]:
        """Return outward neighbors for ``n``."""
        ...

    def edge_cost(self, a: N, b: N) -> Float:
        """Return directed edge cost from ``a`` to ``b``."""
        ...


@runtime_checkable
class HeuristicDiscreteGraph(DiscreteGraph[N], Protocol[N]):
    """Optional heuristic extension for informed discrete planners."""

    def heuristic(self, n: N, goal: N) -> Float:
        """Return admissible/consistent heuristic estimate from ``n`` to ``goal``."""
        ...


@runtime_checkable
class ValidatingDiscreteGraph(DiscreteGraph[N], Protocol[N]):
    """Optional node-validity extension for graph planners."""

    def is_valid_node(self, n: N) -> bool:
        """Return ``True`` when ``n`` is valid/searchable."""
        ...


@runtime_checkable
class GoalTest(Protocol[N_contra]):
    """Goal predicate for discrete problems with non-singleton goal sets."""

    def is_goal(self, n: N_contra) -> bool:
        """Return ``True`` when node ``n`` satisfies the goal condition."""
        ...


DiscreteParams: TypeAlias = Mapping[str, object]
TimeInterval: TypeAlias = tuple[Float, Float]


@dataclass(frozen=True, slots=True)
class ExactGoalTest(Generic[N]):
    """Adapter that turns one exact goal node into a ``GoalTest``."""

    goal_node: N

    def is_goal(self, n: N) -> bool:
        return n == self.goal_node


@dataclass(slots=True)
class DiscreteProblem(Generic[N]):
    """Discrete planning problem wrapper used by graph-search planners."""

    graph: DiscreteGraph[N] | NativeGraph[N]
    start: N
    goal: N | GoalTest[N]
    params: DiscreteParams | None = None

    def resolve_goal_test(self) -> GoalTest[N]:
        """Return a goal-test adapter regardless of goal representation."""
        goal = self.goal
        if hasattr(goal, "is_goal"):
            return cast(GoalTest[N], goal)
        return ExactGoalTest(cast(N, goal))


@dataclass(slots=True)
class TemporalProblem(Generic[N]):
    """Graph planning problem with time-varying node and edge obstacles.

    Blocked intervals use the half-open convention ``[start, end)``. Edge
    durations default to the graph's edge costs when not supplied explicitly.
    """

    graph: DiscreteGraph[N] | NativeGraph[N]
    start: N
    goal: N
    node_blocked: Mapping[N, Sequence[TimeInterval]] | None = None
    edge_blocked: Mapping[tuple[N, N], Sequence[TimeInterval]] | None = None
    edge_durations: Mapping[tuple[N, N], Float] | None = None
    start_time: Float = 0.0
    horizon: Float | None = None
    params: DiscreteParams | None = None


# ---------------------------------------------------------------------------
# Continuous / sampling contracts
# ---------------------------------------------------------------------------
@runtime_checkable
class ContinuousSpace(Protocol[S]):
    """Minimal continuous-space contract for sampling-based planners."""

    def sample_free(self, rng: RNG) -> S:
        """Sample one valid free state from the space."""
        ...

    def is_state_valid(self, x: S) -> bool:
        """Return ``True`` when state ``x`` is valid."""
        ...

    def is_motion_valid(self, a: S, b: S) -> bool:
        """Return ``True`` when local motion from ``a`` to ``b`` is valid."""
        ...

    def distance(self, a: S, b: S) -> Float:
        """Return metric distance between two states."""
        ...

    def steer(self, a: S, b: S, step_size: Float) -> S:
        """Steer from ``a`` toward ``b`` with bounded step size."""
        ...


@runtime_checkable
class InterpolatingContinuousSpace(ContinuousSpace[S], Protocol[S]):
    """Optional interpolation extension for richer local planning behavior."""

    def interpolate(self, a: S, b: S, t: Float) -> S:
        """Interpolate state between ``a`` and ``b`` at ``t in [0, 1]``."""
        ...


@runtime_checkable
class ContinuousSpaceMetadata(Protocol):
    """Optional metadata extension for bounds/dimension aware planners."""

    @property
    def bounds(self) -> object:
        """Space bounds representation (implementation-defined)."""
        ...

    @property
    def dimension(self) -> int:
        """Topological dimension of the space."""
        ...


@runtime_checkable
class SupportsBatchMotionCheck(Protocol[S]):
    """Optional extension for batched local-motion validity checks."""

    def is_motion_valid_batch(self, edges: list[tuple[S, S]]) -> list[bool]:
        """Return validity flags for each edge in ``edges``."""
        ...


@runtime_checkable
class SupportsCollisionStepMotionCheck(Protocol[S_contra]):
    """Optional motion-check extension that accepts a planner-local step size."""

    def is_motion_valid_with_step(
        self,
        a: S_contra,
        b: S_contra,
        collision_step: Float,
    ) -> bool:
        """Return motion validity using ``collision_step`` without mutating the space."""
        ...


@runtime_checkable
class SupportsBatchCollisionStepMotionCheck(Protocol[S_contra]):
    """Optional batched motion-check extension that accepts a local step size."""

    def is_motion_valid_batch_with_step(
        self,
        edges: Sequence[tuple[S_contra, S_contra]],
        collision_step: Float,
    ) -> list[bool]:
        """Return validity flags using ``collision_step`` without mutating the space."""
        ...


@runtime_checkable
class GoalRegion(Protocol[S_contra]):
    """Goal-region predicate for continuous problems."""

    def contains(self, x: S_contra) -> bool:
        """Return ``True`` when ``x`` lies in the goal region."""
        ...


@dataclass(frozen=True, slots=True)
class GoalState(Generic[S]):
    """Concrete goal-state region with optional radius and distance function."""

    state: S
    radius: Float = 0.0
    distance_fn: Callable[[S, S], Float] | None = None

    def contains(self, x: S) -> bool:
        if self.distance_fn is None:
            equal = x == self.state
            try:
                return bool(equal)
            except ValueError:
                return bool(cast(Any, equal).all())
        return self.distance_fn(x, self.state) <= self.radius

    def distance_to_goal(self, x: S) -> Float:
        if self.distance_fn is None:
            return 0.0 if self.contains(x) else float("inf")
        return self.distance_fn(x, self.state)


@runtime_checkable
class DistanceAwareGoalRegion(GoalRegion[S_contra], Protocol[S_contra]):
    """Optional goal-region extension that exposes goal distance estimates."""

    def distance_to_goal(self, x: S_contra) -> Float:
        """Return lower-bound distance from ``x`` to the goal region."""
        ...


@runtime_checkable
class Objective(Protocol[S]):
    """Optional path objective; sampling optimizers select paths with lower values."""

    def path_cost(self, path: Sequence[S], space: ContinuousSpace[S]) -> Float:
        """Return objective value for ``path`` under ``space``."""
        ...


ContinuousParams: TypeAlias = Mapping[str, object]


@dataclass(slots=True)
class ContinuousProblem(Generic[S]):
    """Continuous planning problem wrapper used by sampling planners."""

    space: ContinuousSpace[S]
    start: S
    goal: GoalRegion[S]
    objective: Objective[S] | None = None
    params: ContinuousParams | None = None
