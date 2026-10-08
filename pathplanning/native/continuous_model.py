"""Declarative continuous-space data accepted by the native C planner."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from numpy.typing import ArrayLike


@dataclass(frozen=True, slots=True)
class NativeContinuousSpaceModel:
    """Euclidean bounds and obstacle arrays for callback-free native planning.

    Sampling is uniform within the bounds with rejection against the listed
    obstacle primitives. Boxes and spheres use shape ``(count, dimension)``;
    OBB centers/extents use ``(count, 3)`` and orientations use ``(count, 9)``
    row-major matrices. Empty obstacle arrays may be omitted.
    """

    lower_bounds: ArrayLike
    upper_bounds: ArrayLike
    box_minima: ArrayLike = ()
    box_maxima: ArrayLike = ()
    sphere_centers: ArrayLike = ()
    sphere_radii: ArrayLike = ()
    obb_centers: ArrayLike = ()
    obb_extents: ArrayLike = ()
    obb_orientations: ArrayLike = ()


class NativeContinuousSpaceProvider(Protocol):
    """Custom spaces can expose a native model through this setup-time method."""

    def to_native_model(self) -> NativeContinuousSpaceModel:
        """Return data whose semantics match uniform Euclidean native planning."""
        ...


@dataclass(frozen=True, slots=True)
class NativeAckermannGridModel:
    """Occupancy and vehicle geometry consumed by native SE(2) search."""

    occupancy: ArrayLike
    resolution: float
    origin: ArrayLike


__all__ = [
    "NativeContinuousSpaceModel",
    "NativeContinuousSpaceProvider",
    "NativeAckermannGridModel",
]
