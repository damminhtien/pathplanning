import {
  createProjection,
  drawGrid,
  drawLabel,
  drawPath,
  drawPoint,
  drawVoxel,
  pointAlongPolyline,
  shuttleProgress,
} from "./spatial-3d-shared.js";

const VOXEL_VOLUME = { x: 9, y: 7, z: 4 };
const VOXEL_START = [1, 3, 0];
const VOXEL_GOAL = [7, 3, 0];
const VOXEL_WALL = new Set(
  Array.from({ length: 7 }, (_, y) => [4, y, 0])
    .concat(Array.from({ length: 7 }, (_, y) => [4, y, 1]))
    .concat(Array.from({ length: 7 }, (_, y) => [4, y, 2]))
    .map(([x, y, z]) => `${x},${y},${z}`),
);

function keyOf(point) {
  return point.join(",");
}

function distance(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function isBlockedVoxel(point) {
  return VOXEL_WALL.has(keyOf(point));
}

function canTraverseVoxel(from, to) {
  const axes = [];
  for (let axis = 0; axis < 3; axis += 1) {
    if (from[axis] !== to[axis]) axes.push(axis);
  }

  const fullMask = (1 << axes.length) - 1;
  for (let mask = 1; mask < fullMask; mask += 1) {
    const corner = [...from];
    axes.forEach((axis, index) => {
      if (mask & (1 << index)) corner[axis] = to[axis];
    });
    if (isBlockedVoxel(corner)) return false;
  }
  return true;
}

function planVoxelSearch() {
  const moves = [];
  for (let dx = -1; dx <= 1; dx += 1) {
    for (let dy = -1; dy <= 1; dy += 1) {
      for (let dz = -1; dz <= 1; dz += 1) {
        if (dx || dy || dz) moves.push([dx, dy, dz]);
      }
    }
  }

  const startKey = keyOf(VOXEL_START);
  const goalKey = keyOf(VOXEL_GOAL);
  const open = [startKey];
  const openKeys = new Set(open);
  const closed = new Set();
  const costs = new Map([[startKey, 0]]);
  const parents = new Map();
  const expanded = [];
  let found = false;

  while (open.length) {
    let bestIndex = 0;
    let bestKey = open[0];
    let bestPoint = bestKey.split(",").map(Number);
    let bestEstimate = costs.get(bestKey) + distance(bestPoint, VOXEL_GOAL);
    for (let index = 1; index < open.length; index += 1) {
      const candidateKey = open[index];
      const candidatePoint = candidateKey.split(",").map(Number);
      const estimate = costs.get(candidateKey) + distance(candidatePoint, VOXEL_GOAL);
      if (estimate < bestEstimate) {
        bestIndex = index;
        bestKey = candidateKey;
        bestPoint = candidatePoint;
        bestEstimate = estimate;
      }
    }

    open.splice(bestIndex, 1);
    openKeys.delete(bestKey);
    if (closed.has(bestKey)) continue;
    closed.add(bestKey);
    expanded.push(bestPoint);
    if (bestKey === goalKey) {
      found = true;
      break;
    }

    for (const move of moves) {
      const next = bestPoint.map((value, axis) => value + move[axis]);
      if (
        next.some((value, axis) => value < 0 || value >= VOXEL_VOLUME["xyz"[axis]]) ||
        isBlockedVoxel(next) ||
        !canTraverseVoxel(bestPoint, next)
      )
        continue;

      const nextKey = keyOf(next);
      if (closed.has(nextKey)) continue;
      const candidateCost = costs.get(bestKey) + distance(bestPoint, next);
      if (candidateCost >= (costs.get(nextKey) ?? Number.POSITIVE_INFINITY)) continue;
      costs.set(nextKey, candidateCost);
      parents.set(nextKey, bestKey);
      if (!openKeys.has(nextKey)) {
        open.push(nextKey);
        openKeys.add(nextKey);
      }
    }
  }

  const route = [];
  if (found) {
    let key = goalKey;
    while (key) {
      route.push(key.split(",").map(Number));
      key = parents.get(key);
    }
    route.reverse();
  }
  return { expanded, route };
}

function drawVoxelScene(context, width, height, phase, elapsed, search) {
  const projection = createProjection(width, height, VOXEL_VOLUME);
  drawGrid(context, projection, VOXEL_VOLUME);

  const obstacleCells = [...VOXEL_WALL].map((key) => key.split(",").map(Number));
  obstacleCells.sort((a, b) => a[0] + a[1] + a[2] - b[0] - b[1] - b[2]);
  obstacleCells.forEach(([x, y, z]) => {
    drawVoxel(context, projection, x, y, z);
  });

  const visibleNodes = Math.ceil(search.expanded.length * Math.min(phase / 0.62, 1));
  for (const [x, y, z] of search.expanded.slice(0, visibleNodes)) {
    if (isBlockedVoxel([x, y, z])) continue;
    const point = projection.point(x + 0.5, y + 0.5, z + 0.5);
    context.beginPath();
    context.arc(point.x, point.y, 2.2, 0, Math.PI * 2);
    context.fillStyle = "rgba(65, 130, 190, .56)";
    context.fill();
  }

  const pathProgress = phase < 0.56 ? 0 : Math.min((phase - 0.56) / 0.24, 1);
  const routePoints = search.route.map(([x, y, z]) => [x + 0.5, y + 0.5, z + 0.5]);
  const revealedAgent = drawPath(context, projection, routePoints, pathProgress, "#2d78b8");
  const agent =
    pathProgress < 1
      ? revealedAgent
      : pointAlongPolyline(projection, routePoints, shuttleProgress(elapsed));
  drawPoint(context, projection.point(...VOXEL_START.map((value) => value + 0.5)), "#2d78b8", 4.2);
  drawPoint(context, projection.point(...VOXEL_GOAL.map((value) => value + 0.5)), "#c78643", 4.2);
  if (agent && pathProgress > 0) drawPoint(context, agent, "#216aa7", 4.1);

  drawLabel(context, `EXPANDED  ${visibleNodes} / ${search.expanded.length}`, 16, height - 15);
  drawLabel(context, "X", width - 20, height - 18);
}

export function createVoxelScene() {
  const search = planVoxelSearch();
  return (context, width, height, phase, elapsed) => {
    drawVoxelScene(context, width, height, phase, elapsed, search);
  };
}
