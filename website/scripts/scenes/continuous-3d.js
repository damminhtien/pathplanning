"use strict";

import {
  createProjection,
  drawLabel,
  drawPath,
  drawPoint,
  drawPrism,
  drawVolumeFrame,
  pointAlongPolyline,
  shuttleProgress,
} from "./spatial-3d-shared.js";

const CONTINUOUS_VOLUME = { x: 10, y: 8, z: 4 };
const CONTINUOUS_START = [1, 1, 1];
const CONTINUOUS_GOAL = [9, 7, 1];
const CONTINUOUS_OBSTACLE = { min: [4, 2, 0], max: [6, 5.2, 3.15] };
const CONTINUOUS_ROUTE = [
  CONTINUOUS_START,
  [2.4, 1.8, 1.2],
  [3.3, 1.8, 2.8],
  [3.5, 5.5, 3.7],
  [5.8, 6, 3.6],
  [7.2, 6.7, 2.6],
  CONTINUOUS_GOAL,
];

function distance(a, b) {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

function makeRandom(seed) {
  let state = seed >>> 0;
  return () => {
    state = (Math.imul(state, 1664525) + 1013904223) >>> 0;
    return state / 4294967296;
  };
}

function pointBlocked(point) {
  return point.some((value, axis) => value < 0 || value > CONTINUOUS_VOLUME["xyz"[axis]])
    || point.every((value, axis) => value >= CONTINUOUS_OBSTACLE.min[axis]
      && value <= CONTINUOUS_OBSTACLE.max[axis]);
}

function segmentIsClear(from, to) {
  const steps = Math.ceil(distance(from, to) / 0.12);
  for (let step = 1; step < steps; step += 1) {
    const fraction = step / steps;
    const point = from.map((value, axis) => value + (to[axis] - value) * fraction);
    if (pointBlocked(point)) return false;
  }
  return true;
}

function steer(from, to, maxDistance) {
  const span = distance(from, to);
  if (span <= maxDistance) return [...to];
  const scale = maxDistance / span;
  return from.map((value, axis) => value + (to[axis] - value) * scale);
}

function isAncestor(nodes, ancestor, child) {
  let index = child;
  while (index >= 0) {
    if (index === ancestor) return true;
    index = nodes[index].parent;
  }
  return false;
}

function updateDescendantCosts(nodes, root) {
  const children = new Map();
  for (let index = 1; index < nodes.length; index += 1) {
    const parent = nodes[index].parent;
    if (!children.has(parent)) children.set(parent, []);
    children.get(parent).push(index);
  }
  const pending = [...(children.get(root) ?? [])];
  while (pending.length) {
    const child = pending.pop();
    const parent = nodes[child].parent;
    nodes[child].cost = nodes[parent].cost + distance(nodes[parent].point, nodes[child].point);
    pending.push(...(children.get(child) ?? []));
  }
}

function buildContinuousTree() {
  const random = makeRandom(90421);
  const nodes = [{ point: CONTINUOUS_START, parent: -1, parentAtBirth: -1, cost: 0, birth: 0 }];
  const rewires = [];
  const iterations = 150;
  const waypointTargets = CONTINUOUS_ROUTE.slice(1, -1);

  for (let iteration = 1; iteration <= iterations; iteration += 1) {
    let target;
    if (iteration % 17 === 0) {
      target = waypointTargets[(Math.floor(iteration / 17) - 1) % waypointTargets.length];
    } else if (random() < 0.08) {
      target = CONTINUOUS_GOAL;
    } else {
      target = [
        random() * CONTINUOUS_VOLUME.x,
        random() * CONTINUOUS_VOLUME.y,
        random() * CONTINUOUS_VOLUME.z,
      ];
    }

    let nearest = 0;
    let nearestDistance = distance(nodes[0].point, target);
    for (let index = 1; index < nodes.length; index += 1) {
      const candidateDistance = distance(nodes[index].point, target);
      if (candidateDistance < nearestDistance) {
        nearest = index;
        nearestDistance = candidateDistance;
      }
    }

    const point = steer(nodes[nearest].point, target, 1.05);
    if (pointBlocked(point) || !segmentIsClear(nodes[nearest].point, point)) continue;

    let parent = nearest;
    let parentCost = nodes[nearest].cost + distance(nodes[nearest].point, point);
    for (let index = 0; index < nodes.length; index += 1) {
      const span = distance(nodes[index].point, point);
      if (span > 2.1 || nodes[index].cost + span >= parentCost) continue;
      if (segmentIsClear(nodes[index].point, point)) {
        parent = index;
        parentCost = nodes[index].cost + span;
      }
    }

    const nodeIndex = nodes.length;
    nodes.push({ point, parent, parentAtBirth: parent, cost: parentCost, birth: iteration });
    for (let index = 1; index < nodeIndex; index += 1) {
      const span = distance(nodes[index].point, point);
      const rewiredCost = parentCost + span;
      if (
        span > 1.8
        || rewiredCost >= nodes[index].cost
        || isAncestor(nodes, index, parent)
        || !segmentIsClear(point, nodes[index].point)
      ) continue;
      const previousParent = nodes[index].parent;
      nodes[index].parent = nodeIndex;
      nodes[index].cost = rewiredCost;
      rewires.push({ iteration, child: index, previousParent, nextParent: nodeIndex });
      updateDescendantCosts(nodes, index);
    }
  }

  let parent = 0;
  CONTINUOUS_ROUTE.slice(1).forEach((point, index) => {
    const previous = nodes[parent].point;
    if (!segmentIsClear(previous, point)) return;
    const nodeIndex = nodes.length;
    nodes.push({
      point,
      parent,
      parentAtBirth: parent,
      cost: nodes[parent].cost + distance(previous, point),
      birth: iterations + (index + 1) * 4,
      route: true,
    });
    parent = nodeIndex;
  });

  return { nodes, rewires, totalIterations: iterations + CONTINUOUS_ROUTE.length * 4 };
}

function parentAtIteration(tree, nodeIndex, iteration) {
  let parent = tree.nodes[nodeIndex].parentAtBirth;
  for (const event of tree.rewires) {
    if (event.iteration > iteration) break;
    if (event.child === nodeIndex) parent = event.nextParent;
  }
  return parent;
}

function drawContinuousScene(context, width, height, phase, elapsed, tree) {
  const projection = createProjection(width, height, CONTINUOUS_VOLUME);
  drawVolumeFrame(context, projection, CONTINUOUS_VOLUME);
  const iteration = Math.floor(phase * tree.totalIterations);
  const firstHiddenIndex = tree.nodes.findIndex((node) => node.birth > iteration);
  const visibleCount = firstHiddenIndex < 0 ? tree.nodes.length : firstHiddenIndex;

  context.save();
  context.lineWidth = 1.1;
  context.strokeStyle = "rgba(59, 127, 183, .28)";
  for (let nodeIndex = 0; nodeIndex < visibleCount; nodeIndex += 1) {
    const node = tree.nodes[nodeIndex];
    const parent = parentAtIteration(tree, nodeIndex, iteration);
    if (parent < 0 || parent >= visibleCount) continue;
    const from = projection.point(...tree.nodes[parent].point);
    const to = projection.point(...node.point);
    context.beginPath();
    context.moveTo(from.x, from.y);
    context.lineTo(to.x, to.y);
    context.stroke();
  }

  for (let index = 0; index < visibleCount; index += 1) {
    const node = tree.nodes[index];
    const point = projection.point(...node.point);
    context.beginPath();
    context.arc(point.x, point.y, node.route ? 2.5 : 1.65, 0, Math.PI * 2);
    context.fillStyle = node.route ? "rgba(55, 123, 183, .7)" : "rgba(69, 133, 188, .49)";
    context.fill();
    if (index === 0) drawPoint(context, point, "#2d78b8", 4);
  }
  context.restore();

  drawPrism(context, projection, CONTINUOUS_OBSTACLE);

  let recentRewire = null;
  for (const event of tree.rewires) {
    if (event.iteration > iteration) break;
    recentRewire = event;
  }
  if (recentRewire && phase > 0.25 && phase < 0.78) {
    const from = projection.point(...tree.nodes[recentRewire.nextParent].point);
    const to = projection.point(...tree.nodes[recentRewire.child].point);
    context.save();
    context.setLineDash([4, 3]);
    context.lineWidth = 2;
    context.strokeStyle = "rgba(51, 132, 190, .86)";
    context.beginPath();
    context.moveTo(from.x, from.y);
    context.lineTo(to.x, to.y);
    context.stroke();
    context.restore();
    drawLabel(context, "REWIRE", (from.x + to.x) / 2 + 4, (from.y + to.y) / 2 - 4, "#2d78b8");
  }

  const routeProgress = phase < 0.71 ? 0 : Math.min((phase - 0.71) / 0.29, 1);
  const route = CONTINUOUS_ROUTE;
  const revealedAgent = drawPath(context, projection, route, routeProgress, "#2d78b8");
  const agent = routeProgress < 1
    ? revealedAgent
    : pointAlongPolyline(projection, route, shuttleProgress(elapsed));
  drawPoint(context, projection.point(...CONTINUOUS_START), "#2d78b8", 4.2);
  drawPoint(context, projection.point(...CONTINUOUS_GOAL), "#c78643", 4.2);
  if (agent && routeProgress > 0) drawPoint(context, agent, "#216aa7", 4.1);

  const rewireCount = tree.rewires.findIndex((event) => event.iteration > iteration);
  drawLabel(context, `SAMPLES  ${visibleCount - 1}   /   REWIRES  ${rewireCount < 0 ? tree.rewires.length : rewireCount}`, 16, height - 15);
}

export function createContinuousScene() {
  const tree = buildContinuousTree();
  return (context, width, height, phase, elapsed) => {
    drawContinuousScene(context, width, height, phase, elapsed, tree);
  };
}
