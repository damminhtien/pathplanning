import {
  distance,
  drawTinyLabel as drawCanvasLabel,
  drawMotionMarker,
  drawPathTrace,
  roundedRect as drawRoundedRect,
  pathAtProgress,
} from "./shared.js";

const REFINEMENT_PLANNERS = new Set([
  "rrt_star",
  "informed_rrt_star",
  "bit_star",
  "abit_star",
  "rit_star",
]);

function segmentHitsRect(a, b, rect) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  let entry = 0;
  let exit = 1;

  for (const [direction, offset] of [
    [-dx, a.x - rect.x],
    [dx, rect.x + rect.w - a.x],
    [-dy, a.y - rect.y],
    [dy, rect.y + rect.h - a.y],
  ]) {
    if (direction === 0) {
      if (offset < 0) return false;
      continue;
    }

    const intersection = offset / direction;
    if (direction < 0) entry = Math.max(entry, intersection);
    else exit = Math.min(exit, intersection);
    if (entry > exit) return false;
  }

  return true;
}

function findCollisionFreePath(start, goal, obstacles, clearance) {
  const safetyObstacles = obstacles.map((rect) => ({
    x: rect.x - clearance + 1,
    y: rect.y - clearance + 1,
    w: rect.w + clearance * 2 - 2,
    h: rect.h + clearance * 2 - 2,
  }));
  const nodes = [start, goal];

  for (const rect of obstacles) {
    const left = rect.x - clearance;
    const right = rect.x + rect.w + clearance;
    const top = rect.y - clearance;
    const bottom = rect.y + rect.h + clearance;
    nodes.push(
      { x: left, y: top },
      { x: right, y: top },
      { x: right, y: bottom },
      { x: left, y: bottom },
    );
  }

  const costs = Array(nodes.length).fill(Infinity);
  const parents = Array(nodes.length).fill(-1);
  const visited = new Set();
  costs[0] = 0;

  while (visited.size < nodes.length) {
    let current = -1;
    for (let index = 0; index < nodes.length; index += 1) {
      if (!visited.has(index) && (current < 0 || costs[index] < costs[current])) current = index;
    }
    if (current < 0 || !Number.isFinite(costs[current])) break;
    if (current === 1) break;
    visited.add(current);

    for (let next = 0; next < nodes.length; next += 1) {
      if (visited.has(next) || current === next) continue;
      if (safetyObstacles.some((rect) => segmentHitsRect(nodes[current], nodes[next], rect)))
        continue;
      const cost = costs[current] + distance(nodes[current], nodes[next]);
      if (cost < costs[next]) {
        costs[next] = cost;
        parents[next] = current;
      }
    }
  }

  if (!Number.isFinite(costs[1])) return [];
  const path = [];
  for (let index = 1; index >= 0; index = parents[index]) {
    path.push(nodes[index]);
    if (index === 0) break;
  }
  return path.reverse();
}

export function createContinuousScene({ context, state, planners }) {
  const roundedRect = (x, y, width, height, radius) =>
    drawRoundedRect(context, x, y, width, height, radius);
  const drawTinyLabel = (text, x, y, color) => drawCanvasLabel(context, text, x, y, color);

  function seededRandom(seed) {
    let value = seed >>> 0;
    return () => {
      value = (value * 1664525 + 1013904223) >>> 0;
      return value / 4294967296;
    };
  }

  function rectsForWorld(width, height) {
    return [
      { x: width * 0.36, y: height * 0.22, w: width * 0.16, h: height * 0.48 },
      { x: width * 0.64, y: height * 0.48, w: width * 0.13, h: height * 0.3 },
      { x: width * 0.78, y: height * 0.12, w: width * 0.07, h: height * 0.2 },
    ];
  }

  function drawContinuous(width, height) {
    const planner = state.planner;
    const obstacles = rectsForWorld(width, height);
    const random = seededRandom(state.seed + planners.indexOf(planner) * 17);
    const samples = Array.from({ length: 82 }, () => ({
      x: width * 0.07 + random() * width * 0.86,
      y: height * 0.13 + random() * height * 0.74,
    }));
    for (let x = 24; x < width; x += 30) {
      context.beginPath();
      context.moveTo(x, 0);
      context.lineTo(x, height);
      context.strokeStyle = "rgba(170, 194, 223, .035)";
      context.stroke();
    }
    for (let y = 23; y < height; y += 30) {
      context.beginPath();
      context.moveTo(0, y);
      context.lineTo(width, y);
      context.strokeStyle = "rgba(170, 194, 223, .035)";
      context.stroke();
    }
    const start = { x: width * 0.07, y: height * 0.76 };
    const goal = { x: width * 0.92, y: height * 0.22 };
    const clearance = Math.max(14, Math.min(width, height) * 0.035);
    const route = findCollisionFreePath(start, goal, obstacles, clearance);
    const roomyRoute = REFINEMENT_PLANNERS.has(planner.id)
      ? findCollisionFreePath(start, goal, obstacles, clearance * 1.8)
      : route;
    const informed =
      ["informed_rrt_star", "bit_star", "abit_star", "fcit_star", "rit_star"].includes(
        planner.id,
      ) && state.progress > 0.43;
    if (informed) {
      context.save();
      context.translate((start.x + goal.x) / 2, (start.y + goal.y) / 2);
      context.rotate(-0.54);
      context.beginPath();
      context.ellipse(
        0,
        0,
        width * (0.36 - Math.max(0, state.progress - 0.45) * 0.14),
        height * 0.13,
        0,
        0,
        Math.PI * 2,
      );
      context.fillStyle = "rgba(131, 240, 205, .045)";
      context.fill();
      context.setLineDash([5, 6]);
      context.strokeStyle = "rgba(131, 240, 205, .32)";
      context.lineWidth = 1.2;
      context.stroke();
      context.restore();
    }
    const count = Math.floor(samples.length * Math.min(1, state.progress * 1.1));
    const trees = [
      "rrt",
      "rrt_star",
      "informed_rrt_star",
      "rrt_connect",
      "eirm_star",
      "rit_star",
      "jit_star",
    ].includes(planner.id);
    const twoTrees = planner.id === "rrt_connect";
    const roadmap = !trees || ["eirm_star", "rit_star"].includes(planner.id);
    context.save();
    if (roadmap) {
      const maxEdges = Math.min(count * 4, 190);
      let drawn = 0;
      for (let i = 0; i < count && drawn < maxEdges; i += 1) {
        const from = samples[i];
        const nearest = samples
          .slice(0, i)
          .map((point, index) => ({ point, index, d: distance(from, point) }))
          .sort((a, b) => a.d - b.d)
          .slice(0, planner.id === "fcit_star" ? 8 : 4);
        for (const item of nearest) {
          if (drawn >= maxEdges) break;
          const crosses = obstacles.some((rect) => segmentHitsRect(from, item.point, rect));
          const lazy = planner.id === "lazy_prm" || planner.id === "eirm_star";
          context.beginPath();
          context.moveTo(from.x, from.y);
          context.lineTo(item.point.x, item.point.y);
          context.lineWidth = 0.8;
          context.strokeStyle = crosses
            ? "rgba(255, 125, 141, .3)"
            : lazy
              ? "rgba(139, 188, 255, .28)"
              : "rgba(139, 188, 255, .2)";
          if (lazy && !crosses && drawn > maxEdges * 0.78) context.setLineDash([3, 5]);
          else context.setLineDash([]);
          context.stroke();
          drawn += 1;
        }
      }
    } else {
      const treeCount = Math.min(count, 57);
      const leftCount = twoTrees ? Math.ceil(treeCount / 2) : treeCount;
      const rightCount = twoTrees ? Math.max(0, treeCount - leftCount) : 0;
      for (let i = 0; i < leftCount; i += 1) {
        const node = samples[i];
        const pool = i ? samples.slice(0, i) : [start];
        const nearest = pool.reduce(
          (best, point) => (distance(point, node) < distance(best, node) ? point : best),
          pool[0],
        );
        drawEdge(
          nearest,
          node,
          obstacles,
          i % 5 === 0 ? "rgba(131, 240, 205, .46)" : "rgba(130, 169, 208, .23)",
        );
      }
      if (twoTrees) {
        for (let i = 0; i < rightCount; i += 1) {
          const node = samples[samples.length - 1 - i];
          const pool = i ? samples.slice(samples.length - i) : [goal];
          const nearest = pool.reduce(
            (best, point) => (distance(point, node) < distance(best, node) ? point : best),
            pool[0],
          );
          drawEdge(nearest, node, obstacles, "rgba(180, 161, 255, .35)");
        }
      }
    }
    context.restore();
    for (const rect of obstacles) {
      context.fillStyle = "#263546";
      roundedRect(rect.x, rect.y, rect.w, rect.h, 8);
      context.strokeStyle = "rgba(195, 217, 237, .38)";
      context.lineWidth = 1;
      context.stroke();
    }
    for (let i = 0; i < count; i += 1) {
      const point = samples[i];
      const blocked = obstacles.some(
        (rect) =>
          point.x > rect.x &&
          point.x < rect.x + rect.w &&
          point.y > rect.y &&
          point.y < rect.y + rect.h,
      );
      context.beginPath();
      context.arc(point.x, point.y, blocked ? 2.2 : 2.8, 0, Math.PI * 2);
      context.fillStyle = blocked
        ? "rgba(255, 125, 141, .62)"
        : i % 6 === 0
          ? "#ffb875"
          : "rgba(139, 188, 255, .8)";
      context.fill();
    }
    if (count && state.progress < 0.55) {
      const sample = samples[count - 1];
      const blocked = obstacles.some(
        (rect) =>
          sample.x > rect.x &&
          sample.x < rect.x + rect.w &&
          sample.y > rect.y &&
          sample.y < rect.y + rect.h,
      );
      drawMotionMarker(context, sample, blocked ? "#ff8494" : "#a6d2ff", state.elapsed, "", 5);
      drawTinyLabel(
        blocked ? "REJECT COLLISION" : "NEW SAMPLE",
        sample.x + 12,
        sample.y - 9,
        blocked ? "#ff9cab" : "#c5e3ff",
      );
    }
    if (planner.id === "ait_star" || planner.id === "eit_star")
      drawReverseArrows(samples, count, goal, obstacles);
    if (planner.id === "jit_star" && state.progress > 0.46) {
      context.beginPath();
      context.ellipse(
        width * 0.55,
        height * 0.42,
        width * 0.16,
        height * 0.12,
        -0.48,
        0,
        Math.PI * 2,
      );
      context.fillStyle = "rgba(255, 184, 117, .055)";
      context.fill();
      context.strokeStyle = "rgba(255, 184, 117, .4)";
      context.setLineDash([3, 5]);
      context.stroke();
      context.setLineDash([]);
      drawTinyLabel("collision-guided samples", width * 0.42, height * 0.23, "#ffc78e");
    }
    if (planner.id === "abit_star" || planner.id === "bit_star" || planner.id === "fcit_star")
      drawTinyLabel("BATCH, EDGE QUEUE, INCUMBENT", 15, 19, "#9bb9d8");
    if (planner.id === "eirm_star")
      drawTinyLabel("ROADMAP CACHE, reusable valid or blocked edges", 15, 19, "#9bb9d8");
    if (planner.id === "rit_star")
      drawTinyLabel("metric-shaped neighborhood", width * 0.48, height * 0.9, "#c2afff");
    const routeProgress = Math.max(0, Math.min(1, (state.progress - 0.48) / 0.52));
    if (route.length > 1) {
      drawCandidatePath(route, state.progress, state.elapsed, "rgba(131, 240, 205, .96)");
    }
    if (roomyRoute.length > 1 && state.progress > 0.66 && REFINEMENT_PLANNERS.has(planner.id)) {
      drawCandidatePath(roomyRoute, state.progress, state.elapsed, "rgba(131, 240, 205, .28)");
      drawCandidatePath(route, state.progress, state.elapsed, "rgba(255, 184, 117, .94)", 0.66);
      drawTinyLabel("collision-free route refinement", width * 0.39, height * 0.18, "#ffca97");
    }
    if (route.length > 1 && state.progress > 0.48) {
      drawMotionMarker(
        context,
        pathAtProgress(route, routeProgress),
        "#a4f5df",
        state.elapsed,
        "R",
        8,
      );
    }
    drawPoint(start.x, start.y, "#83f0cd", "START", state.elapsed);
    drawPoint(goal.x, goal.y, "#ffb875", "GOAL", state.elapsed + 1);
    if ((planner.id === "hybrid_astar" || planner.id === "state_lattice") && route.length > 1) {
      const vehiclePoint = pathAtProgress(route, state.progress);
      const before = pathAtProgress(route, Math.max(0, state.progress - 0.01));
      const after = pathAtProgress(route, Math.min(1, state.progress + 0.01));
      drawVehicle(
        vehiclePoint.x,
        vehiclePoint.y,
        Math.atan2(after.y - before.y, after.x - before.x),
      );
    }
    if (twoTrees && state.progress > 0.72)
      drawTinyLabel("TREES CONNECT", width * 0.43, height * 0.84, "#bff9e7");
  }

  function drawEdge(a, b, obstacles, color) {
    if (obstacles.some((rect) => segmentHitsRect(a, b, rect))) return;
    context.beginPath();
    context.moveTo(a.x, a.y);
    context.lineTo(b.x, b.y);
    context.strokeStyle = color;
    context.lineWidth = 0.8;
    context.stroke();
  }

  function drawReverseArrows(samples, count, goal, obstacles) {
    for (let i = 1; i < count; i += 9) {
      const point = samples[i];
      const dx = goal.x - point.x;
      const dy = goal.y - point.y;
      const length = Math.hypot(dx, dy) || 1;
      const arrow = { x: point.x + (dx / length) * 15, y: point.y + (dy / length) * 15 };
      if (obstacles.some((rect) => segmentHitsRect(point, arrow, rect))) continue;
      context.beginPath();
      context.moveTo(point.x, point.y);
      context.lineTo(arrow.x, arrow.y);
      context.strokeStyle = "rgba(255, 184, 117, .57)";
      context.lineWidth = 1;
      context.stroke();
    }
    drawTinyLabel("reverse cost-to-go estimate", goal.x - 158, goal.y - 18, "#ffca97");
  }

  function drawCandidatePath(points, progress, elapsed, color, startProgress = 0.28) {
    if (progress < startProgress) return;
    const reveal = Math.max(0, Math.min(1, (progress - startProgress) / 0.2));
    context.save();
    context.lineWidth = 2.8;
    context.lineCap = "round";
    context.lineJoin = "round";
    context.strokeStyle = color;
    context.setLineDash([8, 6]);
    context.lineDashOffset = -elapsed * 20;
    drawPathTrace(context, points, reveal);
    context.restore();
  }

  function drawPoint(x, y, color, label, phase) {
    const pulse = (Math.sin(phase * 4) + 1) / 2;
    context.save();
    context.beginPath();
    context.arc(x, y, 12 + pulse * 3, 0, Math.PI * 2);
    context.strokeStyle = `${color}66`;
    context.lineWidth = 1.4;
    context.stroke();
    context.shadowColor = color;
    context.shadowBlur = 9 + pulse * 8;
    context.beginPath();
    context.arc(x, y, 7, 0, Math.PI * 2);
    context.fillStyle = color;
    context.fill();
    context.shadowBlur = 0;
    context.font = "9px ui-monospace, monospace";
    context.fillStyle = "#d6e0ef";
    context.textAlign = "center";
    context.fillText(label, x, y + 24);
    context.restore();
  }

  function drawVehicle(x, y, angle) {
    context.save();
    context.translate(x, y);
    context.rotate(angle);
    context.fillStyle = "#83f0cd";
    roundedRect(-11, -7, 22, 14, 4);
    context.fillStyle = "#122326";
    context.fillRect(1, -5, 7, 10);
    context.fillStyle = "#baffea";
    context.beginPath();
    context.moveTo(0, -8);
    context.lineTo(8, -13);
    context.lineTo(8, -8);
    context.fill();
    context.restore();
  }
  return drawContinuous;
}
