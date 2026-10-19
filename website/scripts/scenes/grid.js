import {
  drawTinyLabel as drawCanvasLabel,
  drawMotionMarker,
  drawPathTrace,
  roundedRect as drawRoundedRect,
  pathAtProgress,
} from "./shared.js";

export function createGridScene({ context, state }) {
  const gridResults = new Map();
  const columns = 30;
  const rows = 17;
  const startCell = { x: 1, y: 1 };
  const goalCell = { x: 28, y: 15 };
  const directions = [
    [1, 0],
    [0, 1],
    [-1, 0],
    [0, -1],
    [1, 1],
    [-1, 1],
    [1, -1],
    [-1, -1],
  ];
  const roundedRect = (x, y, width, height, radius) =>
    drawRoundedRect(context, x, y, width, height, radius);
  const drawTinyLabel = (text, x, y, color) => drawCanvasLabel(context, text, x, y, color);

  function blockedCells(changed) {
    const cells = new Set();
    for (let y = 0; y <= 12; y += 1) if (y !== 5 && y !== 6) cells.add(`${8},${y}`);
    for (let y = 5; y < rows; y += 1) if (y !== 10 && y !== 11) cells.add(`${15},${y}`);
    for (let y = 0; y <= 10; y += 1) if (y !== 4 && y !== 5) cells.add(`${21},${y}`);
    for (let x = 23; x <= 26; x += 1) for (let y = 12; y <= 14; y += 1) cells.add(`${x},${y}`);
    for (let y = 3; y <= 8; y += 1) if (y !== 6) cells.add(`${4},${y}`);
    if (changed) for (let y = 0; y < 14; y += 1) if (y !== 12 && y !== 13) cells.add(`${25},${y}`);
    cells.delete(`${startCell.x},${startCell.y}`);
    cells.delete(`${goalCell.x},${goalCell.y}`);
    return cells;
  }

  function heuristic(x, y) {
    const dx = Math.abs(goalCell.x - x);
    const dy = Math.abs(goalCell.y - y);
    return Math.max(dx, dy) + (Math.SQRT2 - 1) * Math.min(dx, dy);
  }

  function terrainCost(x, y) {
    return 1 + ((x * 13 + y * 7) % 17 < 5 ? 1.2 : 0);
  }

  function searchGrid(planner, changed) {
    const blocked = blockedCells(changed);
    const count = columns * rows;
    const id = (x, y) => y * columns + x;
    const start = id(startCell.x, startCell.y);
    const goal = id(goalCell.x, goalCell.y);
    const g = new Float64Array(count).fill(Infinity);
    const parent = new Int32Array(count).fill(-1);
    const closed = new Uint8Array(count);
    const expanded = [];
    let frontier = [start];
    let queueHead = 0;
    g[start] = 0;
    const isStack = planner.id === "dfs";
    const isQueue = planner.id === "bfs";
    const maxSteps = count * 3;
    let steps = 0;
    while (frontier.length && steps < maxSteps) {
      let current;
      if (isQueue) current = frontier[queueHead++];
      else if (isStack) current = frontier.pop();
      else {
        frontier.sort((a, b) => priority(a) - priority(b));
        current = frontier.shift();
      }
      if (current === undefined || closed[current]) continue;
      const x = current % columns;
      const y = Math.floor(current / columns);
      closed[current] = 1;
      expanded.push(current);
      if (current === goal) break;
      const order = isStack ? [...directions].reverse() : directions;
      for (const [dx, dy] of order) {
        const nx = x + dx;
        const ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= columns || ny >= rows || blocked.has(`${nx},${ny}`)) continue;
        if (dx && dy && (blocked.has(`${x + dx},${y}`) || blocked.has(`${x},${y + dy}`))) continue;
        const next = id(nx, ny);
        const diagonal = dx && dy ? Math.SQRT2 : 1;
        const cost =
          diagonal * (planner.id === "jpsw" ? (terrainCost(x, y) + terrainCost(nx, ny)) / 2 : 1);
        const candidate = g[current] + cost;
        if (candidate + 1e-9 >= g[next]) continue;
        g[next] = candidate;
        parent[next] = current;
        if (
          planner.id === "reexp_astar" ||
          planner.id === "anytime_astar" ||
          planner.id === "weighted_astar"
        )
          closed[next] = 0;
        frontier.push(next);
      }
      steps += 1;
      if (isQueue && queueHead > 128 && queueHead > frontier.length / 2) {
        frontier = frontier.slice(queueHead);
        queueHead = 0;
      }
    }
    const path = [];
    if (Number.isFinite(g[goal])) {
      let current = goal;
      while (current >= 0 && path.length <= count) {
        path.push({ x: current % columns, y: Math.floor(current / columns) });
        if (current === start) break;
        current = parent[current];
      }
      if (path.at(-1)?.x === startCell.x && path.at(-1)?.y === startCell.y) path.reverse();
      else path.length = 0;
    }
    return { expanded, path, blocked };

    function priority(node) {
      const x = node % columns;
      const y = Math.floor(node / columns);
      if (planner.id === "greedy_best_first") return heuristic(x, y);
      if (planner.id === "dijkstra" || planner.id === "bidirectional_dijkstra") return g[node];
      const weight =
        planner.id === "weighted_astar"
          ? 1.8
          : planner.id === "reexp_astar"
            ? 1.35
            : planner.id === "anytime_astar"
              ? 1.65
              : 1;
      return g[node] + weight * heuristic(x, y);
    }
  }

  function cellVisible(x, y, blocked) {
    let cx = startCell.x;
    let cy = startCell.y;
    const dx = Math.abs(x - cx);
    const dy = Math.abs(y - cy);
    const stepX = x >= cx ? 1 : -1;
    const stepY = y >= cy ? 1 : -1;
    let error = dx - dy;
    while (cx !== x || cy !== y) {
      const previousX = cx;
      const previousY = cy;
      const doubledError = 2 * error;
      if (doubledError > -dy) {
        error -= dy;
        cx += stepX;
      }
      if (doubledError < dx) {
        error += dx;
        cy += stepY;
      }
      if (
        cx !== previousX &&
        cy !== previousY &&
        (blocked.has(`${cx},${previousY}`) || blocked.has(`${previousX},${cy}`))
      )
        return false;
      if (blocked.has(`${cx},${cy}`)) return false;
    }
    return true;
  }

  function simplifyPath(path, blocked) {
    if (path.length < 3) return path;
    const result = [path[0]];
    let index = 0;
    while (index < path.length - 1) {
      let next = path.length - 1;
      while (next > index + 1 && !cellVisible(path[next].x, path[next].y, blocked)) next -= 1;
      result.push(path[next]);
      index = next;
    }
    return result;
  }

  function drawGrid(width, height) {
    const cacheKey = `${state.planner.id}:${state.changed}`;
    if (!gridResults.has(cacheKey))
      gridResults.set(cacheKey, searchGrid(state.planner, state.changed));
    const result = gridResults.get(cacheKey);
    const gap = 2;
    const cell = Math.min((width - 38) / columns, (height - 38) / rows);
    const originX = (width - cell * columns) / 2;
    const originY = (height - cell * rows) / 2;
    const visibleCount = Math.floor(
      result.expanded.length * Math.min(1, state.progress * 1.13 + 0.04),
    );
    const visited = new Map();
    const split = Math.floor(visibleCount / 2);
    for (let i = 0; i < visibleCount; i += 1) visited.set(result.expanded[i], i);
    for (let y = 0; y < rows; y += 1) {
      for (let x = 0; x < columns; x += 1) {
        const index = y * columns + x;
        const px = originX + x * cell;
        const py = originY + y * cell;
        const key = `${x},${y}`;
        const isStart = x === startCell.x && y === startCell.y;
        const isGoal = x === goalCell.x && y === goalCell.y;
        const isWall = result.blocked.has(key);
        const exploredAt = visited.get(index);
        let fill = "#142237";
        if (isWall) fill = "#516176";
        else if (exploredAt !== undefined) {
          const age = visibleCount - exploredAt - 1;
          const glow = Math.max(0, 1 - age / 14);
          const alpha = 0.24 + glow * 0.32;
          const tint =
            state.planner.id.startsWith("bidirectional_") && exploredAt > split
              ? "180, 161, 255"
              : "116, 171, 231";
          fill = `rgba(${tint}, ${alpha})`;
        }
        if (state.planner.id === "jpsw" && !isWall && !isStart && !isGoal && terrainCost(x, y) > 1)
          fill = exploredAt !== undefined ? "rgba(255, 184, 117, .32)" : "rgba(255, 184, 117, .12)";
        context.fillStyle = fill;
        roundedRect(px + gap / 2, py + gap / 2, cell - gap, cell - gap, Math.max(2, cell * 0.14));
        if (isWall) {
          context.fillStyle = "rgba(204, 221, 238, .07)";
          roundedRect(
            px + cell * 0.16,
            py + cell * 0.18,
            cell * 0.68,
            cell * 0.64,
            Math.max(2, cell * 0.1),
          );
        }
      }
    }
    let path = result.path;
    if (["theta_star", "lazy_theta_star"].includes(state.planner.id))
      path = simplifyPath(path, result.blocked);
    const revealStart = Math.max(
      0.35,
      Math.min(
        0.78,
        result.expanded.length
          ? result.expanded.indexOf(columns * goalCell.y + goalCell.x) / result.expanded.length
          : 0.6,
      ),
    );
    const pathProgress = Math.max(
      0,
      Math.min(1, (state.progress - revealStart) / (1 - revealStart)),
    );
    if (visibleCount) {
      const frontier = result.expanded[Math.min(visibleCount - 1, result.expanded.length - 1)];
      const frontierX = originX + ((frontier % columns) + 0.5) * cell;
      const frontierY = originY + (Math.floor(frontier / columns) + 0.5) * cell;
      const pulse = 0.32 + ((Math.sin(state.elapsed * 7) + 1) / 2) * 0.22;
      context.beginPath();
      context.arc(frontierX, frontierY, cell * pulse, 0, Math.PI * 2);
      context.strokeStyle = "rgba(193, 224, 255, .92)";
      context.lineWidth = Math.max(1.5, cell * 0.09);
      context.stroke();
    }
    if (path.length > 1 && pathProgress > 0) {
      const canvasPath = path.map((node) => ({
        x: originX + (node.x + 0.5) * cell,
        y: originY + (node.y + 0.5) * cell,
      }));
      context.save();
      context.strokeStyle = "rgba(131, 240, 205, .96)";
      context.lineWidth = Math.max(2, cell * 0.16);
      context.lineCap = "round";
      context.lineJoin = "round";
      context.setLineDash([7, 6]);
      context.lineDashOffset = -state.elapsed * 18;
      drawPathTrace(context, canvasPath, pathProgress);
      context.restore();
      const robot = pathAtProgress(canvasPath, pathProgress);
      drawMotionMarker(context, robot, "#a4f5df", state.elapsed, "R", Math.max(5, cell * 0.2));
    }
    drawGridMarker(
      originX + (startCell.x + 0.5) * cell,
      originY + (startCell.y + 0.5) * cell,
      cell,
      "#83f0cd",
      "S",
    );
    drawGridMarker(
      originX + (goalCell.x + 0.5) * cell,
      originY + (goalCell.y + 0.5) * cell,
      cell,
      "#ffb875",
      "G",
    );
    if (state.planner.id === "dstar_lite" && !state.changed)
      drawTinyLabel('Press "Change map" to block a route', 15, height - 12, "#c4d0df");
    if (state.planner.id === "anytime_astar" && state.progress > 0.68)
      drawTinyLabel("incumbent improves", width - 194, 17, "#a4edd7");
    if (state.planner.id.startsWith("bidirectional_"))
      drawTinyLabel("forward and backward waves", width - 196, 17, "#c9bbff");
  }

  function drawGridMarker(x, y, cell, color, label) {
    context.beginPath();
    context.arc(x, y, cell * 0.32, 0, Math.PI * 2);
    context.fillStyle = color;
    context.fill();
    context.fillStyle = "#10202a";
    context.font = `700 ${Math.max(8, cell * 0.36)}px ui-monospace, monospace`;
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText(label, x, y + 0.5);
  }
  return drawGrid;
}
