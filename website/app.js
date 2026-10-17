(async () => {
  "use strict";

  let planners;
  try {
    const response = await fetch(new URL("data/planners.json", window.location.href));
    if (!response.ok) throw new Error(`Planner catalogue request failed: ${response.status}`);
    planners = await response.json();
  } catch (error) {
    const status = document.querySelector("#simulation-status");
    if (status) status.textContent = "Could not load the planner catalogue";
    console.error(error);
    return;
  }

  const requestedDemoId = new URLSearchParams(window.location.search).get("demo");
  const initialPlanner = planners.find((planner) => planner.id === requestedDemoId)
    || planners.find((planner) => planner.id === "astar");
  const isDemoMode = initialPlanner.id === requestedDemoId;
  if (isDemoMode) document.documentElement.classList.add("demo-mode");


  const kindLabels = {
    discrete: "Grid & graph search",
    continuous: "Continuous motion",
    temporal: "Time-dependent planning",
    multi_agent: "Multi-agent planning",
  };
  const kindShortLabels = {
    discrete: "GRID & GRAPH",
    continuous: "CONTINUOUS SPACE",
    temporal: "TIME-AWARE",
    multi_agent: "MULTI-AGENT",
  };
  const kindSymbols = { discrete: "▦", continuous: "⤴", temporal: "◷", multi_agent: "⧉" };
  const plannerById = new Map(planners.map((planner) => [planner.id, planner]));
  const gridResults = new Map();
  const canvas = document.querySelector("#sim-canvas");
  const context = canvas.getContext("2d");
  const selector = document.querySelector("#planner-select");
  const playButton = document.querySelector("#play-button");
  const stepButton = document.querySelector("#step-button");
  const resetButton = document.querySelector("#reset-button");
  const changeButton = document.querySelector("#change-button");
  const prefersReducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  const state = { planner: initialPlanner, progress: isDemoMode ? 0 : 0.4, playing: !prefersReducedMotion, changed: false, seed: 7, lastFrame: 0, width: 900, height: 500 };

  function fillSelector() {
    const kinds = ["discrete", "continuous", "temporal", "multi_agent"];
    selector.innerHTML = kinds.map((kind) => {
      const options = planners.filter((planner) => planner.kind === kind)
        .map((planner) => `<option value="${planner.id}">${planner.title}</option>`).join("");
      return `<optgroup label="${kindLabels[kind]}">${options}</optgroup>`;
    }).join("");
    selector.value = state.planner.id;
  }

  function updatePlannerNote() {
    const planner = state.planner;
    const index = planners.indexOf(planner) + 1;
    document.querySelector("#planner-kind").textContent = kindShortLabels[planner.kind];
    document.querySelector("#planner-index").textContent = `${String(index).padStart(2, "0")} / ${planners.length}`;
    document.querySelector("#planner-title").textContent = planner.title;
    document.querySelector("#planner-description").textContent = planner.description;
    document.querySelector("#planner-observe").textContent = planner.observe;
    document.querySelector("#planner-tradeoff").textContent = planner.tradeoff;
    const modes = { discrete: "GRID MAP · 2D", continuous: "CONFIGURATION SPACE", temporal: "SAFE INTERVALS · TIME", multi_agent: "MULTI-AGENT · SPACE/TIME" };
    document.querySelector("#simulation-mode").textContent = modes[planner.kind];
    document.querySelector("#canvas-legend").innerHTML = legendFor(planner.kind);
    changeButton.hidden = planner.id !== "dstar_lite";
    updateSimulationStatus();
  }

  function legendFor(kind) {
    if (kind === "continuous") return "<span><i class=\"legend-start\"></i>Start</span><span><i class=\"legend-goal\"></i>Goal</span><span><i class=\"legend-open\"></i>Samples / connections</span><span><i class=\"legend-path\"></i>Candidate path</span>";
    if (kind === "temporal") return "<span><i class=\"legend-start\"></i>Agent</span><span><i class=\"legend-goal\"></i>Goal</span><span><i class=\"legend-open\"></i>Safe interval</span><span><i class=\"legend-path\"></i>Time-aware route</span>";
    if (kind === "multi_agent") return "<span><i class=\"legend-start\"></i>Agent A</span><span><i class=\"legend-goal\"></i>Agent B</span><span><i class=\"legend-open\"></i>Agent C</span><span><i class=\"legend-path\"></i>Conflict-free paths</span>";
    return "<span><i class=\"legend-start\"></i>Start</span><span><i class=\"legend-goal\"></i>Goal</span><span><i class=\"legend-open\"></i>Explored frontier</span><span><i class=\"legend-path\"></i>Path</span>";
  }

  function renderCatalog() {
    const selectedKind = document.querySelector(".filter-chip.is-active")?.dataset.filter || "all";
    const query = document.querySelector("#planner-search").value.trim().toLowerCase();
    const visible = planners.filter((planner) => {
      const matchesKind = selectedKind === "all" || planner.kind === selectedKind;
      const searchText = `${planner.id} ${planner.title} ${planner.principle} ${planner.description}`.toLowerCase();
      return matchesKind && (!query || searchText.includes(query));
    });
    const grid = document.querySelector("#planner-grid");
    grid.innerHTML = "";
    if (!visible.length) {
      grid.innerHTML = '<div class="empty-state">No planners match that search. Try another name or idea.</div>';
    }
    for (const planner of visible) {
      const card = document.createElement("article");
      card.className = "planner-card";
      const top = document.createElement("div");
      top.className = "planner-card-top";
      const family = document.createElement("span");
      family.className = "planner-family";
      family.textContent = kindLabels[planner.kind];
      const mark = document.createElement("span");
      mark.className = "planner-mark";
      mark.setAttribute("aria-hidden", "true");
      mark.textContent = kindSymbols[planner.kind];
      top.append(family, mark);

      const heading = document.createElement("h3");
      const lessonLink = document.createElement("a");
      lessonLink.className = "planner-card-title";
      lessonLink.href = `algorithms/${encodeURIComponent(planner.id)}/`;
      lessonLink.textContent = planner.title;
      heading.append(lessonLink);

      const description = document.createElement("p");
      description.textContent = planner.description;
      const footer = document.createElement("div");
      footer.className = "planner-card-footer";
      const principle = document.createElement("span");
      principle.className = "planner-principle";
      principle.textContent = planner.principle;
      const actions = document.createElement("span");
      actions.className = "planner-card-actions";
      const readLink = document.createElement("a");
      readLink.className = "card-lesson";
      readLink.href = lessonLink.href;
      readLink.textContent = "Read lesson ↗";
      const demoButton = document.createElement("button");
      demoButton.className = "card-demo";
      demoButton.type = "button";
      demoButton.dataset.planner = planner.id;
      demoButton.setAttribute("aria-label", `Show ${planner.title} in the simulator`);
      demoButton.textContent = "Try it";
      actions.append(readLink, demoButton);
      footer.append(principle, actions);
      card.append(top, heading, description, footer);
      grid.append(card);
    }
    document.querySelector("#catalog-count").textContent = `Showing ${visible.length} of ${planners.length} planners`;
  }

  function setPlanner(plannerId, shouldScroll = false) {
    const next = plannerById.get(plannerId);
    if (!next) return;
    state.planner = next;
    state.progress = prefersReducedMotion ? 0.76 : 0;
    state.playing = !prefersReducedMotion;
    state.changed = false;
    selector.value = plannerId;
    updatePlannerNote();
    if (shouldScroll) document.querySelector("#playground").scrollIntoView({ behavior: prefersReducedMotion ? "instant" : "smooth", block: "start" });
    drawScene();
    updatePlayButton();
  }

  function updatePlayButton() {
    playButton.innerHTML = state.playing ? '<span aria-hidden="true">Ⅱ</span> Pause' : '<span aria-hidden="true">▶</span> Run';
  }

  function updateSimulationStatus() {
    const status = document.querySelector("#simulation-status");
    if (state.changed && state.planner.id === "dstar_lite") status.textContent = state.playing ? "World changed · repairing route" : "World changed · route repaired";
    else if (state.progress >= 1) status.textContent = "Illustration complete";
    else if (state.playing) {
      const labels = { discrete: "Exploring the search frontier", continuous: "Sampling and connecting", temporal: "Checking safe time windows", multi_agent: "Resolving path conflicts" };
      status.textContent = labels[state.planner.kind];
    } else status.textContent = "Ready to explore";
  }

  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.round(rect.width * ratio);
    const height = Math.round(rect.height * ratio);
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    state.width = rect.width;
    state.height = rect.height;
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    drawScene();
  }

  function seededRandom(seed) {
    let value = seed >>> 0;
    return () => {
      value = (value * 1664525 + 1013904223) >>> 0;
      return value / 4294967296;
    };
  }

  const columns = 30;
  const rows = 17;
  const startCell = { x: 1, y: 1 };
  const goalCell = { x: 28, y: 15 };
  const directions = [[1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [-1, 1], [1, -1], [-1, -1]];

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
    return 1 + (((x * 13 + y * 7) % 17) < 5 ? 1.2 : 0);
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
        const cost = diagonal * (planner.id === "jpsw" ? (terrainCost(x, y) + terrainCost(nx, ny)) / 2 : 1);
        const candidate = g[current] + cost;
        if (candidate + 1e-9 >= g[next]) continue;
        g[next] = candidate;
        parent[next] = current;
        if (planner.id === "reexp_astar" || planner.id === "anytime_astar" || planner.id === "weighted_astar") closed[next] = 0;
        frontier.push(next);
      }
      steps += 1;
      if (isQueue && queueHead > 128 && queueHead > frontier.length / 2) { frontier = frontier.slice(queueHead); queueHead = 0; }
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
      const weight = planner.id === "weighted_astar" ? 1.8 : planner.id === "reexp_astar" ? 1.35 : planner.id === "anytime_astar" ? 1.65 : 1;
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
      if (doubledError > -dy) { error -= dy; cx += stepX; }
      if (doubledError < dx) { error += dx; cy += stepY; }
      if (cx !== previousX && cy !== previousY && (blocked.has(`${cx},${previousY}`) || blocked.has(`${previousX},${cy}`))) return false;
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

  function drawScene() {
    if (!context || !state.width || !state.height) return;
    const { width, height } = state;
    context.clearRect(0, 0, width, height);
    context.fillStyle = "#101b2b";
    context.fillRect(0, 0, width, height);
    if (state.planner.kind === "discrete") drawGrid(width, height);
    else if (state.planner.kind === "continuous") drawContinuous(width, height);
    else if (state.planner.kind === "temporal") drawTemporal(width, height);
    else drawMultiAgent(width, height);
  }

  function drawGrid(width, height) {
    const cacheKey = `${state.planner.id}:${state.changed}`;
    if (!gridResults.has(cacheKey)) gridResults.set(cacheKey, searchGrid(state.planner, state.changed));
    const result = gridResults.get(cacheKey);
    const gap = 2;
    const cell = Math.min((width - 38) / columns, (height - 38) / rows);
    const originX = (width - cell * columns) / 2;
    const originY = (height - cell * rows) / 2;
    const visibleCount = Math.floor(result.expanded.length * Math.min(1, state.progress * 1.13 + 0.04));
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
          fill = state.planner.id.startsWith("bidirectional_") && exploredAt > split ? "rgba(180, 161, 255, .39)" : "rgba(116, 171, 231, .37)";
        }
        if (state.planner.id === "jpsw" && !isWall && !isStart && !isGoal && terrainCost(x, y) > 1) fill = exploredAt !== undefined ? "rgba(255, 184, 117, .32)" : "rgba(255, 184, 117, .12)";
        context.fillStyle = fill;
        roundedRect(context, px + gap / 2, py + gap / 2, cell - gap, cell - gap, Math.max(2, cell * .14));
        if (isWall) {
          context.fillStyle = "rgba(204, 221, 238, .07)";
          roundedRect(context, px + cell * .16, py + cell * .18, cell * .68, cell * .64, Math.max(2, cell * .1));
        }
      }
    }
    let path = result.path;
    if (["theta_star", "lazy_theta_star"].includes(state.planner.id)) path = simplifyPath(path, result.blocked);
    const revealStart = Math.max(.35, Math.min(.78, result.expanded.length ? result.expanded.indexOf(columns * goalCell.y + goalCell.x) / result.expanded.length : .6));
    const pathProgress = Math.max(0, Math.min(1, (state.progress - revealStart) / (1 - revealStart)));
    if (path.length > 1 && pathProgress > 0) {
      const pointCount = Math.max(2, Math.ceil(path.length * pathProgress));
      context.beginPath();
      path.slice(0, pointCount).forEach((node, index) => {
        const px = originX + (node.x + .5) * cell;
        const py = originY + (node.y + .5) * cell;
        if (index === 0) context.moveTo(px, py); else context.lineTo(px, py);
      });
      context.strokeStyle = "rgba(131, 240, 205, .96)";
      context.lineWidth = Math.max(2, cell * .16);
      context.lineCap = "round";
      context.lineJoin = "round";
      context.stroke();
      context.fillStyle = "#baffea";
      for (const node of path.filter((_, index) => index % 4 === 0)) {
        context.beginPath();
        context.arc(originX + (node.x + .5) * cell, originY + (node.y + .5) * cell, Math.max(2, cell * .1), 0, Math.PI * 2);
        context.fill();
      }
    }
    drawGridMarker(originX + (startCell.x + .5) * cell, originY + (startCell.y + .5) * cell, cell, "#83f0cd", "S");
    drawGridMarker(originX + (goalCell.x + .5) * cell, originY + (goalCell.y + .5) * cell, cell, "#ffb875", "G");
    if (state.planner.id === "dstar_lite" && !state.changed) drawTinyLabel("Press ‘Change map’ to block a route", 15, height - 12, "#c4d0df");
    if (state.planner.id === "anytime_astar" && state.progress > .68) drawTinyLabel("incumbent → shorter incumbent", width - 194, 17, "#a4edd7");
    if (state.planner.id.startsWith("bidirectional_")) drawTinyLabel("forward wave   ·   backward wave", width - 196, 17, "#c9bbff");
  }

  function drawGridMarker(x, y, cell, color, label) {
    context.beginPath();
    context.arc(x, y, cell * .32, 0, Math.PI * 2);
    context.fillStyle = color;
    context.fill();
    context.fillStyle = "#10202a";
    context.font = `700 ${Math.max(8, cell * .36)}px ui-monospace, monospace`;
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText(label, x, y + .5);
  }

  function roundedRect(ctx, x, y, w, h, radius) {
    const r = Math.max(0, Math.min(radius, w / 2, h / 2));
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
    ctx.fill();
  }

  function drawTinyLabel(text, x, y, color) {
    context.font = "10px ui-monospace, SFMono-Regular, Menlo, monospace";
    context.textAlign = "left";
    context.textBaseline = "alphabetic";
    context.fillStyle = color;
    context.fillText(text, x, y);
  }

  function rectsForWorld(width, height) {
    return [
      { x: width * .36, y: height * .22, w: width * .16, h: height * .48 },
      { x: width * .64, y: height * .48, w: width * .13, h: height * .3 },
      { x: width * .78, y: height * .12, w: width * .07, h: height * .2 },
    ];
  }

  function segmentHitsRect(a, b, rect) {
    const steps = 14;
    for (let i = 0; i <= steps; i += 1) {
      const x = a.x + (b.x - a.x) * i / steps;
      const y = a.y + (b.y - a.y) * i / steps;
      if (x >= rect.x && x <= rect.x + rect.w && y >= rect.y && y <= rect.y + rect.h) return true;
    }
    return false;
  }

  function drawContinuous(width, height) {
    const planner = state.planner;
    const obstacles = rectsForWorld(width, height);
    const random = seededRandom(state.seed + planners.indexOf(planner) * 17);
    const samples = Array.from({ length: 82 }, () => ({ x: width * .07 + random() * width * .86, y: height * .13 + random() * height * .74 }));
    for (const rect of obstacles) {
      context.fillStyle = "rgba(113, 139, 161, .23)";
      roundedRect(context, rect.x, rect.y, rect.w, rect.h, 8);
      context.strokeStyle = "rgba(166, 196, 219, .13)";
      context.lineWidth = 1;
      context.stroke();
    }
    for (let x = 24; x < width; x += 30) {
      context.beginPath(); context.moveTo(x, 0); context.lineTo(x, height); context.strokeStyle = "rgba(170, 194, 223, .035)"; context.stroke();
    }
    for (let y = 23; y < height; y += 30) {
      context.beginPath(); context.moveTo(0, y); context.lineTo(width, y); context.strokeStyle = "rgba(170, 194, 223, .035)"; context.stroke();
    }
    const start = { x: width * .07, y: height * .76 };
    const goal = { x: width * .92, y: height * .22 };
    const informed = ["informed_rrt_star", "bit_star", "abit_star", "fcit_star", "rit_star"].includes(planner.id) && state.progress > .43;
    if (informed) {
      context.save();
      context.translate((start.x + goal.x) / 2, (start.y + goal.y) / 2);
      context.rotate(-.54);
      context.beginPath(); context.ellipse(0, 0, width * (.36 - Math.max(0, state.progress - .45) * .14), height * .13, 0, 0, Math.PI * 2);
      context.fillStyle = "rgba(131, 240, 205, .045)"; context.fill();
      context.setLineDash([5, 6]); context.strokeStyle = "rgba(131, 240, 205, .32)"; context.lineWidth = 1.2; context.stroke();
      context.restore();
    }
    const count = Math.floor(samples.length * Math.min(1, state.progress * 1.1));
    const trees = ["rrt", "rrt_star", "informed_rrt_star", "rrt_connect", "eirm_star", "rit_star", "jit_star"].includes(planner.id);
    const twoTrees = planner.id === "rrt_connect";
    const roadmap = !trees || ["eirm_star", "rit_star"].includes(planner.id);
    context.save();
    if (roadmap) {
      const maxEdges = Math.min(count * 4, 190);
      let drawn = 0;
      for (let i = 0; i < count && drawn < maxEdges; i += 1) {
        const from = samples[i];
        const nearest = samples.slice(0, i).map((point, index) => ({ point, index, d: distance(from, point) })).sort((a, b) => a.d - b.d).slice(0, planner.id === "fcit_star" ? 8 : 4);
        for (const item of nearest) {
          if (drawn >= maxEdges) break;
          const crosses = obstacles.some((rect) => segmentHitsRect(from, item.point, rect));
          const lazy = planner.id === "lazy_prm" || planner.id === "eirm_star";
          context.beginPath(); context.moveTo(from.x, from.y); context.lineTo(item.point.x, item.point.y);
          context.lineWidth = .8;
          context.strokeStyle = crosses ? "rgba(255, 125, 141, .3)" : lazy ? "rgba(139, 188, 255, .28)" : "rgba(139, 188, 255, .2)";
          if (lazy && !crosses && drawn > maxEdges * .78) context.setLineDash([3, 5]); else context.setLineDash([]);
          context.stroke(); drawn += 1;
        }
      }
    } else {
      const treeCount = Math.min(count, 57);
      const leftCount = twoTrees ? Math.ceil(treeCount / 2) : treeCount;
      const rightCount = twoTrees ? Math.max(0, treeCount - leftCount) : 0;
      for (let i = 0; i < leftCount; i += 1) {
        const node = samples[i];
        const pool = i ? samples.slice(0, i) : [start];
        const nearest = pool.reduce((best, point) => distance(point, node) < distance(best, node) ? point : best, pool[0]);
        drawEdge(nearest, node, obstacles, i % 5 === 0 ? "rgba(131, 240, 205, .46)" : "rgba(130, 169, 208, .23)");
      }
      if (twoTrees) {
        for (let i = 0; i < rightCount; i += 1) {
          const node = samples[samples.length - 1 - i];
          const pool = i ? samples.slice(samples.length - i) : [goal];
          const nearest = pool.reduce((best, point) => distance(point, node) < distance(best, node) ? point : best, pool[0]);
          drawEdge(nearest, node, obstacles, "rgba(180, 161, 255, .35)");
        }
      }
    }
    context.restore();
    for (let i = 0; i < count; i += 1) {
      const point = samples[i];
      const blocked = obstacles.some((rect) => point.x > rect.x && point.x < rect.x + rect.w && point.y > rect.y && point.y < rect.y + rect.h);
      context.beginPath(); context.arc(point.x, point.y, blocked ? 2.2 : 2.8, 0, Math.PI * 2);
      context.fillStyle = blocked ? "rgba(255, 125, 141, .62)" : i % 6 === 0 ? "#ffb875" : "rgba(139, 188, 255, .8)";
      context.fill();
    }
    if (planner.id === "ait_star" || planner.id === "eit_star") drawReverseArrows(samples, count, goal);
    if (planner.id === "jit_star" && state.progress > .46) {
      context.beginPath(); context.ellipse(width * .55, height * .42, width * .16, height * .12, -.48, 0, Math.PI * 2);
      context.fillStyle = "rgba(255, 184, 117, .055)"; context.fill();
      context.strokeStyle = "rgba(255, 184, 117, .4)"; context.setLineDash([3, 5]); context.stroke(); context.setLineDash([]);
      drawTinyLabel("collision-guided samples", width * .42, height * .23, "#ffc78e");
    }
    if (planner.id === "abit_star" || planner.id === "bit_star" || planner.id === "fcit_star") drawTinyLabel("BATCH · EDGE QUEUE · INCUMBENT", 15, 19, "#9bb9d8");
    if (planner.id === "eirm_star") drawTinyLabel("ROADMAP CACHE · reusable valid / blocked edges", 15, 19, "#9bb9d8");
    if (planner.id === "rit_star") drawTinyLabel("metric-shaped neighborhood", width * .48, height * .9, "#c2afff");
    drawCandidatePath(start, goal, obstacles, width, height, state.progress);
    drawPoint(start.x, start.y, "#83f0cd", "START");
    drawPoint(goal.x, goal.y, "#ffb875", "GOAL");
    if (planner.id === "hybrid_astar" || planner.id === "state_lattice") drawVehicle(width * (.12 + .7 * state.progress), height * (.73 - .42 * state.progress), -0.52);
    if (twoTrees && state.progress > .72) drawTinyLabel("TREES CONNECT", width * .43, height * .84, "#bff9e7");
  }

  function distance(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }

  function drawEdge(a, b, obstacles, color) {
    if (obstacles.some((rect) => segmentHitsRect(a, b, rect))) return;
    context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y);
    context.strokeStyle = color; context.lineWidth = .8; context.stroke();
  }

  function drawReverseArrows(samples, count, goal) {
    for (let i = 1; i < count; i += 9) {
      const point = samples[i];
      const dx = goal.x - point.x; const dy = goal.y - point.y; const length = Math.hypot(dx, dy) || 1;
      context.beginPath(); context.moveTo(point.x, point.y); context.lineTo(point.x + dx / length * 15, point.y + dy / length * 15);
      context.strokeStyle = "rgba(255, 184, 117, .57)"; context.lineWidth = 1; context.stroke();
    }
    drawTinyLabel("reverse cost-to-go estimate", goal.x - 158, goal.y - 18, "#ffca97");
  }

  function drawCandidatePath(start, goal, obstacles, width, height, progress) {
    if (progress < .3) return;
    const points = [start, { x: width * .25, y: height * .79 }, { x: width * .31, y: height * .83 }, { x: width * .55, y: height * .83 }, { x: width * .58, y: height * .28 }, { x: width * .87, y: height * .27 }, goal];
    context.beginPath();
    points.forEach((point, index) => { if (!index) context.moveTo(point.x, point.y); else context.lineTo(point.x, point.y); });
    context.lineWidth = 2.4; context.lineCap = "round"; context.lineJoin = "round";
    context.strokeStyle = "rgba(131, 240, 205, .86)"; context.setLineDash([7, 6]); context.stroke(); context.setLineDash([]);
    if (progress > .66 && ["rrt_star", "informed_rrt_star", "bit_star", "abit_star", "rit_star"].includes(state.planner.id)) {
      context.beginPath(); context.moveTo(start.x, start.y); context.bezierCurveTo(width * .34, height * .7, width * .5, height * .2, goal.x, goal.y);
      context.strokeStyle = "rgba(255, 184, 117, .67)"; context.lineWidth = 1.6; context.setLineDash([4, 5]); context.stroke(); context.setLineDash([]);
      drawTinyLabel("improved incumbent", width * .48, height * .18, "#ffca97");
    }
  }

  function drawPoint(x, y, color, label) {
    context.beginPath(); context.arc(x, y, 7, 0, Math.PI * 2); context.fillStyle = color; context.fill();
    context.beginPath(); context.arc(x, y, 12, 0, Math.PI * 2); context.strokeStyle = `${color}66`; context.stroke();
    context.font = "9px ui-monospace, monospace"; context.fillStyle = "#d6e0ef"; context.textAlign = "center"; context.fillText(label, x, y + 24);
  }

  function drawVehicle(x, y, angle) {
    context.save(); context.translate(x, y); context.rotate(angle);
    context.fillStyle = "#83f0cd"; roundedRect(context, -11, -7, 22, 14, 4);
    context.fillStyle = "#122326"; context.fillRect(1, -5, 7, 10);
    context.fillStyle = "#baffea"; context.beginPath(); context.moveTo(0, -8); context.lineTo(8, -13); context.lineTo(8, -8); context.fill();
    context.restore();
  }

  function drawTemporal(width, height) {
    const lanes = ["Dock", "Hall", "Lift", "Goal"];
    const left = 80; const right = width - 24; const top = 54; const laneGap = (height - 110) / (lanes.length - 1);
    context.font = "10px ui-monospace, monospace"; context.textBaseline = "middle";
    for (let i = 0; i < lanes.length; i += 1) {
      const y = top + i * laneGap;
      context.fillStyle = "#a7b6ca"; context.textAlign = "right"; context.fillText(lanes[i], left - 15, y);
      context.beginPath(); context.moveTo(left, y); context.lineTo(right, y); context.strokeStyle = "rgba(164, 189, 220, .18)"; context.lineWidth = 1; context.stroke();
      context.beginPath(); context.arc(left, y, 4, 0, Math.PI * 2); context.fillStyle = i === 0 ? "#83f0cd" : i === lanes.length - 1 ? "#ffb875" : "#8092aa"; context.fill();
    }
    const blocks = [{ lane: 1, from: .18, to: .53 }, { lane: 2, from: .48, to: .76 }, { lane: 1, from: .74, to: .9 }];
    blocks.forEach((block) => {
      const y = top + block.lane * laneGap - 11;
      const x = left + (right - left) * block.from;
      const w = (right - left) * (block.to - block.from);
      context.fillStyle = "rgba(255, 125, 141, .19)"; roundedRect(context, x, y, w, 22, 6);
      context.strokeStyle = "rgba(255, 125, 141, .29)"; context.stroke();
      context.fillStyle = "#ef9ba7"; context.font = "8px ui-monospace, monospace"; context.textAlign = "center"; context.fillText("blocked", x + w / 2, y + 11);
    });
    const pathPoints = [0, .2, .48, .48, .69, .82, 1].map((t, i) => ({ x: left + t * (right - left), y: top + [0, 0, 1, 2, 2, 3, 3][i] * laneGap }));
    context.beginPath(); pathPoints.forEach((point, index) => { if (!index) context.moveTo(point.x, point.y); else context.lineTo(point.x, point.y); });
    context.strokeStyle = "rgba(131, 240, 205, .8)"; context.lineWidth = 2.5; context.stroke();
    const dot = pathAtProgress(pathPoints, state.progress);
    context.beginPath(); context.arc(dot.x, dot.y, 7, 0, Math.PI * 2); context.fillStyle = "#83f0cd"; context.fill();
    context.beginPath(); context.arc(dot.x, dot.y, 13, 0, Math.PI * 2); context.strokeStyle = "rgba(131, 240, 205, .3)"; context.stroke();
    context.fillStyle = "#8697ad"; context.font = "9px ui-monospace, monospace"; context.textAlign = "left";
    context.fillText("TIME →", left, height - 25);
    context.textAlign = "right"; context.fillText("Half-open blocked intervals [start, end)", right, height - 25);
    if (state.planner.id === "kinodynamic_sipp") drawTinyLabel("wait only at zero speed · v(t) changes along each edge", width * .4, 22, "#c3b1ff");
    else if (state.planner.id === "bounded_suboptimal_sipp") drawTinyLabel("FOCAL: choose within the w-bounded band", width * .48, 22, "#c3b1ff");
    else drawTinyLabel("wait safely · enter when the next interval opens", width * .47, 22, "#a5efd9");
  }

  function pathAtProgress(points, progress) {
    const total = points.slice(1).reduce((sum, point, index) => sum + distance(point, points[index]), 0);
    let remaining = total * progress;
    for (let i = 1; i < points.length; i += 1) {
      const length = distance(points[i - 1], points[i]);
      if (remaining <= length) return { x: points[i - 1].x + (points[i].x - points[i - 1].x) * remaining / length, y: points[i - 1].y + (points[i].y - points[i - 1].y) * remaining / length };
      remaining -= length;
    }
    return points.at(-1);
  }

  function drawMultiAgent(width, height) {
    const cols = 11; const rows = 6; const gapX = (width - 88) / (cols - 1); const gapY = (height - 90) / (rows - 1); const ox = 44; const oy = 45;
    const node = (x, y) => ({ x: ox + x * gapX, y: oy + y * gapY });
    for (let y = 0; y < rows; y += 1) for (let x = 0; x < cols; x += 1) {
      if (x < cols - 1) drawNetworkEdge(node(x, y), node(x + 1, y));
      if (y < rows - 1) drawNetworkEdge(node(x, y), node(x, y + 1));
    }
    const paths = [
      { color: "#83f0cd", label: "A", nodes: [[0, 0], [2, 0], [2, 2], [5, 2], [5, 4], [10, 4]] },
      { color: "#ffb875", label: "B", nodes: [[10, 0], [8, 0], [8, 1], [5, 1], [5, 3], [1, 3], [1, 5]] },
      { color: "#b4a1ff", label: "C", nodes: [[0, 5], [3, 5], [3, 4], [7, 4], [7, 2], [10, 2]] },
    ];
    paths.forEach((agent, index) => {
      const points = agent.nodes.map(([x, y]) => node(x, y));
      context.beginPath(); points.forEach((point, i) => { if (!i) context.moveTo(point.x, point.y); else context.lineTo(point.x, point.y); });
      context.strokeStyle = `${agent.color}9c`; context.lineWidth = 2.2; context.setLineDash([6, 5]); context.stroke(); context.setLineDash([]);
      points.forEach((point, i) => {
        if (i % 2 === 0 || i === points.length - 1) { context.beginPath(); context.arc(point.x, point.y, 3, 0, Math.PI * 2); context.fillStyle = `${agent.color}88`; context.fill(); }
      });
      const position = pathAtProgress(points, Math.min(1, state.progress * 1.12 + index * .09));
      context.beginPath(); context.arc(position.x, position.y, 8, 0, Math.PI * 2); context.fillStyle = agent.color; context.fill();
      context.font = "700 8px ui-monospace, monospace"; context.textAlign = "center"; context.textBaseline = "middle"; context.fillStyle = "#10202a"; context.fillText(agent.label, position.x, position.y + .4);
    });
    context.textAlign = "left"; context.textBaseline = "alphabetic"; context.fillStyle = "#9aa9bd"; context.font = "9px ui-monospace, monospace";
    context.fillText(state.planner.id === "eecbs" ? "CONFLICT? → ADD A CONSTRAINT → REPLAN" : "ASSIGN JOINT MOVE → REJECT CONFLICT → CONTINUE", 16, height - 17);
  }

  function drawNetworkEdge(a, b) {
    context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y);
    context.strokeStyle = "rgba(161, 185, 213, .12)"; context.lineWidth = 1; context.stroke();
    context.beginPath(); context.arc(a.x, a.y, 2.2, 0, Math.PI * 2); context.fillStyle = "rgba(190, 211, 234, .44)"; context.fill();
    context.beginPath(); context.arc(b.x, b.y, 2.2, 0, Math.PI * 2); context.fill();
  }

  function animate(timestamp) {
    if (state.lastFrame && state.playing) {
      const delta = Math.min(48, timestamp - state.lastFrame);
      state.progress = Math.min(1, state.progress + delta / 13500);
      if (state.progress >= 1) state.playing = false;
      updatePlayButton();
      updateSimulationStatus();
    }
    state.lastFrame = timestamp;
    drawScene();
    window.requestAnimationFrame(animate);
  }

  selector.addEventListener("change", (event) => setPlanner(event.target.value));
  document.querySelector("#planner-grid").addEventListener("click", (event) => {
    const button = event.target.closest("[data-planner]");
    if (button) setPlanner(button.dataset.planner, true);
  });
  document.querySelectorAll(".filter-chip").forEach((button) => button.addEventListener("click", () => {
    document.querySelector(".filter-chip.is-active").classList.remove("is-active");
    button.classList.add("is-active");
    renderCatalog();
  }));
  document.querySelector("#planner-search").addEventListener("input", renderCatalog);
  playButton.addEventListener("click", () => {
    if (state.progress >= 1) state.progress = 0;
    state.playing = !state.playing;
    updatePlayButton();
    updateSimulationStatus();
  });
  stepButton.addEventListener("click", () => {
    state.playing = false;
    state.progress = Math.min(1, state.progress + .08);
    updatePlayButton();
    updateSimulationStatus();
    drawScene();
  });
  resetButton.addEventListener("click", () => {
    state.progress = 0;
    state.playing = false;
    state.changed = false;
    updatePlayButton();
    updateSimulationStatus();
    drawScene();
  });
  changeButton.addEventListener("click", () => {
    state.changed = !state.changed;
    state.progress = 0;
    state.playing = true;
    updateSimulationStatus();
    updatePlayButton();
    drawScene();
  });

  fillSelector();
  updatePlannerNote();
  renderCatalog();
  resizeCanvas();
  window.addEventListener("resize", resizeCanvas, { passive: true });
  window.requestAnimationFrame(animate);
})();
