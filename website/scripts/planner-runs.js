const TRACE_DURATION_MIN_MS = 4500;
const TRACE_DURATION_MAX_MS = 22000;

export function startRecordedPlannerRuns() {
  const canvas = document.querySelector("#run-canvas");
  if (!canvas) return;

  const context = canvas.getContext("2d");
  const tabs = document.querySelector("#recorded-run-tabs");
  const phaseTabs = document.querySelector("#run-phase-tabs");
  const progress = document.querySelector("#run-progress");
  const playButton = document.querySelector("#run-play");
  const resetButton = document.querySelector("#run-reset");
  const stepButton = document.querySelector("#run-step");
  const copyButton = document.querySelector("#copy-run-command");
  const state = {
    data: null,
    run: null,
    phase: null,
    phaseIndex: 0,
    cursor: 0,
    processed: 0,
    playing: false,
    startedAt: 0,
    startCursor: 0,
    parents: new Map(),
    expanded: new Set(),
    discovered: new Set(),
    samples: new Set(),
    hasSolution: false,
    width: 0,
    height: 0,
  };

  async function loadRuns() {
    try {
      const response = await fetch(new URL("data/planner-demo-runs.json", window.location.href));
      if (!response.ok) throw new Error(`Recorded trace request failed: ${response.status}`);
      const payload = await response.json();
      if (
        payload.schema_version !== 1 ||
        !Array.isArray(payload.runs) ||
        payload.runs.length !== 3
      ) {
        throw new Error("The recorded trace file has an unsupported schema.");
      }
      state.data = payload;
      renderRunTabs();
      selectRun(payload.runs[0]);
    } catch (error) {
      console.error(error);
      document.querySelector("#run-title").textContent = "Recorded runs could not be loaded";
      document.querySelector("#run-summary").textContent =
        "Reload the page or use the Python runner from the repository.";
      document.querySelector("#run-status").textContent = "Trace data unavailable";
      playButton.disabled = true;
      resetButton.disabled = true;
      stepButton.disabled = true;
    }
  }

  function renderRunTabs() {
    tabs.replaceChildren();
    for (const run of state.data.runs) {
      const button = document.createElement("button");
      button.className = "recorded-run-tab";
      button.type = "button";
      button.textContent = run.planner;
      button.setAttribute("aria-pressed", "false");
      button.addEventListener("click", () => selectRun(run));
      tabs.append(button);
    }
  }

  function selectRun(run) {
    stopPlayback();
    state.run = run;
    state.phaseIndex = 0;
    document.querySelectorAll(".recorded-run-tab").forEach((button, index) => {
      button.setAttribute("aria-pressed", String(state.data.runs[index] === run));
    });
    document.querySelector("#run-title").textContent = run.title;
    document.querySelector("#run-summary").textContent = run.summary;
    document.querySelector("#run-package-badge").textContent =
      `${state.data.package.name} ${state.data.package.version}`;
    document.querySelector("#run-evidence-tag").textContent =
      "TRACE CAPTURED FROM NATIVE PACKAGE RUN";
    renderPhaseTabs();
    setPhase(0);
  }

  function renderPhaseTabs() {
    phaseTabs.replaceChildren();
    const phases = state.run.phases;
    phaseTabs.hidden = phases.length <= 1;
    if (phases.length <= 1) return;
    phases.forEach((phase, index) => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "run-phase-tab";
      button.textContent = phase.label;
      button.setAttribute("aria-pressed", String(index === state.phaseIndex));
      button.addEventListener("click", () => setPhase(index));
      phaseTabs.append(button);
    });
  }

  function setPhase(index) {
    stopPlayback();
    state.phaseIndex = index;
    state.phase = state.run.phases[index];
    state.cursor = 0;
    resetTraceState();
    phaseTabs.querySelectorAll("button").forEach((button, buttonIndex) => {
      button.setAttribute("aria-pressed", String(buttonIndex === index));
    });
    renderRunDetails();
    const eventCount = state.phase.trace.events.length;
    progress.max = String(eventCount);
    progress.value = "0";
    document.querySelector("#run-event-count").textContent = `0 / ${eventCount} events`;
    playButton.textContent = "Play trace";
    updateStatus("paused");
    updateLegend();
    draw();
  }

  function renderRunDetails() {
    const run = state.run;
    const result = run.result;
    const phase = run.phases[state.phaseIndex];
    const metricItems = [];
    if (run.family === "continuous") {
      metricItems.push(["Path length", `${result.path_cost.toFixed(3)} m`]);
      metricItems.push(["Iterations", result.iterations.toLocaleString()]);
      metricItems.push(["Tree vertices", result.tree_vertices.toLocaleString()]);
      metricItems.push(["Route check", phase.valid ? "Passed" : "Failed"]);
    } else {
      metricItems.push(["Path cost", phase.path_cost.toFixed(3)]);
      metricItems.push(["Expanded", phase.expanded_nodes.toLocaleString()]);
      metricItems.push(["Route check", phase.valid ? "Passed" : "Failed"]);
      metricItems.push(["Package", state.data.package.version]);
    }
    const metrics = document.querySelector("#run-metrics");
    metrics.replaceChildren();
    for (const [label, value] of metricItems) {
      const item = document.createElement("div");
      item.className = "run-metric";
      const strong = document.createElement("strong");
      strong.textContent = value;
      const small = document.createElement("span");
      small.textContent = label;
      item.append(strong, small);
      metrics.append(item);
    }

    const input = run.input;
    const start = input.start || input.query?.start;
    const goal = input.goal || input.query?.goal;
    const facts = document.querySelector("#run-facts");
    facts.replaceChildren();
    addFact(facts, "Environment", input.environment);
    addFact(facts, "Start and goal", `${formatPoint(start)} to ${formatPoint(goal)}`);
    addFact(
      facts,
      "Random seed",
      run.seed.value === null
        ? run.seed.meaning
        : `${run.seed.value.toLocaleString()} · ${run.seed.meaning}`,
    );
    addFact(facts, "Movement model", input.movement);
    if (run.planner_id === "dstar_lite") {
      addFact(
        facts,
        "Map update",
        `Insert obstacle at ${formatPoint(input.inserted_obstacle)}; update ${input.directed_edges_updated} directed edge costs, then reuse the same D* Lite session.`,
      );
    }
    if (run.planner_id === "rrt_star") {
      addFact(facts, "Robot model", input.robot_model);
      addFact(
        facts,
        "Geometry",
        `${input.source_obstacle_count} source cylinders; bounds ${input.bounds[0].map(formatNumber).join(", ")} to ${input.bounds[1].map(formatNumber).join(", ")}.`,
      );
      addFact(
        facts,
        "Collision model",
        `Planner obstacle radii inflated by ${input.planning_obstacle_radius_inflation.toFixed(6)} m; validity checked against source radii.`,
      );
    }
    const parameterSummary =
      formatParameters(run.parameters) ||
      "Package defaults; heuristic and legal moves come from the MovingAI graph.";
    addFact(facts, "Planner parameters", parameterSummary);
    addFact(
      facts,
      "Package and runtime",
      `${state.data.package.name} ${state.data.package.version} · Python ${state.data.runtime.python_version}`,
    );
    const phaseCost =
      state.run.phases.length > 1
        ? `Current phase cost ${phase.path_cost.toFixed(3)}; final repaired cost ${result.path_cost.toFixed(3)}. `
        : `Path cost ${result.path_cost.toFixed(6)}. `;
    const routeCheck = phase.valid ? "Route check passed." : "Route check failed.";
    addFact(
      facts,
      "Result",
      `${phaseCost}${result.stop_reason}. ${routeCheck} ${phase.validity_method || result.validity_method}`,
    );
    if (input.source?.url || input.source?.benchmark_url) {
      addLinkFact(
        facts,
        "Source",
        input.source.url || input.source.benchmark_url,
        input.source.file || input.source.dataset,
      );
    }
    if (input.source?.sha256) addFact(facts, "Source SHA-256", input.source.sha256, true);
    if (input.source?.scenario_file) {
      addFact(
        facts,
        "Scenario",
        `${input.source.scenario_file}, line ${input.source.scenario_line}; reference cost ${input.source.published_optimal_cost.toFixed(8)}.`,
      );
    }
    if (run.planner_id === "rrt_star") {
      addFact(facts, "Query derivation", input.query.derivation);
      addFact(
        facts,
        "Scope",
        "This is a point-robot XY planning problem derived from BARN geometry. It does not evaluate the Jackal footprint, dynamics, or full simulator task.",
      );
    }

    document.querySelector("#run-command").textContent = state.data.reproduction_command;
    document
      .querySelector("#run-canvas")
      .setAttribute(
        "aria-label",
        `${run.planner} package trace replay on ${input.environment}. ${phase.trace.events.length} recorded events.`,
      );
  }

  function addFact(parent, label, value, asCode = false) {
    const term = document.createElement("dt");
    term.textContent = label;
    const description = document.createElement("dd");
    if (asCode) {
      const code = document.createElement("code");
      code.textContent = value;
      description.append(code);
    } else {
      description.textContent = value;
    }
    parent.append(term, description);
  }

  function addLinkFact(parent, label, url, text) {
    const term = document.createElement("dt");
    term.textContent = label;
    const description = document.createElement("dd");
    const link = document.createElement("a");
    link.href = url;
    link.target = "_blank";
    link.rel = "noreferrer";
    link.textContent = text;
    description.append(link);
    parent.append(term, description);
  }

  function formatParameters(parameters) {
    return Object.entries(parameters)
      .filter(([key]) => key !== "planner" && key !== "seed")
      .map(
        ([key, value]) =>
          `${key}=${value === null ? "None" : typeof value === "boolean" ? String(value) : typeof value === "number" ? formatNumber(value) : value}`,
      )
      .join(" · ");
  }

  function formatPoint(point) {
    if (!Array.isArray(point)) return "not recorded";
    return `(${point
      .map((value) =>
        Number(value)
          .toFixed(3)
          .replace(/\.?0+$/, ""),
      )
      .join(", ")})`;
  }

  function formatNumber(value) {
    return Number(value)
      .toFixed(4)
      .replace(/\.?0+$/, "");
  }

  function resetTraceState() {
    state.processed = 0;
    state.parents = new Map();
    state.expanded = new Set();
    state.discovered = new Set();
    state.samples = new Set();
    state.hasSolution = false;
  }

  function applyEventsUntil(target) {
    const events = state.phase.trace.events;
    if (target < state.processed) resetTraceState();
    while (state.processed < target) {
      const [node, parent, , kind] = events[state.processed];
      if (node >= 0) {
        if (kind === 1) state.discovered.add(node);
        if (kind === 2) state.expanded.add(node);
        if ((kind === 1 || kind === 3) && node !== parent) state.parents.set(node, parent);
        if (kind === 4) state.samples.add(node);
      }
      if (kind === 6) state.hasSolution = true;
      state.processed += 1;
    }
    state.cursor = target;
    progress.value = String(target);
    document.querySelector("#run-event-count").textContent =
      `${target.toLocaleString()} / ${events.length.toLocaleString()} events`;
    draw();
  }

  function updateLegend() {
    const items =
      state.run.family === "continuous"
        ? [
            ["tree", "Tree edge"],
            ["sample", "Samples (1 in 4 shown)"],
            ["path", "Verified route"],
            ["start", "Start"],
            ["goal", "Goal"],
          ]
        : [
            ["expanded", "Expanded node"],
            ["frontier", "Discovered"],
            ["path", "Verified route"],
            ["start", "Start"],
            ["goal", "Goal"],
          ];
    if (state.run.planner_id === "dstar_lite" && state.phase.id === "repair")
      items.unshift(["obstacle", "Inserted obstacle"]);
    const legend = document.querySelector("#run-legend");
    legend.replaceChildren();
    for (const [kind, label] of items) {
      const item = document.createElement("span");
      const swatch = document.createElement("i");
      swatch.className = `run-legend-${kind}`;
      item.append(swatch, document.createTextNode(label));
      legend.append(item);
    }
  }

  function draw() {
    const rect = canvas.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const pixelWidth = Math.max(1, Math.round(rect.width * ratio));
    const pixelHeight = Math.max(1, Math.round(rect.height * ratio));
    if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
      canvas.width = pixelWidth;
      canvas.height = pixelHeight;
    }
    state.width = rect.width;
    state.height = rect.height;
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, state.width, state.height);
    context.fillStyle = "#101b2b";
    context.fillRect(0, 0, state.width, state.height);
    if (!state.run || !state.phase) return;
    if (state.run.family === "continuous") drawContinuous();
    else drawGrid();
  }

  function worldTransform(bounds, padding = 38) {
    const [[xMin, yMin], [xMax, yMax]] = bounds;
    const scale = Math.min(
      (state.width - padding * 2) / (xMax - xMin),
      (state.height - padding * 2) / (yMax - yMin),
    );
    const worldWidth = (xMax - xMin) * scale;
    const worldHeight = (yMax - yMin) * scale;
    const left = (state.width - worldWidth) / 2;
    const top = (state.height - worldHeight) / 2;
    return {
      scale,
      x: (value) => left + (value - xMin) * scale,
      y: (value) => top + (yMax - value) * scale,
      left,
      top,
      right: left + worldWidth,
      bottom: top + worldHeight,
    };
  }

  function drawGrid() {
    const input = state.run.input;
    const width = input.width;
    const height = input.height;
    const map =
      state.run.planner_id === "dstar_lite" && state.phase.id === "repair"
        ? input.after_rows
        : state.run.planner_id === "dstar_lite"
          ? input.before_rows
          : input.map_rows;
    const transform = worldTransform(
      [
        [0, 0],
        [width, height],
      ],
      26,
    );
    const cell = transform.scale;

    for (let y = 0; y < height; y += 1) {
      const row = map[y];
      for (let x = 0; x < width; x += 1) {
        const px = transform.x(x);
        const py = transform.y(y + 1);
        const blocked = state.run.planner_id === "dstar_lite" ? row[x] === "#" : row[x] !== ".";
        const inserted =
          state.run.planner_id === "dstar_lite" &&
          state.phase.id === "repair" &&
          input.inserted_obstacle[0] === x &&
          input.inserted_obstacle[1] === y;
        context.fillStyle = inserted ? "rgba(255, 184, 117, .72)" : blocked ? "#26394e" : "#142438";
        context.fillRect(px, py, cell, cell);
        if (inserted) {
          context.strokeStyle = "#ffd09c";
          context.lineWidth = 2;
          context.strokeRect(px + 1, py + 1, cell - 2, cell - 2);
        }
        if (!blocked) {
          const node = y * width + x;
          if (state.discovered.has(node)) {
            context.fillStyle = "rgba(58, 134, 188, .34)";
            context.fillRect(px + 0.5, py + 0.5, cell - 1, cell - 1);
          }
          if (state.expanded.has(node)) {
            context.fillStyle = "rgba(85, 183, 197, .62)";
            context.fillRect(px + 0.5, py + 0.5, cell - 1, cell - 1);
          }
        }
      }
    }

    context.strokeStyle = "rgba(176, 200, 226, .09)";
    context.lineWidth = 0.5;
    context.beginPath();
    for (let x = 0; x <= width; x += 1) {
      context.moveTo(transform.x(x), transform.y(0));
      context.lineTo(transform.x(x), transform.y(height));
    }
    for (let y = 0; y <= height; y += 1) {
      context.moveTo(transform.x(0), transform.y(y));
      context.lineTo(transform.x(width), transform.y(y));
    }
    context.stroke();

    if (state.hasSolution) drawGridPath(state.phase.path, transform);
    drawEndpoint(input.start || input.query?.start, transform, "#83f0cd", cell * 0.27);
    drawEndpoint(input.goal || input.query?.goal, transform, "#ffb875", cell * 0.27);
    if (state.run.planner_id === "dstar_lite" && state.phase.id === "repair") {
      drawAnnotation(
        `Obstacle inserted at ${formatPoint(input.inserted_obstacle)}`,
        transform.left + 8,
        transform.top + 19,
      );
    }
    drawAxisLabel("x", transform.right - 8, transform.bottom + 20);
    drawAxisLabel("y", transform.left - 15, transform.top + 10);
  }

  function drawGridPath(path, transform) {
    if (!Array.isArray(path) || path.length < 2) return;
    context.beginPath();
    path.forEach(([x, y], index) => {
      const px = transform.x(x + 0.5);
      const py = transform.y(y + 0.5);
      if (index === 0) context.moveTo(px, py);
      else context.lineTo(px, py);
    });
    context.strokeStyle = "#83f0cd";
    context.lineWidth = Math.max(2, transform.scale * 0.16);
    context.lineCap = "round";
    context.lineJoin = "round";
    context.stroke();
  }

  function drawContinuous() {
    const input = state.run.input;
    const transform = worldTransform(input.bounds, 34);
    context.fillStyle = "#1a2c40";
    context.fillRect(
      transform.left,
      transform.top,
      transform.right - transform.left,
      transform.bottom - transform.top,
    );

    for (const [cx, cy, radius] of input.source_circles) {
      context.beginPath();
      context.arc(transform.x(cx), transform.y(cy), radius * transform.scale, 0, Math.PI * 2);
      context.fillStyle = "rgba(102, 135, 163, .54)";
      context.fill();
      context.strokeStyle = "rgba(185, 211, 231, .18)";
      context.lineWidth = 0.6;
      context.stroke();
    }

    const points = state.phase.trace.points;
    if (points) {
      context.lineWidth = 1;
      context.strokeStyle = "rgba(107, 179, 222, .46)";
      for (const [node, parent] of state.parents) {
        if (node < 0 || parent < 0 || node >= points.length || parent >= points.length) continue;
        context.beginPath();
        context.moveTo(transform.x(points[parent][0]), transform.y(points[parent][1]));
        context.lineTo(transform.x(points[node][0]), transform.y(points[node][1]));
        context.stroke();
      }
      context.fillStyle = "rgba(201, 222, 238, .55)";
      for (const node of state.samples) {
        if (node % 4 !== 0) continue;
        if (node < 0 || node >= points.length) continue;
        context.beginPath();
        context.arc(
          transform.x(points[node][0]),
          transform.y(points[node][1]),
          1.2,
          0,
          Math.PI * 2,
        );
        context.fill();
      }
    }

    if (state.hasSolution) drawContinuousPath(state.phase.path, transform);
    drawEndpoint(input.start || input.query?.start, transform, "#83f0cd", 5);
    drawEndpoint(input.goal || input.query?.goal, transform, "#ffb875", 5);
    drawAnnotation(`${input.source_obstacle_count} source obstacles · point robot XY`, 14, 19);
    drawAxisLabel("x (m)", transform.right - 12, transform.bottom + 20);
    drawAxisLabel("y (m)", transform.left - 26, transform.top + 9);
  }

  function drawContinuousPath(path, transform) {
    if (!Array.isArray(path) || path.length < 2) return;
    context.beginPath();
    path.forEach(([x, y], index) => {
      if (index === 0) context.moveTo(transform.x(x), transform.y(y));
      else context.lineTo(transform.x(x), transform.y(y));
    });
    context.strokeStyle = "#83f0cd";
    context.lineWidth = 3;
    context.lineCap = "round";
    context.lineJoin = "round";
    context.shadowColor = "rgba(131, 240, 205, .38)";
    context.shadowBlur = 8;
    context.stroke();
    context.shadowBlur = 0;
  }

  function drawEndpoint(point, transform, color, radius) {
    const x = transform.x(point[0] + (state.run.family === "discrete" ? 0.5 : 0));
    const y = transform.y(point[1] + (state.run.family === "discrete" ? 0.5 : 0));
    context.beginPath();
    context.arc(x, y, radius + 3, 0, Math.PI * 2);
    context.strokeStyle = color;
    context.lineWidth = 1.5;
    context.stroke();
    context.beginPath();
    context.arc(x, y, radius, 0, Math.PI * 2);
    context.fillStyle = color;
    context.fill();
  }

  function drawAnnotation(text, x, y) {
    context.font = "11px ui-monospace, SFMono-Regular, Menlo, monospace";
    context.fillStyle = "rgba(237, 245, 251, .88)";
    context.fillText(text, x, y);
  }

  function drawAxisLabel(text, x, y) {
    context.font = "10px ui-monospace, SFMono-Regular, Menlo, monospace";
    context.fillStyle = "rgba(178, 198, 217, .7)";
    context.fillText(text, x, y);
  }

  function updateStatus(mode) {
    const total = state.phase?.trace.events.length || 0;
    const complete = state.cursor >= total;
    const suffix =
      mode === "playing"
        ? "replaying captured events"
        : complete
          ? "trace complete"
          : "trace paused";
    document.querySelector("#run-status").textContent =
      `${state.phase?.label || "Trace"} · ${suffix}`;
    playButton.textContent = state.playing
      ? "Pause trace"
      : complete
        ? "Replay trace"
        : "Play trace";
  }

  function stopPlayback() {
    state.playing = false;
    updateStatus("paused");
  }

  function animate(timestamp) {
    if (!state.playing) return;
    const total = state.phase.trace.events.length;
    const duration = Math.min(TRACE_DURATION_MAX_MS, Math.max(TRACE_DURATION_MIN_MS, total * 2.4));
    if (!state.startedAt) state.startedAt = timestamp;
    const elapsed = Math.min(duration, timestamp - state.startedAt);
    const target = Math.floor(
      state.startCursor + ((total - state.startCursor) * elapsed) / duration,
    );
    applyEventsUntil(target);
    if (elapsed >= duration || target >= total) {
      state.playing = false;
      updateStatus("complete");
      return;
    }
    requestAnimationFrame(animate);
  }

  function beginPlayback() {
    if (!state.phase) return;
    if (state.cursor >= state.phase.trace.events.length) {
      resetTraceState();
      state.cursor = 0;
      progress.value = "0";
      applyEventsUntil(0);
    }
    state.playing = true;
    state.startedAt = 0;
    state.startCursor = state.cursor;
    updateStatus("playing");
    requestAnimationFrame(animate);
  }

  playButton.addEventListener("click", () => {
    if (state.playing) stopPlayback();
    else beginPlayback();
  });
  resetButton.addEventListener("click", () => {
    stopPlayback();
    state.cursor = 0;
    resetTraceState();
    progress.value = "0";
    applyEventsUntil(0);
    updateStatus("paused");
  });
  stepButton.addEventListener("click", () => {
    stopPlayback();
    applyEventsUntil(Math.min(state.phase.trace.events.length, state.cursor + 1));
    updateStatus("paused");
  });
  progress.addEventListener("input", () => {
    stopPlayback();
    applyEventsUntil(Number(progress.value));
    updateStatus("paused");
  });
  copyButton.addEventListener("click", async () => {
    try {
      await navigator.clipboard.writeText(state.data.reproduction_command);
      copyButton.textContent = "Copied";
      window.setTimeout(() => {
        copyButton.textContent = "Copy";
      }, 1600);
    } catch (error) {
      console.error("Could not copy the reproduction command", error);
      copyButton.textContent = "Select command";
    }
  });

  window.addEventListener("resize", draw, { passive: true });
  loadRuns();
}
