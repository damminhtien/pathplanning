"use strict";

import { createGridScene } from "./scenes/grid.js";
import { createContinuousScene } from "./scenes/continuous.js";
import { createTemporalScene } from "./scenes/temporal.js";
import { createMultiAgentScene } from "./scenes/multi-agent.js";
import { PLANNER_KIND_LABELS, PLANNER_KIND_SHORT_LABELS } from "./planner-kinds.js";

export function createSimulator(planners, initialPlanner, isDemoMode) {
  const plannerById = new Map(planners.map((planner) => [planner.id, planner]));
  const canvas = document.querySelector("#sim-canvas");
  const context = canvas.getContext("2d");
  const selector = document.querySelector("#planner-select");
  const playButton = document.querySelector("#play-button");
  const stepButton = document.querySelector("#step-button");
  const resetButton = document.querySelector("#reset-button");
  const changeButton = document.querySelector("#change-button");
  const prefersReducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  const state = { planner: initialPlanner, progress: isDemoMode ? 0 : 0.4, elapsed: 0, playing: !prefersReducedMotion, changed: false, seed: 7, lastFrame: 0, width: 900, height: 500 };

  const drawGrid = createGridScene({ context, state });
  const drawContinuous = createContinuousScene({ context, state, planners });
  const drawTemporal = createTemporalScene({ context, state });
  const drawMultiAgent = createMultiAgentScene({ context, state });
  const scenes = {
    discrete: drawGrid,
    continuous: drawContinuous,
    temporal: drawTemporal,
    multi_agent: drawMultiAgent,
  };

  function fillSelector() {
    const kinds = Object.keys(PLANNER_KIND_LABELS);
    selector.innerHTML = kinds.map((kind) => {
      const options = planners.filter((planner) => planner.kind === kind)
        .map((planner) => `<option value="${planner.id}">${planner.title}</option>`).join("");
      return `<optgroup label="${PLANNER_KIND_LABELS[kind]}">${options}</optgroup>`;
    }).join("");
    selector.value = state.planner.id;
  }

  function updatePlannerNote() {
    const planner = state.planner;
    const index = planners.indexOf(planner) + 1;
    document.querySelector("#planner-kind").textContent = PLANNER_KIND_SHORT_LABELS[planner.kind]
      || planner.kind.toUpperCase();
    document.querySelector("#planner-index").textContent = `${String(index).padStart(2, "0")} / ${planners.length}`;
    document.querySelector("#planner-title").textContent = planner.title;
    document.querySelector("#planner-description").textContent = planner.description;
    document.querySelector("#planner-observe").textContent = planner.observe;
    document.querySelector("#planner-tradeoff").textContent = planner.tradeoff;
    const modes = { discrete: "GRID MAP, 2D", continuous: "CONFIGURATION SPACE", temporal: "SAFE INTERVALS, TIME", multi_agent: "MULTI-AGENT, SPACE/TIME" };
    document.querySelector("#simulation-mode").textContent = modes[planner.kind] || "MOTION PLANNING";
    document.querySelector("#canvas-legend").innerHTML = legendFor(planner.kind);
    changeButton.hidden = planner.id !== "dstar_lite";
    updateSimulationStatus();
  }

  function legendFor(kind) {
    if (kind === "continuous") return "<span><i class=\"legend-start\"></i>Start</span><span><i class=\"legend-goal\"></i>Goal</span><span><i class=\"legend-open\"></i>Samples / connections</span><span><i class=\"legend-path\"></i>Candidate path</span>";
    if (kind === "temporal") return "<span><i class=\"legend-start\"></i>Agent</span><span><i class=\"legend-goal\"></i>Goal</span><span><i class=\"legend-open\"></i>Safe interval</span><span><i class=\"legend-path\"></i>Time-aware route</span>";
    if (kind === "multi_agent") {
      return [
        '<span><i class="legend-agent-a"></i>Agent A</span>',
        '<span><i class="legend-agent-b"></i>Agent B</span>',
        '<span><i class="legend-agent-c"></i>Agent C</span>',
        '<span><i class="legend-path"></i>Conflict-free paths</span>',
      ].join("");
    }
    return "<span><i class=\"legend-start\"></i>Start</span><span><i class=\"legend-goal\"></i>Goal</span><span><i class=\"legend-open\"></i>Explored frontier</span><span><i class=\"legend-path\"></i>Path</span>";
  }

  function setPlanner(plannerId, shouldScroll = false) {
    const next = plannerById.get(plannerId);
    if (!next) return;
    state.planner = next;
    state.progress = prefersReducedMotion ? 0.76 : 0;
    state.elapsed = 0;
    state.playing = !prefersReducedMotion;
    state.changed = false;
    selector.value = plannerId;
    updatePlannerNote();
    if (shouldScroll) document.querySelector("#playground").scrollIntoView({ behavior: prefersReducedMotion ? "instant" : "smooth", block: "start" });
    drawScene();
    updatePlayButton();
  }

  function updatePlayButton() {
    playButton.textContent = state.playing ? "Pause" : "Run";
  }

  function updateSimulationStatus() {
    const status = document.querySelector("#simulation-status");
    if (state.changed && state.planner.id === "dstar_lite") status.textContent = state.playing ? "World changed, repairing route" : "World changed, route repaired";
    else if (state.progress >= 1) status.textContent = "Illustration complete";
    else if (state.playing) {
      const labels = { discrete: "Exploring the search frontier", continuous: "Sampling and connecting", temporal: "Checking safe time windows", multi_agent: "Resolving path conflicts" };
      status.textContent = labels[state.planner.kind] || "Exploring the planner";
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

  function drawScene() {
    if (!context || !state.width || !state.height) return;
    const { width, height } = state;
    context.clearRect(0, 0, width, height);
    context.fillStyle = "#101b2b";
    context.fillRect(0, 0, width, height);
    scenes[state.planner.kind]?.(width, height);
  }

  function animate(timestamp) {
    if (state.lastFrame && state.playing) {
      const delta = Math.min(48, timestamp - state.lastFrame);
      state.progress = Math.min(1, state.progress + delta / 13500);
      state.elapsed += delta / 1000;
      if (state.progress >= 1) state.playing = false;
      updatePlayButton();
      updateSimulationStatus();
    }
    state.lastFrame = timestamp;
    drawScene();
    window.requestAnimationFrame(animate);
  }

  selector.addEventListener("change", (event) => setPlanner(event.target.value));
  playButton.addEventListener("click", () => {
    if (state.progress >= 1) {
      state.progress = 0;
      state.elapsed = 0;
    }
    state.playing = !state.playing;
    updatePlayButton();
    updateSimulationStatus();
  });
  stepButton.addEventListener("click", () => {
    state.playing = false;
    state.progress = Math.min(1, state.progress + .08);
    state.elapsed += 1.08;
    updatePlayButton();
    updateSimulationStatus();
    drawScene();
  });
  resetButton.addEventListener("click", () => {
    state.progress = 0;
    state.elapsed = 0;
    state.playing = false;
    state.changed = false;
    updatePlayButton();
    updateSimulationStatus();
    drawScene();
  });
  changeButton.addEventListener("click", () => {
    state.changed = !state.changed;
    state.progress = 0;
    state.elapsed = 0;
    state.playing = true;
    updateSimulationStatus();
    updatePlayButton();
    drawScene();
  });

  fillSelector();
  updatePlannerNote();
  resizeCanvas();
  window.addEventListener("resize", resizeCanvas, { passive: true });
  window.requestAnimationFrame(animate);
  return { setPlanner };
}
