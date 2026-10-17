"use strict";

import { createCatalog } from "./scripts/catalog.js";
import { startApplicationScenes } from "./scripts/applications.js";
import { createSimulator } from "./scripts/simulator.js";

async function loadPlanners() {
  const response = await fetch(new URL("data/planners.json", window.location.href));
  if (!response.ok) throw new Error(`Planner catalogue request failed: ${response.status}`);
  const planners = await response.json();
  if (!Array.isArray(planners) || planners.length === 0) {
    throw new Error("The planner catalogue is empty or invalid.");
  }
  return planners;
}

async function startLearningLab() {
  let planners;
  try {
    planners = await loadPlanners();
  } catch (error) {
    const status = document.querySelector("#simulation-status");
    if (status) status.textContent = "Could not load the planner catalogue";
    console.error(error);
    return;
  }

  const requestedDemoId = new URLSearchParams(window.location.search).get("demo");
  const initialPlanner = planners.find((planner) => planner.id === requestedDemoId)
    || planners.find((planner) => planner.id === "astar")
    || planners[0];
  const isDemoMode = initialPlanner.id === requestedDemoId;
  if (isDemoMode) document.documentElement.classList.add("demo-mode");

  const simulator = createSimulator(planners, initialPlanner, isDemoMode);
  createCatalog(planners, simulator.setPlanner);
}

startLearningLab();
startApplicationScenes();
