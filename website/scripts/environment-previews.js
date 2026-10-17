"use strict";

import { createContinuousScene } from "./scenes/continuous.js";
import { createGridScene } from "./scenes/grid.js";
import { createMultiAgentScene } from "./scenes/multi-agent.js";
import { createTemporalScene } from "./scenes/temporal.js";

const SCENE_FACTORIES = {
  discrete: createGridScene,
  continuous: createContinuousScene,
  temporal: createTemporalScene,
  multi_agent: createMultiAgentScene,
};

export function startEnvironmentPreviews(planners) {
  const container = document.querySelector(".environment-preview-grid");
  if (!container) return () => {};

  const previews = [...container.querySelectorAll("canvas[data-environment-preview]")]
    .flatMap((canvas, index) => {
      const kind = canvas.dataset.environmentPreview;
      const planner = planners.find((item) => item.kind === kind);
      const context = canvas.getContext("2d");
      const createScene = SCENE_FACTORIES[kind];
      if (!planner || !context || !createScene) return [];

      const state = { planner, progress: .7, elapsed: 0, changed: false, seed: 41 + index * 17 };
      const draw = createScene({ context, state, planners });
      return [{ canvas, context, draw, state, phase: index * .31 }];
    });

  if (!previews.length) return () => {};

  const motionQuery = window.matchMedia("(prefers-reduced-motion: reduce)");
  let prefersReducedMotion = motionQuery.matches;
  let isVisible = false;
  let frameId = 0;

  function drawFrame(timestamp) {
    frameId = 0;
    const elapsed = timestamp / 1000;

    for (const preview of previews) {
      const { canvas, context, state } = preview;
      const bounds = canvas.getBoundingClientRect();
      if (!bounds.width || !bounds.height) continue;

      const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
      const pixelWidth = Math.round(bounds.width * pixelRatio);
      const pixelHeight = Math.round(bounds.height * pixelRatio);
      if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
        canvas.width = pixelWidth;
        canvas.height = pixelHeight;
      }
      context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
      context.clearRect(0, 0, bounds.width, bounds.height);
      context.fillStyle = "#101b2b";
      context.fillRect(0, 0, bounds.width, bounds.height);
      const scale = Math.min(bounds.width / 900, bounds.height / 500);
      const offsetX = (bounds.width - 900 * scale) / 2;
      const offsetY = (bounds.height - 500 * scale) / 2;
      context.setTransform(pixelRatio * scale, 0, 0, pixelRatio * scale, offsetX * pixelRatio, offsetY * pixelRatio);

      const cycle = (elapsed / 9 + preview.phase) % 2;
      const sweep = cycle <= 1 ? cycle : 2 - cycle;
      state.elapsed = prefersReducedMotion ? 0 : elapsed;
      state.progress = prefersReducedMotion ? .88 : .16 + sweep * .8;
      preview.draw(900, 500);
    }

    if (isVisible && !prefersReducedMotion) frameId = window.requestAnimationFrame(drawFrame);
  }

  function start() {
    if (isVisible) return;
    isVisible = true;
    if (prefersReducedMotion) drawFrame(0);
    else if (!frameId) frameId = window.requestAnimationFrame(drawFrame);
  }

  function stop() {
    isVisible = false;
    if (frameId) window.cancelAnimationFrame(frameId);
    frameId = 0;
  }

  const observer = "IntersectionObserver" in window
    ? new IntersectionObserver((entries) => {
      if (entries.some((entry) => entry.isIntersecting)) start();
      else stop();
    }, { rootMargin: "120px 0px", threshold: 0 })
    : null;

  function handleResize() {
    if (isVisible && prefersReducedMotion) drawFrame(0);
  }

  function handleMotionPreference(event) {
    prefersReducedMotion = event.matches;
    if (!isVisible) return;
    if (frameId) window.cancelAnimationFrame(frameId);
    frameId = 0;
    if (prefersReducedMotion) drawFrame(0);
    else frameId = window.requestAnimationFrame(drawFrame);
  }

  observer?.observe(container);
  if (!observer) start();
  window.addEventListener("resize", handleResize, { passive: true });
  motionQuery.addEventListener("change", handleMotionPreference);

  return () => {
    stop();
    observer?.disconnect();
    window.removeEventListener("resize", handleResize);
    motionQuery.removeEventListener("change", handleMotionPreference);
  };
}
