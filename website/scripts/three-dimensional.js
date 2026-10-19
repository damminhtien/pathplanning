import { createContinuousScene } from "./scenes/continuous-3d.js";
import { createVoxelScene } from "./scenes/voxel-3d.js";

export function startThreeDimensionalScenes() {
  const section = document.querySelector(".three-dimensional-section");
  if (!section) return () => {};

  const sceneDrawers = { voxel: createVoxelScene(), continuous: createContinuousScene() };
  const scenes = [...section.querySelectorAll("canvas[data-three-d-scene]")].flatMap((canvas) => {
    const context = canvas.getContext("2d");
    if (!context) return [];
    return [{ canvas, context, kind: canvas.dataset.threeDScene }];
  });
  if (!scenes.length) return () => {};

  const motionQuery = window.matchMedia("(prefers-reduced-motion: reduce)");
  let prefersReducedMotion = motionQuery.matches;
  let isVisible = false;
  let frameId = 0;

  function drawFrame(timestamp) {
    frameId = 0;
    const elapsed = timestamp / 1000;
    const phase = prefersReducedMotion ? 1 : (elapsed % 11) / 11;

    for (const { canvas, context, kind } of scenes) {
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
      context.fillStyle = "#f5f9fd";
      context.fillRect(0, 0, bounds.width, bounds.height);
      sceneDrawers[kind]?.(context, bounds.width, bounds.height, phase, elapsed);
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

  const observer =
    "IntersectionObserver" in window
      ? new IntersectionObserver(
          (entries) => {
            if (entries.some((entry) => entry.isIntersecting)) start();
            else stop();
          },
          { rootMargin: "100px 0px", threshold: 0 },
        )
      : null;
  const handleResize = () => {
    if (isVisible && prefersReducedMotion) drawFrame(0);
  };
  const handleMotionPreference = (event) => {
    prefersReducedMotion = event.matches;
    if (!isVisible) return;
    if (frameId) window.cancelAnimationFrame(frameId);
    frameId = 0;
    if (prefersReducedMotion) drawFrame(0);
    else frameId = window.requestAnimationFrame(drawFrame);
  };

  observer?.observe(section);
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
