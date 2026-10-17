"use strict";

const TRAVELER_SELECTOR = "[data-motion-target]";
const PATH_SELECTOR = "[data-motion-path]";

export function startApplicationScenes() {
  const scenes = [...document.querySelectorAll(".application-visual")]
    .map(createScene)
    .filter(Boolean);
  if (!scenes.length) return;

  const motionPreference = window.matchMedia("(prefers-reduced-motion: reduce)");
  const visibleScenes = new Set();
  let animationFrame = 0;

  function updateScene(scene, timestamp) {
    for (const traveler of scene.travelers) {
      const elapsed = timestamp / 1000 / traveler.duration + traveler.phaseOffset;
      const phase = ((elapsed % 1) + 1) % 1;
      const progress = progressWithWait(phase, traveler.wait);
      const distance = progress * traveler.length;
      const point = traveler.path.getPointAtLength(distance);
      let rotation = 0;
      if (traveler.rotate) {
        const next = traveler.path.getPointAtLength(Math.min(traveler.length, distance + 0.5));
        rotation = Math.atan2(next.y - point.y, next.x - point.x) * (180 / Math.PI);
      }
      traveler.element.setAttribute("transform", `translate(${point.x} ${point.y}) rotate(${rotation})`);
    }
  }

  function draw(timestamp) {
    animationFrame = 0;
    if (motionPreference.matches || document.hidden) return;
    for (const scene of visibleScenes) updateScene(scene, timestamp);
    if (visibleScenes.size) animationFrame = window.requestAnimationFrame(draw);
  }

  function start() {
    if (!motionPreference.matches && !document.hidden && visibleScenes.size && !animationFrame) {
      animationFrame = window.requestAnimationFrame(draw);
    }
  }

  function stop() {
    if (animationFrame) window.cancelAnimationFrame(animationFrame);
    animationFrame = 0;
  }

  if ("IntersectionObserver" in window) {
    const observer = new IntersectionObserver((entries) => {
      for (const entry of entries) {
        const scene = scenes.find((item) => item.element === entry.target);
        if (!scene) continue;
        if (entry.isIntersecting) {
          visibleScenes.add(scene);
          scene.element.classList.add("is-visible");
        } else {
          visibleScenes.delete(scene);
          scene.element.classList.remove("is-visible");
        }
      }
      if (visibleScenes.size) start();
      else stop();
    }, { rootMargin: "120px 0px" });
    for (const scene of scenes) observer.observe(scene.element);
  } else {
    scenes.forEach((scene) => {
      visibleScenes.add(scene);
      scene.element.classList.add("is-visible");
    });
  }

  function applyMotionPreference() {
    document.documentElement.classList.toggle(
      "application-motion-paused",
      motionPreference.matches || document.hidden,
    );
    if (motionPreference.matches) {
      stop();
      for (const scene of scenes) updateScene(scene, 0);
      return;
    }
    start();
  }

  motionPreference.addEventListener?.("change", applyMotionPreference);
  document.addEventListener("visibilitychange", () => {
    document.documentElement.classList.toggle(
      "application-motion-paused",
      document.hidden || motionPreference.matches,
    );
    if (document.hidden) stop();
    else start();
  });
  applyMotionPreference();
}

function createScene(element) {
  const paths = new Map(
    [...element.querySelectorAll(PATH_SELECTOR)]
      .map((path) => [path.dataset.motionPath, path]),
  );
  const travelers = [...element.querySelectorAll(TRAVELER_SELECTOR)]
    .map((traveler) => {
      const path = paths.get(traveler.dataset.motionTarget);
      if (!path) return null;
      const length = path.getTotalLength();
      if (!length) return null;
      return {
        element: traveler,
        path,
        length,
        duration: Math.max(1, Number(traveler.dataset.motionDuration) || 7),
        phaseOffset: Number(traveler.dataset.motionOffset) || 0,
        wait: parseWait(traveler.dataset.motionWait),
        rotate: traveler.dataset.motionRotate === "true",
      };
    })
    .filter(Boolean);
  return travelers.length ? { element, travelers } : null;
}

function parseWait(value) {
  if (!value) return null;
  const [position, duration] = value.split(",").map(Number);
  if (!Number.isFinite(position) || !Number.isFinite(duration)) return null;
  return { position: Math.max(0, Math.min(1, position)), duration: Math.max(0, Math.min(0.5, duration)) };
}

function progressWithWait(phase, wait) {
  if (!wait || !wait.duration) return phase;
  const movingTime = 1 - wait.duration;
  const waitStart = wait.position * movingTime;
  const waitEnd = waitStart + wait.duration;
  if (phase < waitStart) return phase / movingTime;
  if (phase < waitEnd) return wait.position;
  return wait.position + (phase - waitEnd) / movingTime;
}
