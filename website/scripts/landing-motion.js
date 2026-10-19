const REVEAL_GROUPS = [
  [".hero-copy > *", true],
  [".hero-art", false],
  [".hero-stats > div", true],
  [".loop-copy, .loop-diagram", false],
  [".section-heading", false],
  [".application-card", true],
  [".environment-preview", true],
  [".workbench, .under-note, .catalog-controls, .catalog-count", false],
  [".inventory-head", false],
  [".dataset-card", true],
  [".case-study-card", true],
  [".dataset-guide-link, .campaign-panel, .not-leaderboard", false],
  [".method-step", true],
  [".metric-cards article", true],
  [".evaluation-footnote, .closing-inner > *", false],
];

export function startLandingMotion() {
  startReadingProgress();

  if (
    window.matchMedia("(prefers-reduced-motion: reduce)").matches ||
    !("IntersectionObserver" in window)
  )
    return;

  const revealTargets = collectRevealTargets();
  if (!revealTargets.length) return;

  const observer = new IntersectionObserver(
    (entries) => {
      for (const entry of entries) {
        if (!entry.isIntersecting) continue;
        entry.target.classList.add("is-revealed");
        observer.unobserve(entry.target);
      }
    },
    { rootMargin: "0px 0px -6% 0px", threshold: 0.12 },
  );

  for (const target of revealTargets) observer.observe(target);
  document.documentElement.classList.add("landing-motion-ready");
}

function collectRevealTargets() {
  const delays = new Map();

  for (const [selector, stagger] of REVEAL_GROUPS) {
    const groupIndices = new Map();
    for (const target of document.querySelectorAll(selector)) {
      const group = stagger ? target.parentElement : target;
      const index = groupIndices.get(group) || 0;
      groupIndices.set(group, index + 1);
      delays.set(target, Math.min(index * 75, 300));
    }
  }

  for (const [target, delay] of delays) {
    target.dataset.reveal = "";
    target.style.setProperty("--reveal-delay", `${delay}ms`);
  }
  return [...delays.keys()];
}

function startReadingProgress() {
  const root = document.documentElement;
  let frame = 0;

  function update() {
    frame = 0;
    const pageLength = root.scrollHeight - window.innerHeight;
    const progress = pageLength > 0 ? window.scrollY / pageLength : 0;
    root.style.setProperty("--page-progress", String(Math.max(0, Math.min(1, progress))));
  }

  function scheduleUpdate() {
    if (!frame) frame = window.requestAnimationFrame(update);
  }

  window.addEventListener("scroll", scheduleUpdate, { passive: true });
  window.addEventListener("resize", scheduleUpdate, { passive: true });
  scheduleUpdate();
}
