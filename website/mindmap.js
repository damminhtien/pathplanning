(() => {
  // biome-ignore lint/suspicious/noRedundantUseStrict: This page loads the file as a classic script.
  "use strict";

  const searchInput = document.querySelector("#map-search");
  const nodes = [...document.querySelectorAll(".mindmap-node")];
  const rootBranches = [...document.querySelectorAll(".mindmap-branches > .mindmap-item")];
  const emptyState = document.querySelector("#map-empty");
  const inspectorTitle = document.querySelector("#inspector-title");
  const inspectorSummary = document.querySelector("#inspector-summary");
  const inspectorSource = document.querySelector("#inspector-source");
  const inspectorPath = document.querySelector("#inspector-path");
  const inspectorLesson = document.querySelector("#inspector-lesson");

  function updateInspector(node) {
    if (!node) return;
    const summary = node.querySelector(":scope > .mindmap-content > p");
    const source = node.querySelector(":scope > .mindmap-content > .mindmap-source");
    const lesson = node.querySelector(":scope > .mindmap-content > .mindmap-lesson");
    const label = node.querySelector(":scope > summary .mindmap-label");

    nodes.forEach((entry) => {
      entry.classList.toggle("is-selected", entry === node);
    });
    inspectorTitle.textContent = label?.textContent || "Project map";
    inspectorSummary.textContent =
      summary?.textContent || "Explore the connections across the project.";
    inspectorSource.hidden = !source;
    if (source) {
      inspectorSource.href = source.href;
      inspectorPath.textContent = source.querySelector("code")?.textContent || "Source";
    }
    inspectorLesson.hidden = !lesson;
    if (lesson) {
      inspectorLesson.href = lesson.href;
      inspectorLesson.textContent = lesson.querySelector("span")?.textContent || "Read the lesson";
    }
  }

  function filterTree() {
    const query = searchInput.value.trim().toLocaleLowerCase();
    let visibleCount = 0;

    function visit(item) {
      const node = item.querySelector(":scope > .mindmap-node");
      const childItems = [
        ...item.querySelectorAll(
          ":scope > .mindmap-node > .mindmap-content > .mindmap-children > .mindmap-item",
        ),
      ];
      const childVisible = childItems.some(visit);
      const matches = !query || node.dataset.search.includes(query);
      const visible = matches || childVisible;

      item.hidden = !visible;
      if (query && childVisible) node.open = true;
      if (query && matches && node.querySelector(":scope > .mindmap-content > .mindmap-children"))
        node.open = true;
      if (visible) visibleCount += 1;
      return visible;
    }

    rootBranches.forEach(visit);
    emptyState.hidden = visibleCount > 0;
  }

  document.querySelector("#expand-map").addEventListener("click", () => {
    nodes.forEach((node) => {
      node.open = true;
    });
  });
  document.querySelector("#collapse-map").addEventListener("click", () => {
    nodes.forEach((node) => {
      node.open = false;
    });
  });
  searchInput.addEventListener("input", filterTree);

  document.querySelector("#project-map").addEventListener("click", (event) => {
    const focusButton = event.target.closest(".mindmap-focus");
    const summary = event.target.closest(".mindmap-node > summary");
    const node = (focusButton || summary)?.closest(".mindmap-node");
    if (node) updateInspector(node);
  });
  document.querySelector("#project-map").addEventListener(
    "toggle",
    (event) => {
      if (event.target.matches(".mindmap-node") && event.target.open) updateInspector(event.target);
    },
    true,
  );

  filterTree();
})();
