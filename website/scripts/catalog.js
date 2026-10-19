import { PLANNER_KIND_LABELS } from "./planner-kinds.js";

export function createCatalog(planners, onSelect) {
  let mathJaxReady;

  function renderCatalog() {
    const selectedKind = document.querySelector(".filter-chip.is-active")?.dataset.filter || "all";
    const query = document.querySelector("#planner-search").value.trim().toLowerCase();
    const visible = planners.filter((planner) => {
      const matchesKind = selectedKind === "all" || planner.kind === selectedKind;
      const searchText =
        `${planner.id} ${planner.title} ${planner.principle} ${planner.description}`.toLowerCase();
      return matchesKind && (!query || searchText.includes(query));
    });
    const grid = document.querySelector("#planner-grid");
    grid.innerHTML = "";
    if (!visible.length) {
      grid.innerHTML =
        '<div class="empty-state">No planners match that search. Try another name or idea.</div>';
    }
    for (const planner of visible) {
      const card = document.createElement("article");
      card.className = "planner-card";
      const top = document.createElement("div");
      top.className = "planner-card-top";
      const family = document.createElement("span");
      family.className = "planner-family";
      family.textContent = PLANNER_KIND_LABELS[planner.kind];
      top.append(family);

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
      readLink.textContent = "Read lesson";
      const demoButton = document.createElement("button");
      demoButton.className = "card-demo";
      demoButton.type = "button";
      demoButton.dataset.planner = planner.id;
      demoButton.setAttribute("aria-label", `View the concept animation for ${planner.title}`);
      demoButton.textContent = "View concept";
      actions.append(readLink, demoButton);
      footer.append(principle, actions);
      card.append(top, heading, description, footer);
      grid.append(card);
    }
    document.querySelector("#catalog-count").textContent =
      `Showing ${visible.length} of ${planners.length} planners`;
    typesetMath(grid);
  }

  function typesetMath(element) {
    if (!mathJaxReady) {
      mathJaxReady = new Promise((resolve) => {
        const startup = window.MathJax?.startup?.promise;
        if (startup) {
          startup.then(resolve);
          return;
        }
        document.querySelector("#mathjax-script")?.addEventListener(
          "load",
          () => {
            window.MathJax.startup.promise.then(resolve);
          },
          { once: true },
        );
      });
    }
    mathJaxReady.then(async () => {
      try {
        window.MathJax.typesetClear([element]);
        await window.MathJax.typesetPromise([element]);
      } catch (error) {
        console.error("Could not typeset planner formulas", error);
      }
    });
  }

  document.querySelector("#planner-grid").addEventListener("click", (event) => {
    const button = event.target.closest("[data-planner]");
    if (button) onSelect(button.dataset.planner, true);
  });
  document.querySelectorAll(".filter-chip").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelector(".filter-chip.is-active").classList.remove("is-active");
      button.classList.add("is-active");
      renderCatalog();
    });
  });
  document.querySelector("#planner-search").addEventListener("input", renderCatalog);
  renderCatalog();
}
