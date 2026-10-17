import { distance, drawTinyLabel as drawCanvasLabel, roundedRect as drawRoundedRect } from "./shared.js";


export function createContinuousScene({ context, state, planners }) {
  const roundedRect = (x, y, width, height, radius) => drawRoundedRect(context, x, y, width, height, radius);
  const drawTinyLabel = (text, x, y, color) => drawCanvasLabel(context, text, x, y, color);

  function seededRandom(seed) {
    let value = seed >>> 0;
    return () => {
      value = (value * 1664525 + 1013904223) >>> 0;
      return value / 4294967296;
    };
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
    if (planner.id === "abit_star" || planner.id === "bit_star" || planner.id === "fcit_star") drawTinyLabel("BATCH, EDGE QUEUE, INCUMBENT", 15, 19, "#9bb9d8");
    if (planner.id === "eirm_star") drawTinyLabel("ROADMAP CACHE, reusable valid or blocked edges", 15, 19, "#9bb9d8");
    if (planner.id === "rit_star") drawTinyLabel("metric-shaped neighborhood", width * .48, height * .9, "#c2afff");
    drawCandidatePath(start, goal, obstacles, width, height, state.progress);
    drawPoint(start.x, start.y, "#83f0cd", "START");
    drawPoint(goal.x, goal.y, "#ffb875", "GOAL");
    if (planner.id === "hybrid_astar" || planner.id === "state_lattice") drawVehicle(width * (.12 + .7 * state.progress), height * (.73 - .42 * state.progress), -0.52);
    if (twoTrees && state.progress > .72) drawTinyLabel("TREES CONNECT", width * .43, height * .84, "#bff9e7");
  }

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
  return drawContinuous;
}
