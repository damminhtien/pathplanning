import { drawTinyLabel as drawCanvasLabel, pathAtProgress, roundedRect as drawRoundedRect } from "./shared.js";


export function createTemporalScene({ context, state }) {
  const roundedRect = (x, y, width, height, radius) => drawRoundedRect(context, x, y, width, height, radius);
  const drawTinyLabel = (text, x, y, color) => drawCanvasLabel(context, text, x, y, color);

  function drawTemporal(width, height) {
    const lanes = ["Dock", "Hall", "Lift", "Goal"];
    const left = 80; const right = width - 24; const top = 54; const laneGap = (height - 110) / (lanes.length - 1);
    context.font = "10px ui-monospace, monospace"; context.textBaseline = "middle";
    for (let i = 0; i < lanes.length; i += 1) {
      const y = top + i * laneGap;
      context.fillStyle = "#a7b6ca"; context.textAlign = "right"; context.fillText(lanes[i], left - 15, y);
      context.beginPath(); context.moveTo(left, y); context.lineTo(right, y); context.strokeStyle = "rgba(164, 189, 220, .18)"; context.lineWidth = 1; context.stroke();
      context.beginPath(); context.arc(left, y, 4, 0, Math.PI * 2); context.fillStyle = i === 0 ? "#83f0cd" : i === lanes.length - 1 ? "#ffb875" : "#8092aa"; context.fill();
    }
    const blocks = [{ lane: 1, from: .18, to: .53 }, { lane: 2, from: .48, to: .76 }, { lane: 1, from: .74, to: .9 }];
    blocks.forEach((block) => {
      const y = top + block.lane * laneGap - 11;
      const x = left + (right - left) * block.from;
      const w = (right - left) * (block.to - block.from);
      context.fillStyle = "rgba(255, 125, 141, .19)"; roundedRect(context, x, y, w, 22, 6);
      context.strokeStyle = "rgba(255, 125, 141, .29)"; context.stroke();
      context.fillStyle = "#ef9ba7"; context.font = "8px ui-monospace, monospace"; context.textAlign = "center"; context.fillText("blocked", x + w / 2, y + 11);
    });
    const pathPoints = [0, .2, .48, .48, .69, .82, 1].map((t, i) => ({ x: left + t * (right - left), y: top + [0, 0, 1, 2, 2, 3, 3][i] * laneGap }));
    context.beginPath(); pathPoints.forEach((point, index) => { if (!index) context.moveTo(point.x, point.y); else context.lineTo(point.x, point.y); });
    context.strokeStyle = "rgba(131, 240, 205, .8)"; context.lineWidth = 2.5; context.stroke();
    const dot = pathAtProgress(pathPoints, state.progress);
    context.beginPath(); context.arc(dot.x, dot.y, 7, 0, Math.PI * 2); context.fillStyle = "#83f0cd"; context.fill();
    context.beginPath(); context.arc(dot.x, dot.y, 13, 0, Math.PI * 2); context.strokeStyle = "rgba(131, 240, 205, .3)"; context.stroke();
    context.fillStyle = "#8697ad"; context.font = "9px ui-monospace, monospace"; context.textAlign = "left";
    context.fillText("TIME", left, height - 25);
    context.textAlign = "right"; context.fillText("Half-open blocked intervals [start, end)", right, height - 25);
    if (state.planner.id === "kinodynamic_sipp") drawTinyLabel("wait only at zero speed; velocity changes along each edge", width * .4, 22, "#c3b1ff");
    else if (state.planner.id === "bounded_suboptimal_sipp") drawTinyLabel("FOCAL: choose within the w-bounded band", width * .48, 22, "#c3b1ff");
    else drawTinyLabel("wait safely; enter when the next interval opens", width * .47, 22, "#a5efd9");
  }

  return drawTemporal;
}
