import {
  drawTinyLabel as drawCanvasLabel,
  drawMotionMarker,
  drawPathTrace,
  roundedRect as drawRoundedRect,
  pathAtProgress,
} from "./shared.js";

export function createTemporalScene({ context, state }) {
  const roundedRect = (x, y, width, height, radius) =>
    drawRoundedRect(context, x, y, width, height, radius);
  const drawTinyLabel = (text, x, y, color) => drawCanvasLabel(context, text, x, y, color);

  function drawTemporal(width, height) {
    const lanes = ["Dock", "Hall", "Lift", "Goal"];
    const left = 80;
    const right = width - 24;
    const top = 54;
    const laneGap = (height - 110) / (lanes.length - 1);
    context.font = "10px ui-monospace, monospace";
    context.textBaseline = "middle";
    for (let i = 0; i < lanes.length; i += 1) {
      const y = top + i * laneGap;
      context.fillStyle = "#a7b6ca";
      context.textAlign = "right";
      context.fillText(lanes[i], left - 15, y);
      context.beginPath();
      context.moveTo(left, y);
      context.lineTo(right, y);
      context.strokeStyle = "rgba(164, 189, 220, .18)";
      context.lineWidth = 1;
      context.stroke();
      context.beginPath();
      context.arc(left, y, 4, 0, Math.PI * 2);
      context.fillStyle = i === 0 ? "#83f0cd" : i === lanes.length - 1 ? "#ffb875" : "#8092aa";
      context.fill();
    }
    const blocks = [
      { lane: 1, from: 0.18, to: 0.53 },
      { lane: 2, from: 0.48, to: 0.76 },
      { lane: 1, from: 0.74, to: 0.9 },
    ];
    blocks.forEach((block) => {
      const y = top + block.lane * laneGap - 11;
      const x = left + (right - left) * block.from;
      const w = (right - left) * (block.to - block.from);
      const active = state.progress >= block.from && state.progress <= block.to;
      context.fillStyle = active ? "rgba(255, 125, 141, .34)" : "rgba(255, 125, 141, .16)";
      roundedRect(x, y, w, 22, 6);
      context.strokeStyle = active ? "rgba(255, 151, 164, .72)" : "rgba(255, 125, 141, .26)";
      context.stroke();
      context.fillStyle = active ? "#ffd0d7" : "#ef9ba7";
      context.font = "8px ui-monospace, monospace";
      context.textAlign = "center";
      context.fillText(active ? "OCCUPIED NOW" : "blocked interval", x + w / 2, y + 11);
    });
    const pathPoints = [0, 0.2, 0.48, 0.48, 0.69, 0.82, 1].map((t, i) => ({
      x: left + t * (right - left),
      y: top + [0, 0, 1, 2, 2, 3, 3][i] * laneGap,
    }));
    const cursorX = left + state.progress * (right - left);
    context.save();
    context.beginPath();
    context.moveTo(cursorX, top - 21);
    context.lineTo(cursorX, top + laneGap * (lanes.length - 1) + 20);
    context.setLineDash([3, 6]);
    context.strokeStyle = "rgba(166, 210, 255, .5)";
    context.lineWidth = 1.2;
    context.stroke();
    context.restore();
    context.save();
    context.strokeStyle = "rgba(131, 240, 205, .96)";
    context.lineWidth = 3;
    context.lineCap = "round";
    context.lineJoin = "round";
    context.shadowColor = "rgba(131, 240, 205, .75)";
    context.shadowBlur = 9;
    context.setLineDash([8, 6]);
    context.lineDashOffset = -state.elapsed * 16;
    drawPathTrace(context, pathPoints, state.progress);
    context.restore();
    const dot = pathAtProgress(pathPoints, state.progress);
    drawMotionMarker(context, dot, "#a4f5df", state.elapsed, "R", 8);
    context.fillStyle = "#8697ad";
    context.font = "9px ui-monospace, monospace";
    context.textAlign = "left";
    context.fillText("TIME", left, height - 25);
    context.textAlign = "right";
    context.fillText("Half-open blocked intervals [start, end)", right, height - 25);
    if (state.planner.id === "kinodynamic_sipp")
      drawTinyLabel(
        "wait only at zero speed; velocity changes along each edge",
        width * 0.4,
        22,
        "#c3b1ff",
      );
    else if (state.planner.id === "bounded_suboptimal_sipp")
      drawTinyLabel("FOCAL: choose within the w-bounded band", width * 0.48, 22, "#c3b1ff");
    else
      drawTinyLabel("wait safely; enter when the next interval opens", width * 0.47, 22, "#a5efd9");
  }

  return drawTemporal;
}
