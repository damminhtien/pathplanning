import { drawMotionMarker, drawPathTrace, pathAtProgress } from "./shared.js";

export function createMultiAgentScene({ context, state }) {
  function drawMultiAgent(width, height) {
    const cols = 11;
    const rows = 6;
    const gapX = (width - 88) / (cols - 1);
    const gapY = (height - 90) / (rows - 1);
    const ox = 44;
    const oy = 45;
    const node = (x, y) => ({ x: ox + x * gapX, y: oy + y * gapY });
    for (let y = 0; y < rows; y += 1)
      for (let x = 0; x < cols; x += 1) {
        if (x < cols - 1) drawNetworkEdge(node(x, y), node(x + 1, y));
        if (y < rows - 1) drawNetworkEdge(node(x, y), node(x, y + 1));
      }
    const paths = [
      {
        color: "#83f0cd",
        label: "A",
        nodes: [
          [0, 0],
          [2, 0],
          [2, 2],
          [5, 2],
          [5, 4],
          [10, 4],
        ],
      },
      {
        color: "#ffb875",
        label: "B",
        nodes: [
          [10, 0],
          [8, 0],
          [8, 1],
          [5, 1],
          [5, 3],
          [1, 3],
          [1, 5],
        ],
      },
      {
        color: "#b4a1ff",
        label: "C",
        nodes: [
          [0, 5],
          [3, 5],
          [3, 4],
          [7, 4],
          [7, 2],
          [10, 2],
        ],
      },
    ];
    paths.forEach((agent, index) => {
      const points = agent.nodes.map(([x, y]) => node(x, y));
      context.beginPath();
      points.forEach((point, i) => {
        if (!i) context.moveTo(point.x, point.y);
        else context.lineTo(point.x, point.y);
      });
      context.strokeStyle = `${agent.color}65`;
      context.lineWidth = 2.2;
      context.setLineDash([6, 5]);
      context.lineDashOffset = -state.elapsed * 14;
      context.stroke();
      context.setLineDash([]);
      points.forEach((point, i) => {
        if (i % 2 === 0 || i === points.length - 1) {
          context.beginPath();
          context.arc(point.x, point.y, 3, 0, Math.PI * 2);
          context.fillStyle = `${agent.color}88`;
          context.fill();
        }
      });
      const progress = Math.min(1, state.progress * 1.12 + index * 0.09);
      context.save();
      context.strokeStyle = agent.color;
      context.globalAlpha = 0.78;
      context.lineWidth = 3.2;
      context.lineCap = "round";
      context.lineJoin = "round";
      drawPathTrace(context, points, progress);
      context.restore();
      const position = pathAtProgress(points, progress);
      drawMotionMarker(
        context,
        position,
        agent.color,
        state.elapsed + index * 0.65,
        agent.label,
        8,
      );
    });
    context.textAlign = "left";
    context.textBaseline = "alphabetic";
    context.fillStyle = "#9aa9bd";
    context.font = "9px ui-monospace, monospace";
    const phase = Math.min(2, Math.floor(state.progress * 3));
    const phases =
      state.planner.id === "eecbs"
        ? ["01  FIND A CONFLICT", "02  ADD A CONSTRAINT", "03  REPLAN THE AFFECTED AGENTS"]
        : ["01  PLAN JOINT ROUTES", "02  CHECK VERTEX + EDGE SWAPS", "03  COMMIT SAFE MOTION"];
    context.fillStyle = phase === 1 ? "#ffd0a2" : "#b9daf5";
    context.fillText(phases[phase], 16, height - 17);
  }

  function drawNetworkEdge(a, b) {
    context.beginPath();
    context.moveTo(a.x, a.y);
    context.lineTo(b.x, b.y);
    context.strokeStyle = "rgba(161, 185, 213, .12)";
    context.lineWidth = 1;
    context.stroke();
    context.beginPath();
    context.arc(a.x, a.y, 2.2, 0, Math.PI * 2);
    context.fillStyle = "rgba(190, 211, 234, .44)";
    context.fill();
    context.beginPath();
    context.arc(b.x, b.y, 2.2, 0, Math.PI * 2);
    context.fill();
  }
  return drawMultiAgent;
}
