"use strict";

export function roundedRect(context, x, y, width, height, radius) {
  const r = Math.max(0, Math.min(radius, width / 2, height / 2));
  context.beginPath();
  context.moveTo(x + r, y);
  context.arcTo(x + width, y, x + width, y + height, r);
  context.arcTo(x + width, y + height, x, y + height, r);
  context.arcTo(x, y + height, x, y, r);
  context.arcTo(x, y, x + width, y, r);
  context.closePath();
  context.fill();
}

export function drawTinyLabel(context, text, x, y, color) {
  context.font = "10px ui-monospace, SFMono-Regular, Menlo, monospace";
  context.textAlign = "left";
  context.textBaseline = "alphabetic";
  context.fillStyle = color;
  context.fillText(text, x, y);
}

export function distance(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

export function drawPathTrace(context, points, progress) {
  if (points.length < 2) return;

  let remaining = points.slice(1).reduce(
    (sum, point, index) => sum + distance(point, points[index]),
    0,
  ) * Math.max(0, Math.min(1, progress));

  context.beginPath();
  context.moveTo(points[0].x, points[0].y);
  for (let index = 1; index < points.length; index += 1) {
    const from = points[index - 1];
    const to = points[index];
    const length = distance(from, to);
    if (!length) continue;
    if (remaining >= length) {
      context.lineTo(to.x, to.y);
      remaining -= length;
      continue;
    }
    const fraction = remaining / length;
    context.lineTo(from.x + (to.x - from.x) * fraction, from.y + (to.y - from.y) * fraction);
    break;
  }
  context.stroke();
}

export function drawMotionMarker(context, point, color, phase, label = "", radius = 7) {
  const pulse = (Math.sin(phase * 5) + 1) / 2;
  context.save();
  context.globalAlpha = .2 + pulse * .18;
  context.beginPath();
  context.arc(point.x, point.y, radius + 5 + pulse * 4, 0, Math.PI * 2);
  context.strokeStyle = color;
  context.lineWidth = 1.5;
  context.stroke();
  context.globalAlpha = 1;
  context.shadowColor = color;
  context.shadowBlur = 8 + pulse * 9;
  context.beginPath();
  context.arc(point.x, point.y, radius, 0, Math.PI * 2);
  context.fillStyle = color;
  context.fill();
  context.shadowBlur = 0;
  if (label) {
    context.fillStyle = "#10202a";
    context.font = `700 ${Math.max(8, radius * 1.1)}px ui-monospace, monospace`;
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText(label, point.x, point.y + .5);
  }
  context.restore();
}

export function pathAtProgress(points, progress) {
  if (!points.length) return { x: 0, y: 0 };
  const total = points.slice(1).reduce((sum, point, index) => sum + distance(point, points[index]), 0);
  let remaining = total * Math.max(0, Math.min(1, progress));
  for (let index = 1; index < points.length; index += 1) {
    const from = points[index - 1];
    const length = distance(from, points[index]);
    if (!length) continue;
    if (remaining <= length) {
      return {
        x: from.x + (points[index].x - from.x) * remaining / length,
        y: from.y + (points[index].y - from.y) * remaining / length,
      };
    }
    remaining -= length;
  }
  return points.at(-1);
}
