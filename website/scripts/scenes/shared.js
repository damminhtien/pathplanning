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

export function pathAtProgress(points, progress) {
  const total = points.slice(1).reduce((sum, point, index) => sum + distance(point, points[index]), 0);
  let remaining = total * progress;
  for (let index = 1; index < points.length; index += 1) {
    const length = distance(points[index - 1], points[index]);
    if (remaining <= length) {
      return {
        x: points[index - 1].x + (points[index].x - points[index - 1].x) * remaining / length,
        y: points[index - 1].y + (points[index].y - points[index - 1].y) * remaining / length,
      };
    }
    remaining -= length;
  }
  return points.at(-1);
}
