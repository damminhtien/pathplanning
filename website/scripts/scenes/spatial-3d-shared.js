export function createProjection(width, height, volume) {
  const xUnit = Math.min(
    (width - 44) / (volume.x + volume.y),
    (height - 38) / ((volume.x + volume.y) * 0.45 + volume.z * 0.95),
  );
  const yUnit = xUnit * 0.45;
  const zUnit = xUnit * 0.95;
  const originX = width / 2 + ((volume.y - volume.x) * xUnit) / 2;
  const projectedHeight = (volume.x + volume.y) * yUnit + volume.z * zUnit;
  const originY = (height - projectedHeight) / 2 + volume.z * zUnit;
  return {
    point(x, y, z) {
      return {
        x: originX + (x - y) * xUnit,
        y: originY + (x + y) * yUnit - z * zUnit,
      };
    },
  };
}

function drawPolygon(context, points, fill, stroke = "rgba(74, 105, 135, .34)") {
  context.beginPath();
  context.moveTo(points[0].x, points[0].y);
  points.slice(1).forEach((point) => {
    context.lineTo(point.x, point.y);
  });
  context.closePath();
  context.fillStyle = fill;
  context.fill();
  context.strokeStyle = stroke;
  context.stroke();
}

export function drawGrid(context, projection, volume) {
  context.lineWidth = 0.8;
  for (let z = 0; z <= volume.z; z += 1) {
    context.strokeStyle = z === 0 ? "rgba(70, 108, 145, .24)" : "rgba(84, 126, 164, .12)";
    for (let x = 0; x <= volume.x; x += 1) {
      const from = projection.point(x, 0, z);
      const to = projection.point(x, volume.y, z);
      context.beginPath();
      context.moveTo(from.x, from.y);
      context.lineTo(to.x, to.y);
      context.stroke();
    }
    for (let y = 0; y <= volume.y; y += 1) {
      const from = projection.point(0, y, z);
      const to = projection.point(volume.x, y, z);
      context.beginPath();
      context.moveTo(from.x, from.y);
      context.lineTo(to.x, to.y);
      context.stroke();
    }
  }

  context.strokeStyle = "rgba(61, 104, 148, .3)";
  for (const [x, y] of [
    [0, 0],
    [volume.x, 0],
    [0, volume.y],
    [volume.x, volume.y],
  ]) {
    const from = projection.point(x, y, 0);
    const to = projection.point(x, y, volume.z);
    context.beginPath();
    context.moveTo(from.x, from.y);
    context.lineTo(to.x, to.y);
    context.stroke();
  }
}

export function drawVoxel(context, projection, x, y, z) {
  const top = [
    projection.point(x, y, z + 1),
    projection.point(x + 1, y, z + 1),
    projection.point(x + 1, y + 1, z + 1),
    projection.point(x, y + 1, z + 1),
  ];
  drawPolygon(
    context,
    [projection.point(x, y + 1, z), projection.point(x + 1, y + 1, z), top[2], top[3]],
    "#91abc2",
  );
  drawPolygon(
    context,
    [projection.point(x + 1, y, z), projection.point(x + 1, y + 1, z), top[2], top[1]],
    "#7592ad",
  );
  drawPolygon(context, top, "#d8e7f4", "rgba(69, 103, 134, .45)");
}

export function drawPrism(context, projection, box) {
  const [x, y, z] = box.min;
  const [maxX, maxY, maxZ] = box.max;
  const top = [
    projection.point(x, y, maxZ),
    projection.point(maxX, y, maxZ),
    projection.point(maxX, maxY, maxZ),
    projection.point(x, maxY, maxZ),
  ];
  drawPolygon(
    context,
    [projection.point(x, maxY, z), projection.point(maxX, maxY, z), top[2], top[3]],
    "rgba(117, 149, 180, .78)",
    "rgba(57, 88, 119, .52)",
  );
  drawPolygon(
    context,
    [projection.point(maxX, y, z), projection.point(maxX, maxY, z), top[2], top[1]],
    "rgba(85, 119, 153, .88)",
    "rgba(57, 88, 119, .58)",
  );
  drawPolygon(context, top, "rgba(211, 228, 243, .9)", "rgba(57, 88, 119, .58)");
}

export function drawPoint(context, point, color, radius = 3) {
  context.beginPath();
  context.arc(point.x, point.y, radius + 3, 0, Math.PI * 2);
  context.fillStyle = `${color}22`;
  context.fill();
  context.beginPath();
  context.arc(point.x, point.y, radius, 0, Math.PI * 2);
  context.fillStyle = color;
  context.fill();
  context.lineWidth = 1.5;
  context.strokeStyle = "#fff";
  context.stroke();
}

export function drawPath(context, projection, points, progress, color) {
  if (points.length < 2 || progress <= 0) return;
  const screenPoints = points.map(([x, y, z]) => projection.point(x, y, z));
  const lengths = screenPoints
    .slice(1)
    .map((point, index) =>
      Math.hypot(point.x - screenPoints[index].x, point.y - screenPoints[index].y),
    );
  const fullLength = lengths.reduce((sum, value) => sum + value, 0);
  let remaining = fullLength * Math.min(progress, 1);
  const trace = [screenPoints[0]];
  for (let index = 0; index < lengths.length && remaining > 0; index += 1) {
    const from = screenPoints[index];
    const to = screenPoints[index + 1];
    const span = lengths[index];
    if (remaining >= span) trace.push(to);
    else {
      const ratio = remaining / span;
      trace.push({
        x: from.x + (to.x - from.x) * ratio,
        y: from.y + (to.y - from.y) * ratio,
      });
    }
    remaining -= span;
  }

  context.save();
  context.lineJoin = "round";
  context.lineCap = "round";
  context.strokeStyle = `${color}35`;
  context.lineWidth = 7;
  context.beginPath();
  trace.forEach((point, index) => {
    if (index) {
      context.lineTo(point.x, point.y);
    } else {
      context.moveTo(point.x, point.y);
    }
  });
  context.stroke();
  context.strokeStyle = color;
  context.lineWidth = 2.5;
  context.stroke();
  context.restore();
  return trace[trace.length - 1];
}

export function pointAlongPolyline(projection, points, progress) {
  if (!points.length) return null;
  const screenPoints = points.map(([x, y, z]) => projection.point(x, y, z));
  const lengths = screenPoints
    .slice(1)
    .map((point, index) =>
      Math.hypot(point.x - screenPoints[index].x, point.y - screenPoints[index].y),
    );
  let remaining =
    lengths.reduce((sum, value) => sum + value, 0) * Math.min(Math.max(progress, 0), 1);
  for (let index = 0; index < lengths.length; index += 1) {
    const from = screenPoints[index];
    const to = screenPoints[index + 1];
    const span = lengths[index];
    if (remaining <= span) {
      const ratio = span ? remaining / span : 0;
      return { x: from.x + (to.x - from.x) * ratio, y: from.y + (to.y - from.y) * ratio };
    }
    remaining -= span;
  }
  return screenPoints[screenPoints.length - 1];
}

export function shuttleProgress(elapsed, period = 3.2) {
  const cycle = ((elapsed % period) / period) * 2;
  return cycle <= 1 ? cycle : 2 - cycle;
}

export function drawLabel(context, text, x, y, color = "#54728f") {
  context.font = "600 10px ui-monospace, SFMono-Regular, Menlo, monospace";
  context.letterSpacing = "0.08em";
  context.fillStyle = color;
  context.fillText(text, x, y);
}

export function drawVolumeFrame(context, projection, volume) {
  context.save();
  context.lineWidth = 0.8;
  context.strokeStyle = "rgba(72, 112, 151, .22)";
  for (let x = 0; x <= volume.x; x += 2) {
    for (const z of [0, volume.z]) {
      const from = projection.point(x, 0, z);
      const to = projection.point(x, volume.y, z);
      context.beginPath();
      context.moveTo(from.x, from.y);
      context.lineTo(to.x, to.y);
      context.stroke();
    }
  }
  for (let y = 0; y <= volume.y; y += 2) {
    for (const z of [0, volume.z]) {
      const from = projection.point(0, y, z);
      const to = projection.point(volume.x, y, z);
      context.beginPath();
      context.moveTo(from.x, from.y);
      context.lineTo(to.x, to.y);
      context.stroke();
    }
  }
  for (const [x, y] of [
    [0, 0],
    [volume.x, 0],
    [0, volume.y],
    [volume.x, volume.y],
  ]) {
    const from = projection.point(x, y, 0);
    const to = projection.point(x, y, volume.z);
    context.beginPath();
    context.moveTo(from.x, from.y);
    context.lineTo(to.x, to.y);
    context.stroke();
  }
  context.restore();
}
