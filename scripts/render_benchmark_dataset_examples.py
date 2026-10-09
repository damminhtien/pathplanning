#!/usr/bin/env python3
"""Render a source-faithful gallery from the locally installed datasets."""

from __future__ import annotations

import argparse
import base64
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape

ROOT = Path(__file__).resolve().parent.parent
CANVAS_WIDTH = 1800
CANVAS_HEIGHT = 1320
PANEL_WIDTH = 554
PANEL_HEIGHT = 540
PANEL_GAP = 20
LEFT = 48
TOP = 126
IMAGE_WIDTH = 518
IMAGE_HEIGHT = 350


def require_file(path: Path) -> Path:
    if not path.is_file():
        raise FileNotFoundError(
            f"Required dataset asset not found: {path}. Install the datasets first; "
            "see docs/benchmarks/dataset_installation.md."
        )
    return path


def panel_markup(index: int, title: str, subtitle: str, note: str) -> tuple[str, float, float]:
    col = index % 3
    row = index // 3
    x = LEFT + col * (PANEL_WIDTH + PANEL_GAP)
    y = TOP + row * (PANEL_HEIGHT + PANEL_GAP)
    markup = (
        f'<g class="panel"><rect x="{x}" y="{y}" width="{PANEL_WIDTH}" '
        f'height="{PANEL_HEIGHT}" rx="18" fill="#ffffff" stroke="#d8e0eb"/>'
        f'<text class="panel-title" x="{x + 22}" y="{y + 35}">{escape(title)}</text>'
        f'<text class="panel-subtitle" x="{x + 22}" y="{y + 62}">{escape(subtitle)}</text>'
        f'<text class="panel-note" x="{x + 22}" y="{y + PANEL_HEIGHT - 18}">{escape(note)}</text>'
        "</g>"
    )
    return markup, x + 18, y + 92


def image_markup(x: float, y: float, width: float, height: float, href: str) -> str:
    return (
        f'<image x="{x:.2f}" y="{y:.2f}" width="{width:.2f}" height="{height:.2f}" '
        f'href="{href}" preserveAspectRatio="xMidYMid meet"/>'
    )


def fit_bounds(
    x: float,
    y: float,
    width: float,
    height: float,
    data_width: float,
    data_height: float,
    padding: float = 8,
) -> tuple[float, float, float]:
    scale = min(
        (width - 2 * padding) / max(data_width, 1.0),
        (height - 2 * padding) / max(data_height, 1.0),
    )
    drawn_width = data_width * scale
    drawn_height = data_height * scale
    return x + (width - drawn_width) / 2, y + (height - drawn_height) / 2, scale


def draw_grid(
    x: float,
    y: float,
    width: float,
    height: float,
    cells: list[bytearray],
    free_color: str = "#f1f5f9",
    blocked_color: str = "#26364a",
) -> str:
    rows = len(cells)
    cols = len(cells[0]) if rows else 0
    if not rows or not cols:
        raise ValueError("Cannot draw an empty grid")
    origin_x, origin_y, scale = fit_bounds(x, y, width, height, cols, rows)
    cell = scale
    pieces = [
        f'<rect x="{origin_x:.2f}" y="{origin_y:.2f}" width="{cols * cell:.2f}" height="{rows * cell:.2f}" fill="{free_color}"/>'
    ]
    for row_index, row in enumerate(cells):
        start = 0
        while start < cols:
            value = row[start]
            end = start + 1
            while end < cols and row[end] == value:
                end += 1
            if value:
                pieces.append(
                    f'<rect x="{origin_x + start * cell:.2f}" '
                    f'y="{origin_y + row_index * cell:.2f}" '
                    f'width="{(end - start) * cell:.2f}" height="{cell:.2f}" '
                    f'fill="{blocked_color}"/>'
                )
            start = end
    pieces.append(
        f'<rect x="{origin_x:.2f}" y="{origin_y:.2f}" width="{cols * cell:.2f}" '
        f'height="{rows * cell:.2f}" fill="none" stroke="#94a3b8" stroke-width="1"/>'
    )
    return "".join(pieces)


def parse_voxel_slice(path: Path) -> tuple[list[bytearray], int, int, int, int, int]:
    """Return the most populated z slice for a voxel obstacle-list file."""
    require_file(path)
    with path.open(encoding="ascii") as source:
        header = source.readline().split()
        if len(header) != 4 or header[0] not in {"voxel", "rev_voxel"}:
            raise ValueError(f"Unsupported voxel header in {path}: {' '.join(header)}")
        width, height, depth = map(int, header[1:])
        z_counts = [0] * depth
        for line in source:
            parts = line.split()
            if len(parts) != 3:
                continue
            _, _, z = map(int, parts)
            if 0 <= z < depth:
                z_counts[z] += 1
        slice_z = max(range(depth), key=z_counts.__getitem__)

    blocked = [bytearray(width) for _ in range(height)]
    with path.open(encoding="ascii") as source:
        next(source)
        for line in source:
            parts = line.split()
            if len(parts) != 3:
                continue
            voxel_x, voxel_y, voxel_z = map(int, parts)
            if voxel_z != slice_z or not (0 <= voxel_x < width and 0 <= voxel_y < height):
                continue
            # rev_voxel lists traversable cells; voxel lists blocked cells.
            blocked[voxel_y][voxel_x] = 0 if header[0] == "rev_voxel" else 1
    if header[0] == "rev_voxel":
        for row in blocked:
            row[:] = bytearray(1 - value for value in row)
    return blocked, width, height, depth, slice_z, z_counts[slice_z]


def dimacs_local_view(graph_path: Path, coordinates_path: Path) -> tuple[str, str]:
    require_file(graph_path)
    require_file(coordinates_path)
    coords: dict[int, tuple[int, int]] = {}
    for line in coordinates_path.read_text(encoding="ascii").splitlines():
        fields = line.split()
        if len(fields) == 4 and fields[0] == "v":
            coords[int(fields[1])] = (int(fields[2]), int(fields[3]))
    if not coords:
        raise ValueError(f"No DIMACS coordinates found in {coordinates_path}")

    xs = sorted(point[0] for point in coords.values())
    ys = sorted(point[1] for point in coords.values())
    center = len(xs) // 2
    low = max(0, center - int(len(xs) * 0.02))
    high = min(len(xs) - 1, center + int(len(xs) * 0.02))
    x_low, x_high = xs[low], xs[high]
    y_low, y_high = ys[low], ys[high]
    selected = {
        node_id
        for node_id, (coord_x, coord_y) in coords.items()
        if x_low <= coord_x <= x_high and y_low <= coord_y <= y_high
    }
    arcs: list[tuple[int, int]] = []
    for line in graph_path.read_text(encoding="ascii").splitlines():
        fields = line.split()
        if len(fields) == 4 and fields[0] == "a":
            tail, head = int(fields[1]), int(fields[2])
            if tail in selected and head in selected:
                arcs.append((tail, head))

    view_x = LEFT + (PANEL_WIDTH + PANEL_GAP) + 18
    view_y = TOP + 92
    view_width = IMAGE_WIDTH
    view_height = IMAGE_HEIGHT
    graph_xs = [coords[node][0] for node in selected]
    graph_ys = [coords[node][1] for node in selected]
    x_min, x_max = min(graph_xs), max(graph_xs)
    y_min, y_max = min(graph_ys), max(graph_ys)
    origin_x, origin_y, scale = fit_bounds(
        view_x, view_y, view_width, view_height, x_max - x_min, y_max - y_min, padding=10
    )
    points = {
        node: (
            origin_x + (coord_x - x_min) * scale,
            origin_y + (y_max - coord_y) * scale,
        )
        for node, (coord_x, coord_y) in coords.items()
        if node in selected
    }
    pieces = [
        f'<rect x="{view_x}" y="{view_y}" width="{view_width}" height="{view_height}" fill="#f7f9fc"/>'
    ]
    for tail, head in arcs:
        x1, y1 = points[tail]
        x2, y2 = points[head]
        pieces.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            'stroke="#5383ae" stroke-opacity="0.38" stroke-width="1"/>'
        )
    for px, py in points.values():
        pieces.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="1.8" fill="#214d76"/>')
    pieces.append(
        f'<rect x="{view_x}" y="{view_y}" width="{view_width}" height="{view_height}" '
        'fill="none" stroke="#c4cfdd" stroke-width="1"/>'
    )
    note = f"Induced spatial crop: {len(selected):,} vertices, {len(arcs):,} directed arcs; arrowheads omitted"
    return "".join(pieces), note


def barn_circles(path: Path, x: float, y: float, width: float, height: float) -> tuple[str, int]:
    require_file(path)
    root = ET.parse(path).getroot()
    circles: list[tuple[float, float, float]] = []
    for model in root.iter("model"):
        pose = model.find("pose")
        if pose is None or not pose.text:
            continue
        position = [float(value) for value in pose.text.split()[:3]]
        for collision in model.findall(".//collision"):
            cylinder = collision.find("geometry/cylinder")
            if cylinder is None:
                continue
            radius_text = cylinder.findtext("radius")
            if radius_text:
                circles.append((position[0], position[1], float(radius_text)))
    if not circles:
        raise ValueError(f"No cylinder obstacles found in {path}")

    x_min = min(cx - radius for cx, _, radius in circles)
    x_max = max(cx + radius for cx, _, radius in circles)
    y_min = min(cy - radius for _, cy, radius in circles)
    y_max = max(cy + radius for _, cy, radius in circles)
    # Swap the axes for a wide card while preserving the circular geometry.
    origin_x, origin_y, scale = fit_bounds(
        x, y, width, height, y_max - y_min, x_max - x_min, padding=14
    )
    pieces = [f'<rect x="{x}" y="{y}" width="{width}" height="{height}" fill="#f4f7f8"/>']
    for center_x, center_y, radius in circles:
        px = origin_x + (center_y - y_min) * scale
        py = origin_y + (x_max - center_x) * scale
        pieces.append(
            f'<circle cx="{px:.2f}" cy="{py:.2f}" r="{radius * scale:.2f}" '
            'fill="#e15b54" fill-opacity="0.72" stroke="#a83232" stroke-width="0.7"/>'
        )
    pieces.append(
        f'<rect x="{x}" y="{y}" width="{width}" height="{height}" '
        'fill="none" stroke="#c4cfdd" stroke-width="1"/>'
    )
    return "".join(pieces), len(circles)


def read_ppm(path: Path) -> tuple[int, int, int, bytes]:
    """Read a binary P6 portable pixmap (with comments allowed in its header)."""
    raw = require_file(path).read_bytes()
    position = 0

    def token() -> bytes:
        nonlocal position
        while position < len(raw):
            if raw[position] in b" \t\r\n":
                position += 1
            elif raw[position] == ord("#"):
                while position < len(raw) and raw[position] not in b"\r\n":
                    position += 1
            else:
                break
        start = position
        while position < len(raw) and raw[position] not in b" \t\r\n":
            position += 1
        value = raw[start:position]
        if position < len(raw):
            if raw[position : position + 2] == b"\r\n":
                position += 2
            else:
                position += 1
        return value

    magic = token()
    width = int(token())
    height = int(token())
    max_value = int(token())
    if magic != b"P6" or max_value > 255:
        raise ValueError(f"Expected an 8-bit binary P6 image in {path}")
    return width, height, max_value, raw[position:]


def render_ppm(x: float, y: float, width: float, height: float, path: Path) -> str:
    image_width, image_height, max_value, pixels = read_ppm(path)
    if len(pixels) < image_width * image_height * 3:
        raise ValueError(f"Truncated PPM pixel data in {path}")
    sample_size = 112
    origin_x, origin_y, scale = fit_bounds(
        x, y, width, height, image_width, image_height, padding=8
    )
    sample_step = max(image_width / sample_size, image_height / sample_size)
    cols = max(1, round(image_width / sample_step))
    rows = max(1, round(image_height / sample_step))
    cell_width = image_width / cols * scale
    cell_height = image_height / rows * scale
    pieces = [f'<rect x="{x}" y="{y}" width="{width}" height="{height}" fill="#202b36"/>']
    for row in range(rows):
        source_y = min(image_height - 1, int((row + 0.5) * image_height / rows))
        for col in range(cols):
            source_x = min(image_width - 1, int((col + 0.5) * image_width / cols))
            offset = (source_y * image_width + source_x) * 3
            red, green, blue = pixels[offset : offset + 3]
            if max_value != 255:
                red, green, blue = (
                    round(channel * 255 / max_value) for channel in (red, green, blue)
                )
            pieces.append(
                f'<rect x="{origin_x + col * cell_width:.2f}" '
                f'y="{origin_y + row * cell_height:.2f}" width="{cell_width + 0.2:.2f}" '
                f'height="{cell_height + 0.2:.2f}" fill="#{red:02x}{green:02x}{blue:02x}"/>'
            )
    pieces.append(
        f'<rect x="{x}" y="{y}" width="{width}" height="{height}" '
        'fill="none" stroke="#c4cfdd" stroke-width="1"/>'
    )
    return "".join(pieces)


def render_figure(dataset_root: Path) -> str:
    gallery = require_file(ROOT / "assets/images/movingai-maze-scenario.png")
    maze_uri = "data:image/png;base64," + base64.b64encode(gallery.read_bytes()).decode("ascii")
    svg: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{CANVAS_WIDTH}" '
        f'height="{CANVAS_HEIGHT}" viewBox="0 0 {CANVAS_WIDTH} {CANVAS_HEIGHT}">',
        "<style>"
        ".title{font:700 30px system-ui,-apple-system,sans-serif;fill:#142238}"
        ".subtitle{font:15px system-ui,-apple-system,sans-serif;fill:#52647a}"
        ".panel-title{font:650 19px system-ui,-apple-system,sans-serif;fill:#17283c}"
        ".panel-subtitle{font:13px system-ui,-apple-system,sans-serif;fill:#52647a}"
        ".panel-note{font:12px system-ui,-apple-system,sans-serif;fill:#42546a}"
        ".footer{font:13px system-ui,-apple-system,sans-serif;fill:#52647a}"
        "</style>",
        '<rect width="100%" height="100%" fill="#edf2f7"/>',
        '<text class="title" x="48" y="52">Benchmark datasets contain different planning problems</text>',
        '<text class="subtitle" x="48" y="82">Examples come from the installed source files; panels use separate scales and are not a statistical sample.</text>',
    ]

    panel, image_x, image_y = panel_markup(
        0,
        "MovingAI land · maze512-1-0",
        "Implicit 2D grid · octile movement and no corner cutting",
        "Real map, scenario endpoints, and illustrative search path",
    )
    svg.extend((panel, image_markup(image_x, image_y, IMAGE_WIDTH, IMAGE_HEIGHT, maze_uri)))

    graph, graph_note = dimacs_local_view(
        dataset_root / "dimacs/distance/USA-road-d.NY.gr",
        dataset_root / "dimacs/coordinates/USA-road-d.NY.co",
    )
    panel, _, _ = panel_markup(
        1,
        "DIMACS · New York road network",
        "Explicit directed weighted graph · coordinates are metadata",
        graph_note,
    )
    svg.extend((panel, graph))

    panel, image_x, image_y = panel_markup(
        2,
        "MovingAI · Complex.3dmap",
        "Voxel occupancy · most populated horizontal slice",
        "A single z slice is shown; the source asset remains 3D",
    )
    svg.append(panel)
    grid, width, height, depth, slice_z, _ = parse_voxel_slice(
        dataset_root / "movingai-3d/Complex.3dmap"
    )
    svg.append(draw_grid(image_x, image_y, IMAGE_WIDTH, IMAGE_HEIGHT, grid))
    svg.append(
        f'<text x="{image_x + 8}" y="{image_y + IMAGE_HEIGHT - 8}" '
        'font-family="system-ui,sans-serif" font-size="12" fill="#52647a">'
        f"{width} × {height} × {depth} voxels · z = {slice_z}</text>"
    )

    panel, image_x, image_y = panel_markup(
        3,
        "Monash · Descent level02",
        "Voxel map · game-level geometry",
        "One horizontal slice; source query metadata is separate",
    )
    svg.append(panel)
    grid, width, height, depth, slice_z, _ = parse_voxel_slice(
        dataset_root / "monash/descent/level02.3dmap"
    )
    svg.append(draw_grid(image_x, image_y, IMAGE_WIDTH, IMAGE_HEIGHT, grid))
    svg.append(
        f'<text x="{image_x + 8}" y="{image_y + IMAGE_HEIGHT - 8}" '
        'font-family="system-ui,sans-serif" font-size="12" fill="#52647a">'
        f"{width} × {height} × {depth} voxels · z = {slice_z}</text>"
    )

    panel, image_x, image_y = panel_markup(
        4,
        "BARN · world_11.world",
        "Static cylinder obstacles · source XY geometry",
        "Point-robot illustration; no path or planner score is implied",
    )
    svg.append(panel)
    circles, circle_count = barn_circles(
        dataset_root / "barn/worlds/BARN/world_11.world",
        image_x,
        image_y,
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
    )
    svg.append(circles)
    svg.append(
        f'<text x="{image_x + 8}" y="{image_y + IMAGE_HEIGHT - 8}" '
        'font-family="system-ui,sans-serif" font-size="12" fill="#52647a">'
        f"{circle_count} source cylinders · axes rotated to fit the panel"
        "</text>"
    )

    panel, image_x, image_y = panel_markup(
        5,
        "OMPL resource · floor.ppm",
        "Framework image resource · not a benchmark result",
        "OMPL.app configs require matching collision and state-space semantics",
    )
    svg.append(panel)
    svg.append(
        render_ppm(
            image_x,
            image_y,
            IMAGE_WIDTH,
            IMAGE_HEIGHT,
            dataset_root / "ompl/ompl/tests/resources/ppm/floor.ppm",
        )
    )

    svg.extend(
        (
            '<text class="footer" x="48" y="1270">Keep grid-optimal, any-angle, directed-graph, voxel, geometric, and framework-resource cohorts separate.</text>',
            '<text class="footer" x="48" y="1294">The pictures describe input structure; visual density does not rank difficulty, represent coverage, or establish planner performance.</text>',
            "</svg>",
        )
    )
    return "\n".join(svg)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=Path("benchmark-results/datasets"),
        help="Root of the locally installed benchmark datasets",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/benchmarks/images/benchmark_dataset_examples.svg"),
        help="SVG output path",
    )
    parser.add_argument(
        "--png-output", type=Path, help="Optional PNG output path (requires rsvg-convert)"
    )
    args = parser.parse_args()

    dataset_root = args.dataset_root
    if not dataset_root.is_absolute():
        dataset_root = Path.cwd() / dataset_root
    svg_text = render_figure(dataset_root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(svg_text + "\n", encoding="utf-8")
    print(f"Wrote {args.output}")

    if args.png_output:
        converter = shutil.which("rsvg-convert")
        if converter is None:
            print("rsvg-convert is required for --png-output", file=sys.stderr)
            return 2
        args.png_output.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            [
                converter,
                "--width",
                str(CANVAS_WIDTH),
                "--height",
                str(CANVAS_HEIGHT),
                "--output",
                str(args.png_output),
                str(args.output),
            ],
            check=True,
        )
        print(f"Wrote {args.png_output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError, ET.ParseError) as error:
        print(error, file=sys.stderr)
        raise SystemExit(2) from error
