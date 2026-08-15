#!/usr/bin/env python3
"""Generate a flamegraph-style HTML visualization from timings.json."""

import argparse
import json
from pathlib import Path


def total_us(node):
    return node["duration_ms"] * 1000.0


def build_flame_data(nodes, depth=0, offset=0.0):
    """Convert timings tree into flat flame frames (offset/end in microseconds)."""
    frames = []
    for node in nodes:
        dur = total_us(node)
        frames.append(
            {
                "name": node["name"],
                "value": dur,
                "start": offset,
                "depth": depth,
            }
        )
        frames.extend(build_flame_data(node.get("children", []), depth + 1, offset))
        offset += dur
    return frames


HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Flame Graph</title>
<style>
  html, body { margin: 0; padding: 0; height: 100%; background: #1e1e1e; color: #ccc;
               font-family: "Segoe UI", Arial, sans-serif; overflow: hidden; }
  #header { padding: 8px 12px; font-size: 14px; color: #9cdcfe; white-space: nowrap;
            overflow: hidden; text-overflow: ellipsis; }
  #controls { position: fixed; bottom: 0; left: 0; right: 0; display: flex; gap: 10px;
              align-items: center; padding: 8px 12px; background: #252526;
              border-top: 1px solid #3c3c3c; font-size: 13px; user-select: none; }
  #controls label { color: #9cdcfe; white-space: nowrap; }
  #frameSlider { flex: 1; accent-color: #569cd6; }
  #frameBox { width: 70px; background: #1e1e1e; color: #ccc; border: 1px solid #3c3c3c;
              border-radius: 3px; padding: 3px 6px; font-size: 13px; text-align: center; }
  #frameInfo { min-width: 130px; text-align: right; color: #b5b5b5; white-space: nowrap; }
  #tooltip { position: fixed; pointer-events: none; background: #2d2d2d; border: 1px solid #555;
             padding: 6px 10px; border-radius: 4px; font-size: 12px; display: none; z-index: 10;
             white-space: nowrap; box-shadow: 0 2px 8px rgba(0,0,0,.5); }
  #tooltip .name { color: #9cdcfe; font-weight: 600; }
  canvas { display: block; cursor: pointer; }
</style>
</head>
<body>
<div id="header">Click a frame to zoom, click background to reset.</div>
<canvas id="canvas"></canvas>
<div id="controls">
  <label for="frameSlider">Frame</label>
  <input type="range" id="frameSlider" min="0" max="0" value="0" step="1">
  <input type="number" id="frameBox" min="0" max="0" value="0" step="1">
  <span id="frameInfo"></span>
</div>
<div id="tooltip"></div>
<script>
// FRAMES_DATA: one flat frame list per top-level entry in timings.json
const FRAMES_DATA = __FRAMES_DATA__;
const DURATIONS_MS = __DURATIONS__;
const N = FRAMES_DATA.length;

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const tooltip = document.getElementById('tooltip');
const header = document.getElementById('header');
const slider = document.getElementById('frameSlider');
const frameBox = document.getElementById('frameBox');
const frameInfo = document.getElementById('frameInfo');
const FRAME_H = 20, GAP = 1;

// Palette (warm flamegraph tones)
function hashColor(str) {
  let h = 0;
  for (let i = 0; i < str.length; i++) h = (h * 31 + str.charCodeAt(i)) >>> 0;
  const hues = [0, 18, 35, 200, 215, 260, 280];
  const h0 = hues[h % hues.length] + (h % 13) - 6;
  const s = 55 + (h % 20);
  const l = 48 + (h % 12);
  return `hsl(${h0}, ${s}%, ${l}%)`;
}
const colorCache = new Map();
function colorFor(name) {
  let c = colorCache.get(name);
  if (!c) { c = hashColor(name); colorCache.set(name, c); }
  return c;
}

let curIndex = 0;
let frames = [];
let TOTAL = 1;
let view = { start: 0, end: TOTAL };   // visible time range in us
let byDepth = new Map();
let dpr = window.devicePixelRatio || 1;

function loadFrame(i) {
  i = Math.max(0, Math.min(N - 1, i | 0));
  curIndex = i;
  frames = FRAMES_DATA[i];
  TOTAL = Math.max(DURATIONS_MS[i] * 1000, 1e-6);
  view = { start: 0, end: TOTAL };
  byDepth = new Map();
  for (const f of frames) {
    if (!byDepth.has(f.depth)) byDepth.set(f.depth, []);
    byDepth.get(f.depth).push(f);
  }
  slider.value = i;
  frameBox.value = i;
  frameInfo.textContent = `of ${N - 1} \\u2014 ${fmtTime(TOTAL)}`;
  header.textContent = 'Click a frame to zoom, click background to reset.';
  draw();
}

function resize() {
  canvas.width = window.innerWidth * dpr;
  const controlsH = document.getElementById('controls').offsetHeight;
  canvas.height = window.innerHeight - header.offsetHeight - controlsH;
  canvas.style.width = window.innerWidth + 'px';
  canvas.style.height = canvas.height / dpr + 'px';
  draw();
}

function fmtTime(us) {
  if (us >= 1000) return (us / 1000).toFixed(3) + ' ms';
  return us.toFixed(1) + ' \\u00b5s';
}

function draw() {
  const W = canvas.width / dpr, H = canvas.height / dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, W, H);
  const span = view.end - view.start;
  ctx.font = '12px "Segoe UI", Arial, sans-serif';
  ctx.textBaseline = 'middle';

  for (const [depth, list] of byDepth) {
    const y = depth * (FRAME_H + GAP);
    if (y > H) break;
    for (const f of list) {
      const x0 = (f.start - view.start) / span * W;
      const x1 = (f.start + f.value - view.start) / span * W;
      if (x1 < 0 || x0 > W) continue;
      const w = Math.max(x1 - x0, 0.2);
      ctx.fillStyle = colorFor(f.name);
      ctx.fillRect(x0, y, w, FRAME_H);
      if (w > 30) {
        ctx.fillStyle = '#111';
        const label = f.name.length > 60 ? f.name.slice(0, 58) + '..' : f.name;
        ctx.fillText(label, Math.max(x0, 0) + 3, y + FRAME_H / 2 + 1,
                     Math.min(w - 6, W));
      }
    }
  }
}

function frameAt(mx, my) {
  const depth = Math.floor(my / (FRAME_H + GAP));
  const list = byDepth.get(depth);
  if (!list) return null;
  const W = canvas.clientWidth;
  const t = view.start + mx / W * (view.end - view.start);
  for (const f of list) {
    if (t >= f.start && t < f.start + f.value) return f;
  }
  return null;
}

canvas.addEventListener('mousemove', (e) => {
  const f = frameAt(e.offsetX, e.offsetY);
  if (!f) { tooltip.style.display = 'none'; return; }
  const pct = (f.value / TOTAL * 100).toFixed(2);
  tooltip.innerHTML = `<span class="name">${f.name}</span><br>` +
    `${fmtTime(f.value)} (${pct}% of frame)`;
  tooltip.style.display = 'block';
  tooltip.style.left = Math.min(e.clientX + 14, window.innerWidth - 200) + 'px';
  tooltip.style.top = (e.clientY + 14) + 'px';
});

canvas.addEventListener('mouseleave', () => tooltip.style.display = 'none');

canvas.addEventListener('click', (e) => {
  const f = frameAt(e.offsetX, e.offsetY);
  if (f) {
    view = { start: f.start, end: f.start + f.value };
  } else {
    view = { start: 0, end: TOTAL };
  }
  header.textContent = (f ? f.name + ' \\u2014 ' : '') +
    `Click a frame to zoom, click background to reset. [${fmtTime(view.start)} .. ${fmtTime(view.end)}]`;
  draw();
});

slider.addEventListener('input', () => loadFrame(parseInt(slider.value, 10)));
frameBox.addEventListener('change', () => loadFrame(parseInt(frameBox.value, 10)));

window.addEventListener('resize', resize);
slider.max = N - 1;
frameBox.max = N - 1;
loadFrame(0);
resize();
</script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(
        description="Create a flamegraph-style HTML from timings.json"
    )
    parser.add_argument("input", nargs="?", default="timings.json",
                        help="input timings json (default: timings.json)")
    parser.add_argument("-o", "--output", default=None,
                        help="output html path (default: <input>.html)")
    args = parser.parse_args()

    in_path = Path(args.input)
    out_path = Path(args.output) if args.output else in_path.with_suffix(".html")

    with open(in_path) as f:
        data = json.load(f)

    frames_data = [build_flame_data([entry]) for entry in data]
    durations = [entry["duration_ms"] for entry in data]

    html = HTML_TEMPLATE.replace("__FRAMES_DATA__", json.dumps(frames_data))
    html = html.replace("__DURATIONS__", json.dumps(durations))
    out_path.write_text(html)
    print(f"wrote {out_path} ({len(data)} frames, "
          f"total {sum(durations) / 1000:.3f} ms)")


if __name__ == "__main__":
    main()
