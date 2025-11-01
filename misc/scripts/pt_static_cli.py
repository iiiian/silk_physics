# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "numpy",
#     "plotly",
# ]
# ///
"""
pt_static_cli.py

Static 3D visualizer for zero-TOI Point–Triangle narrowphase debug logs.

Usage examples:
  cat pt_debug.log | python pt_static_cli.py
  python pt_static_cli.py pt_debug.log

Assumptions for this static viewer:
- TOI is always zero (zero-toi error logs).
- For the 'tuv' line that provides ranges, the lower bounds are used:
    t in [t0,t1], a in [a0,a1], b in [b0,b1]  => use t=t0, a=a0, b=b0

Requires: numpy, plotly (pip install numpy plotly)
"""

from __future__ import annotations

import sys
import re
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
import plotly.graph_objects as go


# ---------- Parsing ----------


SPDLOG_PREFIX = r"\[.+\]\s*"
NUM = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"


def make_regex_pattern(name: str, array_len: int) -> re.Pattern[str]:
    """Build a relaxed pattern to match three floats after a named prefix.

    Accepts either space or comma separators as produced by spdlog formatting.
    """
    num_in_array = rf"({NUM})\s*,*\s*"
    header = rf"{SPDLOG_PREFIX}{re.escape(name)}:\s*"
    for _ in range(array_len):
        header += num_in_array
    return re.compile(header)


PT_HEADER_RE = re.compile(
    rf"{SPDLOG_PREFIX}pt collision:\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)"
)

# Example line:
# [..] [narrowphase.cpp:252] tuv = 0 1,  0.0625 0.09375,  0.8125 0.84375
TUV_RANGE_RE = re.compile(
    rf"{SPDLOG_PREFIX}tuv\s*=\s*"
    rf"({NUM})\s+({NUM})\s*,\s*"  # t0 t1
    rf"({NUM})\s+({NUM})\s*,\s*"  # a0 a1
    rf"({NUM})\s+({NUM})\s*"  # b0 b1
)


def parse_log(text: str) -> Dict[str, np.ndarray]:
    pd = {
        "pt header": PT_HEADER_RE,
        "tuv_range": TUV_RANGE_RE,
        "p_t0": make_regex_pattern("position x0 t0", 3),
        "tri_v1_t0": make_regex_pattern("position x1 t0", 3),
        "tri_v2_t0": make_regex_pattern("position x2 t0", 3),
        "tri_v3_t0": make_regex_pattern("position x3 t0", 3),
    }
    out: Dict[str, np.ndarray] = {}
    for line in text.splitlines():
        for name, pattern in pd.items():
            m = pattern.match(line)
            if m:
                out[name] = np.array([float(x) for x in m.groups()])
                break
    return out


# ---------- Math utils ----------


def triangle_edges(
    v1: np.ndarray, v2: np.ndarray, v3: np.ndarray
) -> Tuple[list, list, list]:
    xs = [v1[0], v2[0], v3[0], v1[0]]
    ys = [v1[1], v2[1], v3[1], v1[1]]
    zs = [v1[2], v2[2], v3[2], v1[2]]
    return xs, ys, zs


def contact_point(
    v1: np.ndarray, v2: np.ndarray, v3: np.ndarray, a: float, b: float
) -> np.ndarray:
    c = 1.0 - a - b
    return c * v1 + a * v2 + b * v3


def cube_aspect_bounds(pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    ctr = (mins + maxs) / 2.0
    m = float((maxs - mins).max())
    if m == 0:
        m = 1.0
    return ctr - m / 2.0, ctr + m / 2.0


# ---------- Plotting ----------


def make_static_figure(
    p_t0: np.ndarray,
    tri_t0: Tuple[np.ndarray, np.ndarray, np.ndarray],
    a: float,
    b: float,
    pt_header: np.ndarray | None,
) -> go.Figure:
    traces = []

    # Triangle at t0
    x, y, z = triangle_edges(*tri_t0)
    traces.append(
        go.Scatter3d(
            x=x,
            y=y,
            z=z,
            mode="lines",
            name="Triangle t0",
            line=dict(width=4),
            hoverinfo="name+x+y+z",
        )
    )

    # Point at t0
    traces.append(
        go.Scatter3d(
            x=[p_t0[0]],
            y=[p_t0[1]],
            z=[p_t0[2]],
            mode="markers",
            name="Point t0",
            marker=dict(size=6, symbol="circle"),
            hoverinfo="name+x+y+z",
        )
    )

    # Contact using lower-bound barycentrics
    c_t0 = contact_point(*tri_t0, a, b)
    traces.append(
        go.Scatter3d(
            x=[c_t0[0]],
            y=[c_t0[1]],
            z=[c_t0[2]],
            mode="markers",
            name="Contact (a0,b0)",
            marker=dict(size=6, symbol="circle-open"),
            hoverinfo="name+x+y+z",
        )
    )

    # Separation segment
    sep = float(np.linalg.norm(p_t0 - c_t0))
    traces.append(
        go.Scatter3d(
            x=[p_t0[0], c_t0[0]],
            y=[p_t0[1], c_t0[1]],
            z=[p_t0[2], c_t0[2]],
            mode="lines",
            name=f"sep ≈ {sep:.6e}",
            line=dict(width=2, dash="dot"),
            hoverinfo="name+x+y+z",
        )
    )

    all_pts = np.vstack([*tri_t0, p_t0, c_t0])
    lo, hi = cube_aspect_bounds(all_pts)

    header_str = ""
    if pt_header is not None and pt_header.size == 4:
        h = pt_header.astype(int)
        header_str = f"  (pt collision: {h[0]} {h[1]} {h[2]} {h[3]})"

    title = f"Static PT (TOI=0, lower bounds)  a0={a:.6g}  b0={b:.6g}{header_str}"

    layout = go.Layout(
        title=title,
        scene=dict(
            xaxis=dict(title="X", range=[lo[0], hi[0]], showspikes=False),
            yaxis=dict(title="Y", range=[lo[1], hi[1]], showspikes=False),
            zaxis=dict(title="Z", range=[lo[2], hi[2]], showspikes=False),
            aspectmode="cube",
        ),
        legend=dict(orientation="h"),
        margin=dict(l=0, r=0, t=120, b=0),
        showlegend=True,
    )

    return go.Figure(data=traces, layout=layout)


# ---------- Main ----------


def _read_input() -> str:
    if not sys.stdin.isatty():
        data = sys.stdin.read()
        if data.strip():
            return data
    # fall back to file path
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if p.is_file():
            return p.read_text()
    raise SystemExit("No input provided. Pipe the log text in or pass a file path.")


def main() -> None:
    text = _read_input()
    d = parse_log(text)

    required = ["tuv_range", "p_t0", "tri_v1_t0", "tri_v2_t0", "tri_v3_t0"]
    missing = [k for k in required if k not in d]
    if missing:
        raise SystemExit(f"Missing required fields: {', '.join(missing)}")

    # Unpack (lower bounds for a,b; TOI forced to zero)
    # tuv_range layout: [t0, t1, a0, a1, b0, b1]
    t0, _t1, a0, _a1, b0, _b1 = d["tuv_range"]
    _ = t0  # not used (TOI is always zero)
    a = float(a0)
    b = float(b0)

    p_t0 = d["p_t0"]
    tri_t0 = (d["tri_v1_t0"], d["tri_v2_t0"], d["tri_v3_t0"])
    pt_header = d.get("pt header")

    fig = make_static_figure(p_t0, tri_t0, a, b, pt_header)

    out_html = "pt_static_overlay.html"
    fig.write_html(out_html, include_plotlyjs=True, full_html=True)
    print(f"Wrote: {out_html}")


if __name__ == "__main__":
    main()
