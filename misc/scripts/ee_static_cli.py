# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "numpy",
#     "plotly",
# ]
# ///
"""
ee_static_cli.py

Static 3D visualizer for zero-TOI Edge–Edge narrowphase debug logs.

Usage examples:
  cat ee_debug.log | python ee_static_cli.py
  python ee_static_cli.py ee_debug.log

Assumptions for this static viewer:
- TOI is always zero (zero-toi error logs).
- For the 'tuv' line that provides ranges, the lower bounds are used:
    t in [t0,t1], u in [u0,u1], v in [v0,v1]  => use t=t0, u=u0, v=v0

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
    num_in_array = rf"({NUM})\s*,*\s*"
    header = rf"{SPDLOG_PREFIX}{re.escape(name)}:\s*"
    for _ in range(array_len):
        header += num_in_array
    return re.compile(header)


EE_HEADER_RE = re.compile(
    rf"{SPDLOG_PREFIX}ee collision:\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)"
)

# Example line:
# [..] [narrowphase.cpp:???] tuv = 0 1,  0.25 0.3,  0.7 0.8
TUV_RANGE_RE = re.compile(
    rf"{SPDLOG_PREFIX}tuv\s*=\s*"
    rf"({NUM})\s+({NUM})\s*,\s*"  # t0 t1
    rf"({NUM})\s+({NUM})\s*,\s*"  # u0 u1
    rf"({NUM})\s+({NUM})\s*"  # v0 v1
)


def parse_log(text: str) -> Dict[str, np.ndarray]:
    pd = {
        "ee header": EE_HEADER_RE,
        "tuv_range": TUV_RANGE_RE,
        "a_v0_t0": make_regex_pattern("position x0 t0", 3),
        "a_v1_t0": make_regex_pattern("position x1 t0", 3),
        "b_v0_t0": make_regex_pattern("position x2 t0", 3),
        "b_v1_t0": make_regex_pattern("position x3 t0", 3),
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


def interp(p0: np.ndarray, p1: np.ndarray, t: float) -> np.ndarray:
    return p0 + t * (p1 - p0)


def separation(
    A0: np.ndarray, A1: np.ndarray, B0: np.ndarray, B1: np.ndarray, u: float, v: float
) -> float:
    pa = interp(A0, A1, u)
    pb = interp(B0, B1, v)
    return float(np.linalg.norm(pb - pa))


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
    A_v0_t0: np.ndarray,
    A_v1_t0: np.ndarray,
    B_v0_t0: np.ndarray,
    B_v1_t0: np.ndarray,
    u: float,
    v: float,
    ee_header: np.ndarray | None,
) -> go.Figure:
    traces = []

    def line_trace(p0, p1, name, width=4):
        return go.Scatter3d(
            x=[p0[0], p1[0]],
            y=[p0[1], p1[1]],
            z=[p0[2], p1[2]],
            mode="lines",
            name=name,
            line=dict(width=width),
            hoverinfo="name+x+y+z",
        )

    def point_trace(p, name, symbol="circle", size=6):
        return go.Scatter3d(
            x=[p[0]],
            y=[p[1]],
            z=[p[2]],
            mode="markers",
            name=name,
            marker=dict(size=size, symbol=symbol),
            hoverinfo="name+x+y+z",
        )

    # Edges at t0
    traces.append(line_trace(A_v0_t0, A_v1_t0, "Edge A t0"))
    traces.append(line_trace(B_v0_t0, B_v1_t0, "Edge B t0"))

    pa0 = interp(A_v0_t0, A_v1_t0, u)
    pb0 = interp(B_v0_t0, B_v1_t0, v)
    sep = float(np.linalg.norm(pb0 - pa0))

    traces.append(point_trace(pa0, "A@u t0", symbol="circle"))
    traces.append(point_trace(pb0, "B@v t0", symbol="square"))

    traces.append(
        go.Scatter3d(
            x=[pa0[0], pb0[0]],
            y=[pa0[1], pb0[1]],
            z=[pa0[2], pb0[2]],
            mode="lines",
            name=f"sep ≈ {sep:.6e}",
            line=dict(width=2, dash="dot"),
            hoverinfo="name+x+y+z",
        )
    )

    all_pts = np.vstack([A_v0_t0, A_v1_t0, B_v0_t0, B_v1_t0, pa0, pb0])
    lo, hi = cube_aspect_bounds(all_pts)

    header_str = ""
    if ee_header is not None and ee_header.size == 4:
        h = ee_header.astype(int)
        header_str = f"  (ee collision: {h[0]} {h[1]} {h[2]} {h[3]})"

    title = f"Static EE (TOI=0, lower bounds)  u0={u:.6g}  v0={v:.6g}{header_str}"

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
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if p.is_file():
            return p.read_text()
    raise SystemExit("No input provided. Pipe the log text in or pass a file path.")


def main() -> None:
    text = _read_input()
    d = parse_log(text)

    required = ["tuv_range", "a_v0_t0", "a_v1_t0", "b_v0_t0", "b_v1_t0"]
    missing = [k for k in required if k not in d]
    if missing:
        raise SystemExit(f"Missing required fields: {', '.join(missing)}")

    # Use lower bounds for u and v; TOI forced to zero
    # tuv_range layout: [t0, t1, u0, u1, v0, v1]
    _t0, _t1, u0, _u1, v0, _v1 = d["tuv_range"]
    u = float(u0)
    v = float(v0)

    A_v0_t0, A_v1_t0 = d["a_v0_t0"], d["a_v1_t0"]
    B_v0_t0, B_v1_t0 = d["b_v0_t0"], d["b_v1_t0"]
    ee_header = d.get("ee header")

    fig = make_static_figure(A_v0_t0, A_v1_t0, B_v0_t0, B_v1_t0, u, v, ee_header)

    out_html = "ee_static_overlay.html"
    fig.write_html(out_html, include_plotlyjs=True, full_html=True)
    print(f"Wrote: {out_html}")


if __name__ == "__main__":
    main()
