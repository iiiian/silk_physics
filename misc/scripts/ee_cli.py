"""
ee_cli.py

Interactive 3D visualizer for Edge-Edge narrowphase debug logs.

Usage:
  cat ee_debug.log | ee_cli.py

Requires: plotly  (pip install plotly)
"""
import sys
import re
from typing import Dict, List
import numpy as np

import plotly.graph_objects as go

# ---------- Parsing ----------


def make_regex_pattern(name: str, array_len: int) -> re.Pattern:
    SPDLOG_PREFIX = r"\[.+\]\s*"
    NUM = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"
    NUM_IN_ARRAY = rf"({NUM})\s*,*\s*"

    header = rf"{SPDLOG_PREFIX}{name}:\s*"
    for i in range(array_len):
        header += NUM_IN_ARRAY

    return re.compile(header)


def parse_log(text: str) -> Dict[str, np.ndarray]:
    pd = {
        "ee_header": make_regex_pattern("ee collision", 4),
        "tuv": make_regex_pattern("tuv", 3),
        "a_v0_t0": make_regex_pattern("position x0 t0", 3),
        "a_v1_t0": make_regex_pattern("position x1 t0", 3),
        "b_v0_t0": make_regex_pattern("position x2 t0", 3),
        "b_v1_t0": make_regex_pattern("position x3 t0", 3),
        "a_v0_t1": make_regex_pattern("position x0 t1", 3),
        "a_v1_t1": make_regex_pattern("position x1 t1", 3),
        "b_v0_t1": make_regex_pattern("position x2 t1", 3),
        "b_v1_t1": make_regex_pattern("position x3 t1", 3),
        "v_a_v0_t0": make_regex_pattern("velocity x0 t0", 3),
        "v_a_v1_t0": make_regex_pattern("velocity x1 t0", 3),
        "v_b_v0_t0": make_regex_pattern("velocity x2 t0", 3),
        "v_b_v1_t0": make_regex_pattern("velocity x3 t0", 3),
        "v_a_v0_t1": make_regex_pattern("velocity x0 t1", 3),
        "v_a_v1_t1": make_regex_pattern("velocity x1 t1", 3),
        "v_b_v0_t1": make_regex_pattern("velocity x2 t1", 3),
        "v_b_v1_t1": make_regex_pattern("velocity x3 t1", 3),
    }
    d: Dict[str, np.ndarray] = {}
    for line in text.splitlines():
        for name, pattern in pd.items():
            m = pattern.match(line)
            if m:
                d[name] = np.array([float(x) for x in m.groups() if x is not None])
                break
    return d

# ---------- Math utils ----------


def interp(p0: np.ndarray, p1: np.ndarray, t: float) -> np.ndarray:
    return p0 + t * (p1 - p0)


def separation(A0, A1, B0, B1, u, v) -> float:
    pa = interp(A0, A1, u)
    pb = interp(B0, B1, v)
    return float(np.linalg.norm(pb - pa))


def cube_aspect_bounds(pts: np.ndarray):
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    centers = (mins + maxs) / 2.0
    rng = (maxs - mins)
    m = float(rng.max())
    return (centers - m / 2.0, centers + m / 2.0)

# ---------- Plotting ----------


SUPPORTED_SYMBOLS = {'circle', 'circle-open', 'cross', 'diamond', 'diamond-open', 'square', 'square-open', 'x'}


def overlay_figure(A_v0_t0, A_v1_t0, B_v0_t0, B_v1_t0,
                   A_v0_toi, A_v1_toi, B_v0_toi, B_v1_toi,
                   A_v0_ref, A_v1_ref, B_v0_ref, B_v1_ref,
                   u, v, toi, sep_t0, sep_toi, sep_ref):
    traces = []
    idx_t0: List[int] = []
    idx_toi: List[int] = []
    idx_ref: List[int] = []

    def add_trace(tr, group: str):
        traces.append(tr)
        i = len(traces) - 1
        if group == 't0':
            idx_t0.append(i)
        elif group == 'toi':
            idx_toi.append(i)
        elif group == 'ref':
            idx_ref.append(i)

    def line_trace(p0, p1, name, dash=None, width=4):
        return go.Scatter3d(
            x=[p0[0], p1[0]], y=[p0[1], p1[1]], z=[p0[2], p1[2]],
            mode="lines", name=name,
            line=dict(width=width, dash=dash) if dash else dict(width=width),
            hoverinfo="name+x+y+z",
        )

    def point_trace(p, name, symbol="circle", size=6):
        if symbol not in SUPPORTED_SYMBOLS:
            symbol = 'square'
        return go.Scatter3d(
            x=[p[0]], y=[p[1]], z=[p[2]], mode="markers", name=name,
            marker=dict(size=size, symbol=symbol),
            hoverinfo="name+x+y+z",
        )

    # t0
    add_trace(line_trace(A_v0_t0, A_v1_t0, "A t0"), 't0')
    add_trace(line_trace(B_v0_t0, B_v1_t0, "B t0"), 't0')
    pa0 = interp(A_v0_t0, A_v1_t0, u)
    pb0 = interp(B_v0_t0, B_v1_t0, v)
    add_trace(point_trace(pa0, "A@uv t0", symbol="circle"), 't0')
    add_trace(point_trace(pb0, "B@uv t0", symbol="square"), 't0')
    add_trace(line_trace(pa0, pb0, f"sep t0 ≈ {sep_t0:.6e}", dash="dot", width=2), 't0')

    # TOI
    add_trace(line_trace(A_v0_toi, A_v1_toi, "A TOI", dash="dash"), 'toi')
    add_trace(line_trace(B_v0_toi, B_v1_toi, "B TOI", dash="dash"), 'toi')
    pa1 = interp(A_v0_toi, A_v1_toi, u)
    pb1 = interp(B_v0_toi, B_v1_toi, v)
    add_trace(point_trace(pa1, "A@uv TOI", symbol="circle-open"), 'toi')
    add_trace(point_trace(pb1, "B@uv TOI", symbol="square-open"), 'toi')
    add_trace(line_trace(pa1, pb1, f"sep TOI ≈ {sep_toi:.6e}", dash="dot", width=2), 'toi')
    # Motion lines (grouped under TOI)
    for p0, p1, name in [
        (A_v0_t0, A_v0_toi, "A v0 motion"),
        (A_v1_t0, A_v1_toi, "A v1 motion"),
        (B_v0_t0, B_v0_toi, "B v0 motion"),
        (B_v1_t0, B_v1_toi, "B v1 motion"),
        (pa0, pa1, "A@uv motion"),
        (pb0, pb1, "B@uv motion"),
    ]:
        add_trace(line_trace(p0, p1, name, dash="dashdot", width=2), 'toi')

    # Reflected
    add_trace(line_trace(A_v0_ref, A_v1_ref, "A reflected", dash="longdash"), 'ref')
    add_trace(line_trace(B_v0_ref, B_v1_ref, "B reflected", dash="longdash"), 'ref')
    pa2 = interp(A_v0_ref, A_v1_ref, u)
    pb2 = interp(B_v0_ref, B_v1_ref, v)
    add_trace(point_trace(pa2, "A@uv reflected", symbol="diamond"), 'ref')
    add_trace(point_trace(pb2, "B@uv reflected", symbol="x"), 'ref')
    add_trace(line_trace(pa2, pb2, f"sep reflected ≈ {sep_ref:.6e}", dash="dot", width=2), 'ref')

    # Aspect + title
    all_pts = [A_v0_t0, A_v1_t0, B_v0_t0, B_v1_t0,
               A_v0_toi, A_v1_toi, B_v0_toi, B_v1_toi,
               A_v0_ref, A_v1_ref, B_v0_ref, B_v1_ref,
               pa0, pb0, pa1, pb1, pa2, pb2]
    all_pts = np.vstack(all_pts)
    lo, hi = cube_aspect_bounds(all_pts)

    title = f"Edge–Edge Overlay: t0 vs TOI({toi:.8f})  sep_t0={sep_t0:.6e}  sep_TOI={sep_toi:.6e}  sep_reflected={sep_ref:.6e}"

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
        updatemenus=[
            dict(
                type="buttons", direction="left", x=0.0, y=1.20, xanchor="left", yanchor="bottom", showactive=True,
                buttons=[
                    dict(label="t0: Show", method="restyle",
                         args=[{"visible": [True] * len(idx_t0)}, idx_t0]),
                    dict(label="t0: Hide", method="restyle",
                         args=[{"visible": [False] * len(idx_t0)}, idx_t0]),
                ],
            ),
            dict(
                type="buttons", direction="left", x=0.45, y=1.20, xanchor="left", yanchor="bottom", showactive=True,
                buttons=[
                    dict(label="TOI: Show", method="restyle",
                         args=[{"visible": [True] * len(idx_toi)}, idx_toi]),
                    dict(label="TOI: Hide", method="restyle",
                         args=[{"visible": [False] * len(idx_toi)}, idx_toi]),
                ],
            ),
            dict(
                type="buttons", direction="left", x=0.8, y=1.20, xanchor="left", yanchor="bottom", showactive=True,
                buttons=[
                    dict(label="Ref: Show", method="restyle",
                         args=[{"visible": [True] * len(idx_ref)}, idx_ref]),
                    dict(label="Ref: Hide", method="restyle",
                         args=[{"visible": [False] * len(idx_ref)}, idx_ref]),
                ],
            )
        ],
    )
    fig = go.Figure(data=traces, layout=layout)
    return fig

# ---------- Main ----------


def main():
    text = sys.stdin.read()

    if not text.strip():
        raise SystemExit("No input provided. Pipe the log text in or pass a file path.")

    data = parse_log(text)
    required = [
        "tuv",
        "a_v0_t0", "a_v1_t0", "b_v0_t0", "b_v1_t0",
        "a_v0_t1", "a_v1_t1", "b_v0_t1", "b_v1_t1",
        "v_a_v0_t0", "v_a_v1_t0", "v_b_v0_t0", "v_b_v1_t0",
        "v_a_v0_t1", "v_a_v1_t1", "v_b_v0_t1", "v_b_v1_t1",
    ]
    missing = [k for k in required if k not in data]
    if missing:
        raise SystemExit(f"Missing required fields: {', '.join(missing)}")

    toi, u, v = data["tuv"]

    A_v0_t0, A_v1_t0 = data["a_v0_t0"], data["a_v1_t0"]
    B_v0_t0, B_v1_t0 = data["b_v0_t0"], data["b_v1_t0"]
    A_v0_t1, A_v1_t1 = data["a_v0_t1"], data["a_v1_t1"]
    B_v0_t1, B_v1_t1 = data["b_v0_t1"], data["b_v1_t1"]

    v_A_v0_t0, v_A_v1_t0 = data["v_a_v0_t0"], data["v_a_v1_t0"]
    v_B_v0_t0, v_B_v1_t0 = data["v_b_v0_t0"], data["v_b_v1_t0"]
    v_A_v0_t1, v_A_v1_t1 = data["v_a_v0_t1"], data["v_a_v1_t1"]
    v_B_v0_t1, v_B_v1_t1 = data["v_b_v0_t1"], data["v_b_v1_t1"]

    # Interpolate TOI vertices
    A_v0_toi = interp(A_v0_t0, A_v0_t1, toi)
    A_v1_toi = interp(A_v1_t0, A_v1_t1, toi)
    B_v0_toi = interp(B_v0_t0, B_v0_t1, toi)
    B_v1_toi = interp(B_v1_t0, B_v1_t1, toi)

    # Calculate reflected vertices
    A_v0_ref = A_v0_t0 + toi * v_A_v0_t0 + (1 - toi) * v_A_v0_t1
    A_v1_ref = A_v1_t0 + toi * v_A_v1_t0 + (1 - toi) * v_A_v1_t1
    B_v0_ref = B_v0_t0 + toi * v_B_v0_t0 + (1 - toi) * v_B_v0_t1
    B_v1_ref = B_v1_t0 + toi * v_B_v1_t0 + (1 - toi) * v_B_v1_t1

    sep_t0 = separation(A_v0_t0, A_v1_t0, B_v0_t0, B_v1_t0, u, v)
    sep_toi = separation(A_v0_toi, A_v1_toi, B_v0_toi, B_v1_toi, u, v)
    sep_ref = separation(A_v0_ref, A_v1_ref, B_v0_ref, B_v1_ref, u, v)

    fig = overlay_figure(A_v0_t0, A_v1_t0, B_v0_t0, B_v1_t0,
                         A_v0_toi, A_v1_toi, B_v0_toi, B_v1_toi,
                         A_v0_ref, A_v1_ref, B_v0_ref, B_v1_ref,
                         u, v, toi, sep_t0, sep_toi, sep_ref)

    out_html = "ee_overlay.html"
    fig.write_html(out_html, include_plotlyjs=True, full_html=True)
    print(f"Print result to: {out_html}")


if __name__ == "__main__":
    main()