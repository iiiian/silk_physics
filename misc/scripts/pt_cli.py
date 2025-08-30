"""
pt_cli.py

Interactive 3D visualizer for Point–Triangle narrowphase debug logs.

Usage:
  cat pt_debug.log | pt_cli_interactive.py

Requires: plotly  (pip install plotly)
"""

import sys, re
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
        "pt header": make_regex_pattern("pt collision", 4),
        "tuv": make_regex_pattern("tuv", 3),
        "p_t0": make_regex_pattern("position x0 t0", 3),
        "tri_v1_t0": make_regex_pattern("position x1 t0", 3),
        "tri_v2_t0": make_regex_pattern("position x2 t0", 3),
        "tri_v3_t0": make_regex_pattern("position x3 t0", 3),
        "p_t1": make_regex_pattern("position x0 t1", 3),
        "tri_v1_t1": make_regex_pattern("position x1 t1", 3),
        "tri_v2_t1": make_regex_pattern("position x2 t1", 3),
        "tri_v3_t1": make_regex_pattern("position x3 t1", 3),
        "v_p_t0": make_regex_pattern("velocity x0 t0", 3),
        "v_tri_v1_t0": make_regex_pattern("velocity x1 t0", 3),
        "v_tri_v2_t0": make_regex_pattern("velocity x2 t0", 3),
        "v_tri_v3_t0": make_regex_pattern("velocity x3 t0", 3),
        "v_p_t1": make_regex_pattern("velocity x0 t1", 3),
        "v_tri_v1_t1": make_regex_pattern("velocity x1 t1", 3),
        "v_tri_v2_t1": make_regex_pattern("velocity x2 t1", 3),
        "v_tri_v3_t1": make_regex_pattern("velocity x3 t1", 3),
    }
    d: Dict[str, np.ndarray] = {}
    for line in text.splitlines():
        for name, pattern in pd.items():
            m = pattern.match(line)
            if m:
                d[name] = np.array([float(x) for x in m.groups()])
                break
    return d


# ---------- Math utils ----------


def interp(p0: np.ndarray, p1: np.ndarray, t: float) -> np.ndarray:
    return p0 + t * (p1 - p0)


def triangle_edges(v1, v2, v3):
    xs = [v1[0], v2[0], v3[0], v1[0]]
    ys = [v1[1], v2[1], v3[1], v1[1]]
    zs = [v1[2], v2[2], v3[2], v1[2]]
    return xs, ys, zs


def contact_point(v1, v2, v3, a: float, b: float) -> np.ndarray:
    c = 1.0 - a - b
    return c * v1 + a * v2 + b * v3


def cube_aspect_bounds(pts: np.ndarray):
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    ctr = (mins + maxs) / 2.0
    m = float((maxs - mins).max())
    return ctr - m / 2.0, ctr + m / 2.0


# ---------- Plotting ----------

SUPPORTED_SYMBOLS = {
    "circle",
    "circle-open",
    "cross",
    "diamond",
    "diamond-open",
    "square",
    "square-open",
    "x",
}


def make_overlay(
    p_t0,
    tri_t0,
    p_toi,
    tri_toi,
    p_ref,
    tri_ref,
    a,
    b,
    toi,
    sep_t0,
    sep_toi,
    sep_ref,
):
    traces = []
    idx_t0: List[int] = []
    idx_toi: List[int] = []
    idx_ref: List[int] = []

    def add_trace(tr, group: str):
        traces.append(tr)
        i = len(traces) - 1
        if group == "t0":
            idx_t0.append(i)
        elif group == "toi":
            idx_toi.append(i)
        elif group == "ref":
            idx_ref.append(i)

    def tri_trace(tri, name, dash=None, width=4):
        x, y, z = triangle_edges(*tri)
        return go.Scatter3d(
            x=x,
            y=y,
            z=z,
            mode="lines",
            name=name,
            line=dict(width=width, dash=dash) if dash else dict(width=width),
            hoverinfo="name+x+y+z",
        )

    def point_trace(p, name, symbol="circle", size=6):
        if symbol not in SUPPORTED_SYMBOLS:
            symbol = "square"
        return go.Scatter3d(
            x=[p[0]],
            y=[p[1]],
            z=[p[2]],
            mode="markers",
            name=name,
            marker=dict(size=size, symbol=symbol),
            hoverinfo="name+x+y+z",
        )

    def seg_trace(p0, p1, name, dash="dot", width=2):
        return go.Scatter3d(
            x=[p0[0], p1[0]],
            y=[p0[1], p1[1]],
            z=[p0[2], p1[2]],
            mode="lines",
            name=name,
            line=dict(width=width, dash=dash),
            hoverinfo="name+x+y+z",
        )

    # t0
    add_trace(tri_trace(tri_t0, "Triangle t0"), "t0")
    c_t0 = contact_point(*tri_t0, a, b)
    add_trace(point_trace(p_t0, "Point t0", symbol="circle"), "t0")
    add_trace(point_trace(c_t0, "Contact t0 (bary mid)", symbol="circle-open"), "t0")
    if sep_t0 is None:
        sep_t0 = float(np.linalg.norm(p_t0 - c_t0))
    add_trace(seg_trace(p_t0, c_t0, f"sep t0 ≈ {sep_t0:.6e}"), "t0")

    # TOI
    add_trace(tri_trace(tri_toi, "TOI", dash="dash"), "toi")
    c_toi = contact_point(*tri_toi, a, b)
    add_trace(point_trace(p_toi, "Point TOI", symbol="square"), "toi")
    add_trace(point_trace(c_toi, "Contact TOI", symbol="square-open"), "toi")
    if sep_toi is None:
        sep_toi = float(np.linalg.norm(p_toi - c_toi))
    add_trace(seg_trace(p_toi, c_toi, f"sep TOI ≈ {sep_toi:.6e}"), "toi")

    # Motion P (attach to TOI group for visibility control)
    add_trace(seg_trace(p_t0, p_toi, "Point motion", dash="dashdot", width=2), "toi")

    # Reflected (optional)
    all_pts = [*tri_t0, p_t0, c_t0, *tri_toi, p_toi, c_toi]
    title = f"Point–Triangle Overlay: t0 vs TOI({toi:.8f})  sep_t0={sep_t0:.6e}  sep_TOI={sep_toi:.6e}"
    if tri_ref is not None and p_ref is not None:
        add_trace(tri_trace(tri_ref, "Triangle reflected", dash="longdash"), "ref")
        c_ref = contact_point(*tri_ref, a, b)
        add_trace(point_trace(p_ref, "Point reflected", symbol="diamond"), "ref")
        add_trace(point_trace(c_ref, "Contact reflected", symbol="x"), "ref")
        if sep_ref is None:
            sep_ref = float(np.linalg.norm(p_ref - c_ref))
        add_trace(seg_trace(p_ref, c_ref, f"sep reflected ≈ {sep_ref:.6e}"), "ref")
        all_pts += [*tri_ref, p_ref, c_ref]
        title += f"  sep_reflected={sep_ref:.6e}"

    all_pts = np.vstack(all_pts)
    lo, hi = cube_aspect_bounds(all_pts)

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
                type="buttons",
                direction="left",
                x=0.0,
                y=1.20,
                xanchor="left",
                yanchor="bottom",
                showactive=True,
                buttons=[
                    dict(
                        label="t0: Show",
                        method="restyle",
                        args=[{"visible": [True] * len(idx_t0)}, idx_t0],
                    ),
                    dict(
                        label="t0: Hide",
                        method="restyle",
                        args=[{"visible": [False] * len(idx_t0)}, idx_t0],
                    ),
                ],
            ),
            dict(
                type="buttons",
                direction="left",
                x=0.45,
                y=1.20,
                xanchor="left",
                yanchor="bottom",
                showactive=True,
                buttons=[
                    dict(
                        label="TOI: Show",
                        method="restyle",
                        args=[{"visible": [True] * len(idx_toi)}, idx_toi],
                    ),
                    dict(
                        label="TOI: Hide",
                        method="restyle",
                        args=[{"visible": [False] * len(idx_toi)}, idx_toi],
                    ),
                ],
            ),
        ]
        + (
            [
                dict(
                    type="buttons",
                    direction="left",
                    x=0.8,
                    y=1.20,
                    xanchor="left",
                    yanchor="bottom",
                    showactive=True,
                    buttons=[
                        dict(
                            label="Ref: Show",
                            method="restyle",
                            args=[{"visible": [True] * len(idx_ref)}, idx_ref],
                        ),
                        dict(
                            label="Ref: Hide",
                            method="restyle",
                            args=[{"visible": [False] * len(idx_ref)}, idx_ref],
                        ),
                    ],
                )
            ]
            if len(idx_ref) > 0
            else []
        ),
    )
    fig = go.Figure(data=traces, layout=layout)
    return fig


# ---------- Main ----------


def main():
    text = sys.stdin.read()
    if not text.strip():
        raise SystemExit("No input provided. Pipe the log text in or pass a file path.")

    d = parse_log(text)
    required = [
        "pt header",
        "tuv",
        "p_t0",
        "tri_v1_t0",
        "tri_v2_t0",
        "tri_v3_t0",
        "p_t1",
        "tri_v1_t1",
        "tri_v2_t1",
        "tri_v3_t1",
        "v_p_t0",
        "v_tri_v1_t0",
        "v_tri_v2_t0",
        "v_tri_v3_t0",
        "v_p_t1",
        "v_tri_v1_t1",
        "v_tri_v2_t1",
        "v_tri_v3_t1",
    ]
    missing = [k for k in required if k not in d]
    if missing:
        raise SystemExit(f"Missing required fields: {', '.join(missing)}")

    # unpack
    toi = d["tuv"][0]
    a = d["tuv"][1]
    b = d["tuv"][2]
    p_t0 = d["p_t0"]
    p_t1 = d["p_t1"]
    tri_t0 = (d["tri_v1_t0"], d["tri_v2_t0"], d["tri_v3_t0"])
    tri_t1 = (d["tri_v1_t1"], d["tri_v2_t1"], d["tri_v3_t1"])
    v_p_t0 = d["v_p_t0"]
    v_p_t1 = d["v_p_t1"]
    v_tri_t0 = (d["v_tri_v1_t0"], d["v_tri_v2_t0"], d["v_tri_v3_t0"])
    v_tri_t1 = (d["v_tri_v1_t1"], d["v_tri_v2_t1"], d["v_tri_v3_t1"])

    # Interpolate point and triangle to TOI
    p_toi = interp(p_t0, p_t1, toi)
    tri_toi = tuple(interp(v0, v1, toi) for v0, v1 in zip(tri_t0, tri_t1))

    # reflected
    p_ref = p_t0 + toi * v_p_t0 + (1 - toi) * v_p_t1
    tri_ref = []
    tri_ref.append(tri_t0[0] + toi * v_tri_t0[0] + (1 - toi) * v_tri_t1[0])
    tri_ref.append(tri_t0[1] + toi * v_tri_t0[1] + (1 - toi) * v_tri_t1[1])
    tri_ref.append(tri_t0[2] + toi * v_tri_t0[2] + (1 - toi) * v_tri_t1[2])

    # Separations (computed with bary mid consistently per pose)
    c_t0 = contact_point(*tri_t0, a, b)
    c_toi = contact_point(*tri_toi, a, b)
    sep_t0 = float(np.linalg.norm(p_t0 - c_t0))
    sep_toi = float(np.linalg.norm(p_toi - c_toi))
    c_ref = contact_point(*tri_ref, a, b)
    sep_ref = float(np.linalg.norm(p_ref - c_ref))

    fig = make_overlay(
        p_t0,
        tri_t0,
        p_toi,
        tri_toi,
        p_ref,
        tri_ref,
        a,
        b,
        toi,
        sep_t0,
        sep_toi,
        sep_ref,
    )

    out_html = "pt_overlay.html"
    fig.write_html(
        out_html,
        include_plotlyjs=True,
        full_html=True,
    )
    print(f"Print result to: {out_html}")


if __name__ == "__main__":
    main()
