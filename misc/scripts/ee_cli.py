"""
edge_edge_cli_interactive.py

Interactive 3D visualizer for Edge–Edge narrowphase debug logs.
Now includes UI toggles to show/hide the geometry at t0, TOI, and reflected (if present).

Parses:
  - edge a|b v0|v1 at t0/t1
  - optional: edge a|b v0|v1 reflected
  - EE: toi = [t0, t1], edge a para = [u0, u1], edge b para = [v0, v1], tol = ...

Builds a single Plotly overlay:
  - Edges A and B at t0 (solid), at TOI (dashed), and reflected (long-dashed, if present)
  - Mid-parameter points on each edge (using mean of parameter intervals)
  - Separation segments between the midpoints for each pose
  - Motion segments from t0 to TOI (vertices and midpoints)
  - Three updatemenus to toggle visibility of t0 / TOI / Reflected groups
  - Cube aspect, draggable/zoomable view

Usage:
  cat ee_debug.log | edge_edge_cli_interactive.py --prefix out/edge_edge --toi mid --open
  edge_edge_cli_interactive.py ee_debug.log --prefix edge_edge --toi max
  edge_edge_cli_interactive.py ee_debug.log --embed-js   # fully offline HTML

Requires: plotly  (pip install plotly)
"""
import sys
import re
import argparse
import io
import textwrap
import webbrowser
import os
from typing import Dict, Optional, List
import numpy as np

try:
    import plotly.graph_objects as go
except Exception:
    sys.stderr.write("Plotly is required. Install with: pip install plotly\n")
    raise

# ---------- Parsing ----------

EDGE_RE = re.compile(
    r'''edge\s+(?P<edge>[abAB])\s+v(?P<v>[01])\s+at\s+(?P<time>t0|t1)\s*:\s*
        (?P<x>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+
        (?P<y>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+
        (?P<z>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)''',
    re.VERBOSE
)

REFL_RE = re.compile(
    r'''edge\s+(?P<edge>[abAB])\s+v(?P<v>[01])\s+reflected\s*:\s*
        (?P<x>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+
        (?P<y>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s+
        (?P<z>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)''',
    re.VERBOSE
)

TOI_RE = re.compile(
    r'''EE:\s*toi\s*=\s*\[\s*
        (?P<toi0>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*
        (?P<toi1>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*\]\s*,\s*
        edge\s+a\s+para\s*=\s*\[\s*
        (?P<a0>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*
        (?P<a1>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*\]\s*,\s*
        edge\s+b\s+para\s*=\s*\[\s*
        (?P<b0>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*
        (?P<b1>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*\]\s*,\s*
        tol\s*=\s*(?P<tol>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)''',
    re.VERBOSE
)


def parse_log(text: str) -> Dict[str, np.ndarray]:
    data: Dict[str, np.ndarray] = {}
    for m in EDGE_RE.finditer(text):
        edge = m.group("edge").lower()
        v = int(m.group("v"))
        time = m.group("time")
        key = f"{edge}_v{v}_{time}"
        data[key] = np.array([float(m.group("x")), float(m.group("y")), float(m.group("z"))], dtype=float)
    for m in REFL_RE.finditer(text):
        edge = m.group("edge").lower()
        v = int(m.group("v"))
        key = f"{edge}_v{v}_ref"
        data[key] = np.array([float(m.group("x")), float(m.group("y")), float(m.group("z"))], dtype=float)
    # EE line
    toi_match = None
    for m in TOI_RE.finditer(text):
        toi_match = m  # last match wins
    if toi_match is not None:
        data["toi"] = np.array([float(toi_match.group("toi0")), float(toi_match.group("toi1"))], dtype=float)
        data["a_para"] = np.array([float(toi_match.group("a0")), float(toi_match.group("a1"))], dtype=float)
        data["b_para"] = np.array([float(toi_match.group("b0")), float(toi_match.group("b1"))], dtype=float)
        data["tol"] = float(toi_match.group("tol"))
    return data

# ---------- Math utils ----------


def pseg(p0: np.ndarray, p1: np.ndarray, t: float) -> np.ndarray:
    return p0 + t * (p1 - p0)


def separation(A0, A1, B0, B1, ta, tb) -> float:
    pa = pseg(A0, A1, ta)
    pb = pseg(B0, B1, tb)
    return float(np.linalg.norm(pb - pa))


def cube_aspect_from_points(pts: np.ndarray):
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
                   ta, tb, toi_mode, toi, sep_t0, sep_toi,
                   A_v0_ref=None, A_v1_ref=None, B_v0_ref=None, B_v1_ref=None,
                   sep_ref: Optional[float] = None):
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
    pa0 = pseg(A_v0_t0, A_v1_t0, ta)
    pb0 = pseg(B_v0_t0, B_v1_t0, tb)
    add_trace(point_trace(pa0, "A@t(mid) t0", symbol="circle"), 't0')
    add_trace(point_trace(pb0, "B@t(mid) t0", symbol="square"), 't0')
    add_trace(line_trace(pa0, pb0, f"sep t0 ≈ {sep_t0:.6e}", dash="dot", width=2), 't0')

    # TOI
    add_trace(line_trace(A_v0_toi, A_v1_toi, f"A TOI({toi_mode})", dash="dash"), 'toi')
    add_trace(line_trace(B_v0_toi, B_v1_toi, f"B TOI({toi_mode})", dash="dash"), 'toi')
    pa1 = pseg(A_v0_toi, A_v1_toi, ta)
    pb1 = pseg(B_v0_toi, B_v1_toi, tb)
    add_trace(point_trace(pa1, "A@t(mid) TOI", symbol="circle-open"), 'toi')
    add_trace(point_trace(pb1, "B@t(mid) TOI", symbol="square-open"), 'toi')
    add_trace(line_trace(pa1, pb1, f"sep TOI ≈ {sep_toi:.6e}", dash="dot", width=2), 'toi')
    # Motion lines (grouped under TOI)
    for p0, p1, name in [
        (A_v0_t0, A_v0_toi, "A v0 motion"),
        (A_v1_t0, A_v1_toi, "A v1 motion"),
        (B_v0_t0, B_v0_toi, "B v0 motion"),
        (B_v1_t0, B_v1_toi, "B v1 motion"),
        (pa0, pa1, "A@t(mid) motion"),
        (pb0, pb1, "B@t(mid) motion"),
    ]:
        add_trace(line_trace(p0, p1, name, dash="dashdot", width=2), 'toi')

    # Reflected (optional)
    have_ref = all(x is not None for x in [A_v0_ref, A_v1_ref, B_v0_ref, B_v1_ref])
    pa2 = pb2 = None
    if have_ref:
        add_trace(line_trace(A_v0_ref, A_v1_ref, "A reflected", dash="longdash"), 'ref')
        add_trace(line_trace(B_v0_ref, B_v1_ref, "B reflected", dash="longdash"), 'ref')
        pa2 = pseg(A_v0_ref, A_v1_ref, ta)
        pb2 = pseg(B_v0_ref, B_v1_ref, tb)
        if sep_ref is None:
            sep_ref = float(np.linalg.norm(pb2 - pa2))
        add_trace(point_trace(pa2, "A@t(mid) reflected", symbol="diamond"), 'ref')
        add_trace(point_trace(pb2, "B@t(mid) reflected", symbol="x"), 'ref')
        add_trace(line_trace(pa2, pb2, f"sep reflected ≈ {sep_ref:.6e}", dash="dot", width=2), 'ref')

    # Aspect + title
    all_pts = [A_v0_t0, A_v1_t0, B_v0_t0, B_v1_t0,
               A_v0_toi, A_v1_toi, B_v0_toi, B_v1_toi,
               pa0, pb0, pa1, pb1]
    if have_ref:
        all_pts += [A_v0_ref, A_v1_ref, B_v0_ref, B_v1_ref, pa2, pb2]
    all_pts = np.vstack(all_pts)
    lo, hi = cube_aspect_from_points(all_pts)

    title = f"Edge–Edge Overlay: t0 vs TOI({toi_mode}={toi:.8f})  sep_t0={sep_t0:.6e}  sep_TOI={sep_toi:.6e}"
    if have_ref:
        title += f"  sep_reflected={sep_ref:.6e}"

    layout = go.Layout(
        title=title,
        scene=dict(
            xaxis=dict(title="X", range=[lo[0], hi[0]], showspikes=False),
            yaxis=dict(title="Y", range=[lo[1], hi[1]], showspikes=False),
            zaxis=dict(title="Z", range=[lo[2], hi[2]], showspikes=False),
            aspectmode="cube",
        ),
        legend=dict(orientation="h"),
        margin=dict(l=0, r=0, t=90, b=0),
        showlegend=True,
        updatemenus=[
            dict(
                type="buttons", direction="left", x=0.0, y=1.12, showactive=True,
                buttons=[
                    dict(label="t0: Show", method="restyle",
                         args=[{"visible": [True] * len(idx_t0)}, idx_t0]),
                    dict(label="t0: Hide", method="restyle",
                         args=[{"visible": [False] * len(idx_t0)}, idx_t0]),
                ],
            ),
            dict(
                type="buttons", direction="left", x=0.45, y=1.12, showactive=True,
                buttons=[
                    dict(label="TOI: Show", method="restyle",
                         args=[{"visible": [True] * len(idx_toi)}, idx_toi]),
                    dict(label="TOI: Hide", method="restyle",
                         args=[{"visible": [False] * len(idx_toi)}, idx_toi]),
                ],
            ),
        ] + ([
            dict(
                type="buttons", direction="left", x=0.8, y=1.12, showactive=True,
                buttons=[
                    dict(label="Ref: Show", method="restyle",
                         args=[{"visible": [True] * len(idx_ref)}, idx_ref]),
                    dict(label="Ref: Hide", method="restyle",
                         args=[{"visible": [False] * len(idx_ref)}, idx_ref]),
                ],
            )
        ] if len(idx_ref) > 0 else []),
    )
    fig = go.Figure(data=traces, layout=layout)
    return fig

# ---------- Main ----------


def main():
    ap = argparse.ArgumentParser(
        description="Interactive overlay (t0 vs TOI, optional reflected) for Edge–Edge logs.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent('''
          Examples:
            cat ee_debug.log | edge_edge_cli_interactive.py --prefix out/edge_edge --toi mid --open
            edge_edge_cli_interactive.py ee_debug.log --prefix edge_edge --toi max
        '''),
    )
    ap.add_argument("path", nargs="?", help="Optional path to a log file. If omitted, read stdin.")
    ap.add_argument("--prefix", default="edge_edge", help="Output file prefix (default: edge_edge)")
    ap.add_argument("--toi", choices=["mid", "min", "max"], default="mid", help="Which TOI to use from the interval (default: mid)")
    ap.add_argument("--open", action="store_true", help="Open the resulting HTML in your default browser")
    ap.add_argument("--embed-js", action="store_true", help="Embed Plotly.js (works fully offline) instead of using the CDN")
    args = ap.parse_args()

    # Read input
    if args.path:
        with open(args.path, "r", encoding="utf-8") as f:
            text = f.read()
    else:
        if sys.stdin.isatty():
            print("Reading from stdin; press Ctrl-D (Unix) or Ctrl-Z (Windows) when done.", file=sys.stderr)
        text = sys.stdin.read()

    if not text.strip():
        raise SystemExit("No input provided. Pipe the log text in or pass a file path.")

    data = parse_log(text)
    required = ["a_v0_t0", "a_v1_t0", "b_v0_t0", "b_v1_t0", "a_v0_t1", "a_v1_t1", "b_v0_t1", "b_v1_t1", "toi", "a_para", "b_para"]
    missing = [k for k in required if k not in data]
    if missing:
        raise SystemExit(f"Missing required fields: {', '.join(missing)}")

    A_v0_t0, A_v1_t0 = data["a_v0_t0"], data["a_v1_t0"]
    B_v0_t0, B_v1_t0 = data["b_v0_t0"], data["b_v1_t0"]
    A_v0_t1, A_v1_t1 = data["a_v0_t1"], data["a_v1_t1"]
    B_v0_t1, B_v1_t1 = data["b_v0_t1"], data["b_v1_t1"]
    toi0, toi1 = map(float, data["toi"])
    if args.toi == "mid":
        toi = (toi0 + toi1) / 2.0
    elif args.toi == "min":
        toi = toi0
    else:
        toi = toi1

    ta = float(data["a_para"].mean())
    tb = float(data["b_para"].mean())

    # Interpolate TOI vertices
    A_v0_toi = pseg(A_v0_t0, A_v0_t1, toi)
    A_v1_toi = pseg(A_v1_t0, A_v1_t1, toi)
    B_v0_toi = pseg(B_v0_t0, B_v0_t1, toi)
    B_v1_toi = pseg(B_v1_t0, B_v1_t1, toi)

    sep_t0 = separation(A_v0_t0, A_v1_t0, B_v0_t0, B_v1_t0, ta, tb)
    sep_toi = separation(A_v0_toi, A_v1_toi, B_v0_toi, B_v1_toi, ta, tb)

    # Optional reflected pose
    A_v0_ref = data.get("a_v0_ref")
    A_v1_ref = data.get("a_v1_ref")
    B_v0_ref = data.get("b_v0_ref")
    B_v1_ref = data.get("b_v1_ref")
    sep_ref = None
    if all(x is not None for x in [A_v0_ref, A_v1_ref, B_v0_ref, B_v1_ref]):
        sep_ref = separation(A_v0_ref, A_v1_ref, B_v0_ref, B_v1_ref, ta, tb)

    fig = overlay_figure(A_v0_t0, A_v1_t0, B_v0_t0, B_v1_t0,
                         A_v0_toi, A_v1_toi, B_v0_toi, B_v1_toi,
                         ta, tb, args.toi, toi, sep_t0, sep_toi,
                         A_v0_ref, A_v1_ref, B_v0_ref, B_v1_ref, sep_ref)

    out_html = f"{args.prefix}_overlay_{args.toi}.html"
    fig.write_html(out_html, include_plotlyjs=('cdn' if not args.embed_js else True), full_html=True)
    print(out_html)
    if args.open:
        try:
            webbrowser.open('file://' + os.path.realpath(out_html))
        except Exception:
            pass


if __name__ == "__main__":
    main()
