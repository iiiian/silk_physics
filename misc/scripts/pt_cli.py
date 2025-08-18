"""
pt_cli_interactive.py

Interactive 3D visualizer for Point–Triangle narrowphase debug logs.
Adds UI toggles to show/hide the geometry at t0, TOI, and reflected (if present).

Parses:
  - point at t0/t1
  - triangle v1/v2/v3 at t0/t1
  - optional: point reflected / triangle v1/v2/v3 reflected
  - PT: toi = [t0, t1], bary a = [a0, a1], bary b = [b0, b1], tol = ...

Builds a single Plotly overlay:
  - Triangle at t0 (solid), at TOI (dashed), and reflected (long-dashed, if present)
  - Point at t0, at TOI, and reflected (if present)
  - Contact points C = (1-a-b)*v1 + a*v2 + b*v3 using midpoint barycentrics
  - Separation segments P–C for each pose
  - Three updatemenu toggle groups to show/hide t0 / TOI / Reflected
  - Cube aspect, draggable/zoomable view

Usage:
  cat pt_debug.log | pt_cli_interactive.py --prefix out/pt --toi mid --open
  pt_cli_interactive.py pt_debug.log --prefix pt --toi max

Requires: plotly  (pip install plotly)
"""

import sys, re, argparse, io, textwrap, webbrowser, os
from typing import Dict, Optional, List
import numpy as np

try:
    import plotly.graph_objects as go
except Exception:
    sys.stderr.write("Plotly is required. Install with: pip install plotly\n")
    raise

# ---------- Parsing ----------

NUM = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"

PT_RE = re.compile(
    rf"""PT:\s*toi\s*=\s*\[\s*(?P<toi0>{NUM})\s*,\s*(?P<toi1>{NUM})\s*\]\s*,\s*
        bary\s*a\s*=\s*\[\s*(?P<a0>{NUM})\s*,\s*(?P<a1>{NUM})\s*\]\s*,\s*
        bary\s*b\s*=\s*\[\s*(?P<b0>{NUM})\s*,\s*(?P<b1>{NUM})\s*\]\s*,\s*
        tol\s*=\s*(?P<tol>{NUM})""",
    re.VERBOSE,
)

P_AT_T_RE = re.compile(
    rf"point\s+at\s+(?P<time>t0|t1)\s*:\s*(?P<x>{NUM})\s+(?P<y>{NUM})\s+(?P<z>{NUM})",
    re.IGNORECASE,
)
P_REF_RE = re.compile(
    rf"point\s+reflected\s*:\s*(?P<x>{NUM})\s+(?P<y>{NUM})\s+(?P<z>{NUM})",
    re.IGNORECASE,
)

TRI_AT_T_RE = re.compile(
    rf"triangle\s+v(?P<vi>[123])\s+at\s+(?P<time>t0|t1)\s*:\s*(?P<x>{NUM})\s+(?P<y>{NUM})\s+(?P<z>{NUM})",
    re.IGNORECASE,
)
TRI_REF_RE = re.compile(
    rf"triangle\s+v(?P<vi>[123])\s+reflected\s*:\s*(?P<x>{NUM})\s+(?P<y>{NUM})\s+(?P<z>{NUM})",
    re.IGNORECASE,
)


def parse_log(text: str) -> Dict[str, np.ndarray]:
    d: Dict[str, np.ndarray] = {}
    for m in P_AT_T_RE.finditer(text):
        t = m.group("time")
        d[f"p_{t}"] = np.array(
            [float(m.group("x")), float(m.group("y")), float(m.group("z"))], dtype=float
        )
    m = P_REF_RE.search(text)
    if m:
        d["p_ref"] = np.array(
            [float(m.group("x")), float(m.group("y")), float(m.group("z"))], dtype=float
        )

    for m in TRI_AT_T_RE.finditer(text):
        vi = int(m.group("vi"))
        t = m.group("time")
        d[f"tri_v{vi}_{t}"] = np.array(
            [float(m.group("x")), float(m.group("y")), float(m.group("z"))], dtype=float
        )
    for m in TRI_REF_RE.finditer(text):
        vi = int(m.group("vi"))
        d[f"tri_v{vi}_ref"] = np.array(
            [float(m.group("x")), float(m.group("y")), float(m.group("z"))], dtype=float
        )

    last = None
    for m in PT_RE.finditer(text):
        last = m
    if last:
        d["toi"] = np.array(
            [float(last.group("toi0")), float(last.group("toi1"))], dtype=float
        )
        d["bary_a"] = np.array(
            [float(last.group("a0")), float(last.group("a1"))], dtype=float
        )
        d["bary_b"] = np.array(
            [float(last.group("b0")), float(last.group("b1"))], dtype=float
        )
        d["tol"] = float(last.group("tol"))
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
    p_ref=None,
    tri_ref=None,
    a_mid=0.0,
    b_mid=0.0,
    toi_mode="mid",
    toi_val=0.0,
    sep_t0=None,
    sep_toi=None,
    sep_ref=None,
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
    c_t0 = contact_point(*tri_t0, a_mid, b_mid)
    add_trace(point_trace(p_t0, "Point t0", symbol="circle"), "t0")
    add_trace(point_trace(c_t0, "Contact t0 (bary mid)", symbol="circle-open"), "t0")
    if sep_t0 is None:
        sep_t0 = float(np.linalg.norm(p_t0 - c_t0))
    add_trace(seg_trace(p_t0, c_t0, f"sep t0 ≈ {sep_t0:.6e}"), "t0")

    # TOI
    add_trace(tri_trace(tri_toi, f"Triangle TOI({toi_mode})", dash="dash"), "toi")
    c_toi = contact_point(*tri_toi, a_mid, b_mid)
    add_trace(point_trace(p_toi, "Point TOI", symbol="square"), "toi")
    add_trace(point_trace(c_toi, "Contact TOI", symbol="square-open"), "toi")
    if sep_toi is None:
        sep_toi = float(np.linalg.norm(p_toi - c_toi))
    add_trace(seg_trace(p_toi, c_toi, f"sep TOI ≈ {sep_toi:.6e}"), "toi")

    # Motion P (attach to TOI group for visibility control)
    add_trace(seg_trace(p_t0, p_toi, "Point motion", dash="dashdot", width=2), "toi")

    # Reflected (optional)
    all_pts = [*tri_t0, p_t0, c_t0, *tri_toi, p_toi, c_toi]
    title = f"Point–Triangle Overlay: t0 vs TOI({toi_mode}={toi_val:.8f})  sep_t0={sep_t0:.6e}  sep_TOI={sep_toi:.6e}"
    if tri_ref is not None and p_ref is not None:
        add_trace(tri_trace(tri_ref, "Triangle reflected", dash="longdash"), "ref")
        c_ref = contact_point(*tri_ref, a_mid, b_mid)
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
        margin=dict(l=0, r=0, t=90, b=0),
        showlegend=True,
        updatemenus=[
            dict(
                type="buttons",
                direction="left",
                x=0.0,
                y=1.12,
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
                y=1.12,
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
                    y=1.12,
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
    ap = argparse.ArgumentParser(
        description="Interactive overlay (t0 vs TOI, optional reflected) for Point–Triangle logs.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent(
            """
          Examples:
            cat pt_debug.log | pt_cli_interactive.py --prefix out/pt --toi mid --open
            pt_cli_interactive.py pt_debug.log --prefix pt --toi max
        """
        ),
    )
    ap.add_argument(
        "path", nargs="?", help="Optional path to a log file. If omitted, read stdin."
    )
    ap.add_argument("--prefix", default="pt", help="Output file prefix (default: pt)")
    ap.add_argument(
        "--toi",
        choices=["mid", "min", "max"],
        default="mid",
        help="Which TOI to use from the interval (default: mid)",
    )
    ap.add_argument(
        "--open",
        action="store_true",
        help="Open the resulting HTML in your default browser",
    )
    ap.add_argument(
        "--embed-js",
        action="store_true",
        help="Embed Plotly.js (works fully offline) instead of using the CDN",
    )
    args = ap.parse_args()

    # Read text
    if args.path:
        with open(args.path, "r", encoding="utf-8") as f:
            text = f.read()
    else:
        if sys.stdin.isatty():
            print(
                "Reading from stdin; press Ctrl-D (Unix) or Ctrl-Z (Windows) when done.",
                file=sys.stderr,
            )
        text = sys.stdin.read()

    if not text.strip():
        raise SystemExit("No input provided. Pipe the log text in or pass a file path.")

    d = parse_log(text)
    required = [
        "p_t0",
        "p_t1",
        "tri_v1_t0",
        "tri_v2_t0",
        "tri_v3_t0",
        "tri_v1_t1",
        "tri_v2_t1",
        "tri_v3_t1",
        "toi",
        "bary_a",
        "bary_b",
    ]
    missing = [k for k in required if k not in d]
    if missing:
        raise SystemExit(f"Missing required fields: {', '.join(missing)}")

    # Gather
    p_t0, p_t1 = d["p_t0"], d["p_t1"]
    tri_t0 = (d["tri_v1_t0"], d["tri_v2_t0"], d["tri_v3_t0"])
    tri_t1 = (d["tri_v1_t1"], d["tri_v2_t1"], d["tri_v3_t1"])

    toi0, toi1 = map(float, d["toi"])
    if args.toi == "mid":
        toi = (toi0 + toi1) / 2.0
    elif args.toi == "min":
        toi = toi0
    else:
        toi = toi1

    a_mid = float(d["bary_a"].mean())
    b_mid = float(d["bary_b"].mean())

    # Interpolate point and triangle to TOI
    p_toi = interp(p_t0, p_t1, toi)
    tri_toi = tuple(interp(v0, v1, toi) for v0, v1 in zip(tri_t0, tri_t1))

    # Optional reflected
    p_ref = d.get("p_ref")
    have_ref_tri = all(k in d for k in ("tri_v1_ref", "tri_v2_ref", "tri_v3_ref"))
    tri_ref = (
        (d.get("tri_v1_ref"), d.get("tri_v2_ref"), d.get("tri_v3_ref"))
        if have_ref_tri
        else None
    )

    # Separations (computed with bary mid consistently per pose)
    c_t0 = contact_point(*tri_t0, a_mid, b_mid)
    c_toi = contact_point(*tri_toi, a_mid, b_mid)
    sep_t0 = float(np.linalg.norm(p_t0 - c_t0))
    sep_toi = float(np.linalg.norm(p_toi - c_toi))
    sep_ref = None
    if p_ref is not None and tri_ref is not None:
        c_ref = contact_point(*tri_ref, a_mid, b_mid)
        sep_ref = float(np.linalg.norm(p_ref - c_ref))

    fig = make_overlay(
        p_t0,
        tri_t0,
        p_toi,
        tri_toi,
        p_ref,
        tri_ref,
        a_mid=a_mid,
        b_mid=b_mid,
        toi_mode=args.toi,
        toi_val=toi,
        sep_t0=sep_t0,
        sep_toi=sep_toi,
        sep_ref=sep_ref,
    )

    out_html = f"{args.prefix}_pt_overlay_{args.toi}.html"
    fig.write_html(
        out_html,
        include_plotlyjs=("cdn" if not args.embed_js else True),
        full_html=True,
    )
    print(out_html)
    if args.open:
        try:
            webbrowser.open("file://" + os.path.realpath(out_html))
        except Exception:
            pass


if __name__ == "__main__":
    main()
