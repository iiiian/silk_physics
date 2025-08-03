import pandas as pd
from dash import Dash, html
import sys
from pathlib import Path
from dash_vtk import GeometryRepresentation, PolyData, View
import numpy as np


def interpolate(x0: np.ndarray, x1: np.ndarray, t: float) -> np.ndarray:
    return x0 + t * (x1 - x0)


def make_pt_poly_data(df: pd.DataFrame, idx: int, t: float) -> PolyData:
    assert df.iloc[idx]["type"] == "PointTriangle"
    assert t >= 0.0 and t <= 1.0

    d = df.iloc[idx]
    toi = d["toi"]

    # vertices
    points = []
    # vertices at t0
    points += d["x00"].tolist()  # 0
    points += d["x10"].tolist()  # 1
    points += d["x20"].tolist()  # 2
    points += d["x30"].tolist()  # 3
    # vertices at t
    points += interpolate(d["x00"], d["x01"], t).tolist()  # 4
    points += interpolate(d["x10"], d["x11"], t).tolist()  # 5
    points += interpolate(d["x20"], d["x21"], t).tolist()  # 6
    points += interpolate(d["x30"], d["x31"], t).tolist()  # 7
    if t >= toi:
        # vertices at collision
        points += interpolate(d["x00"], d["x01"], toi).tolist()  # 8
        points += interpolate(d["x10"], d["x11"], toi).tolist()  # 9
        points += interpolate(d["x20"], d["x21"], toi).tolist()  # 10
        points += interpolate(d["x30"], d["x31"], toi).tolist()  # 11

        # vertices after reflection
        tr = (t - toi) / (1.0 - toi)
        points += interpolate(d["x0r"], d["x01"], tr).tolist()  # 12
        points += interpolate(d["x1r"], d["x11"], tr).tolist()  # 13
        points += interpolate(d["x2r"], d["x21"], tr).tolist()  # 14
        points += interpolate(d["x3r"], d["x31"], tr).tolist()  # 15

    # triangles
    polys = []
    # current triangle
    polys += [3, 5, 6, 7]
    if t >= toi:
        # triangle at collision
        polys += [3, 9, 10, 11]
        # triangle after reflection
        polys += [3, 13, 14, 15]

    # trajectories
    lines = []
    # vertex trajectories from t0 to t
    lines += [2, 0, 4]
    lines += [2, 1, 5]
    lines += [2, 2, 6]
    lines += [2, 3, 7]
    if t >= toi:
        # reflection trajectories from toi to t
        lines += [2, 8, 12]
        lines += [2, 9, 13]
        lines += [2, 10, 14]
        lines += [2, 11, 15]

    return PolyData(points=points, lines=lines, polys=polys)


def make_ee_poly_data(df: pd.DataFrame, idx: int, t: float) -> PolyData:
    assert df.iloc[idx]["type"] == "EdgeEdge"
    assert t >= 0.0 and t <= 1.0

    d = df.iloc[idx]
    toi = d["toi"]

    # vertices
    points = []
    # vertices at t0
    points += d["x00"].tolist()  # 0
    points += d["x10"].tolist()  # 1
    points += d["x20"].tolist()  # 2
    points += d["x30"].tolist()  # 3
    # vertices at t
    points += interpolate(d["x00"], d["x01"], t).tolist()  # 4
    points += interpolate(d["x10"], d["x11"], t).tolist()  # 5
    points += interpolate(d["x20"], d["x21"], t).tolist()  # 6
    points += interpolate(d["x30"], d["x31"], t).tolist()  # 7
    if t >= toi:
        # vertices at collision
        points += interpolate(d["x00"], d["x01"], toi).tolist()  # 8
        points += interpolate(d["x10"], d["x11"], toi).tolist()  # 9
        points += interpolate(d["x20"], d["x21"], toi).tolist()  # 10
        points += interpolate(d["x30"], d["x31"], toi).tolist()  # 11

        # vertices after reflection
        tr = (t - toi) / (1.0 - toi)
        points += interpolate(d["x0r"], d["x01"], tr).tolist()  # 12
        points += interpolate(d["x1r"], d["x11"], tr).tolist()  # 13
        points += interpolate(d["x2r"], d["x21"], tr).tolist()  # 14
        points += interpolate(d["x3r"], d["x31"], tr).tolist()  # 15

    # edges
    lines = []
    # current edges
    lines += [2, 4, 5]
    lines += [2, 6, 7]
    if t >= toi:
        # triangle at collision
        lines += [2, 8, 9]
        lines += [2, 10, 11]
        # triangle after reflection
        lines += [2, 12, 13]
        lines += [2, 14, 15]

    # trajectories
    # vertex trajectories from t0 to t
    lines += [2, 0, 4]
    lines += [2, 1, 5]
    lines += [2, 2, 6]
    lines += [2, 3, 7]
    if t >= toi:
        # reflection trajectories from toi to t
        lines += [2, 8, 12]
        lines += [2, 9, 13]
        lines += [2, 10, 14]
        lines += [2, 11, 15]

    return PolyData(points=points, lines=lines)


def make_poly_data(df: pd.DataFrame, idx: int, t: float) -> PolyData:
    if df.iloc[idx]["type"] == "PointTriangle":
        return make_pt_poly_data(df, idx, t)
    else:
        return make_ee_poly_data(df, idx, t)


def parse_reflection_csv(path: Path) -> pd.DataFrame:
    def parse_coordinate(string):
        return np.fromiter(string.split(","), dtype=float)

    converters = {
        "x00": parse_coordinate,
        "x10": parse_coordinate,
        "x20": parse_coordinate,
        "x30": parse_coordinate,
        "x01": parse_coordinate,
        "x11": parse_coordinate,
        "x21": parse_coordinate,
        "x31": parse_coordinate,
        "x0r": parse_coordinate,
        "x1r": parse_coordinate,
        "x2r": parse_coordinate,
        "x3r": parse_coordinate,
    }

    df = pd.read_csv(path, sep=";", converters=converters)
    return df


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python ./ccd_viewer.py path_to_csv")
        exit(1)

    csv_path = Path(sys.argv[1])
    df = parse_reflection_csv(csv_path)
    # print(df)

    app = Dash()

    poly_data = make_poly_data(df, 0, 0.0)
    app.layout = html.Div(
        style={"width": "100%", "height": "100vh"},
        children=[
            View(
                [
                    GeometryRepresentation(
                        [
                            poly_data,
                        ]
                    )
                ]
            )
        ],
    )

    app.run(debug=True)
