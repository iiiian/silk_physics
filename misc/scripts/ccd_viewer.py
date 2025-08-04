import pandas as pd
from dash import Dash, dcc, html
from dash.dependencies import Input, Output
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

    if t >= toi:
        # vertices
        points = []
        # vertices at t0
        points += d["x00"].tolist()  # 0
        points += d["x10"].tolist()  # 1
        points += d["x20"].tolist()  # 2
        points += d["x30"].tolist()  # 3
        # vertices at collision
        points += interpolate(d["x00"], d["x01"], toi).tolist()  # 4
        points += interpolate(d["x10"], d["x11"], toi).tolist()  # 5
        points += interpolate(d["x20"], d["x21"], toi).tolist()  # 6
        points += interpolate(d["x30"], d["x31"], toi).tolist()  # 7
        # vertices after reflection
        tr = (t - toi) / (1.0 - toi)
        points += interpolate(d["x0r"], d["x01"], tr).tolist()  # 8
        points += interpolate(d["x1r"], d["x11"], tr).tolist()  # 9
        points += interpolate(d["x2r"], d["x21"], tr).tolist()  # 10
        points += interpolate(d["x3r"], d["x31"], tr).tolist()  # 11

        # triangles
        polys = []
        # triangle at collision
        polys += [3, 5, 6, 7]
        # triangle after reflection
        polys += [3, 9, 10, 11]

        # trajectories
        lines = []
        # trajectories from t0 to toi
        lines += [2, 0, 4]
        lines += [2, 1, 5]
        lines += [2, 2, 6]
        lines += [2, 3, 7]
        # reflection trajectories from toi to t
        lines += [2, 4, 8]
        lines += [2, 5, 9]
        lines += [2, 6, 10]
        lines += [2, 7, 11]

        return PolyData(points=points, lines=lines, polys=polys)
    else:
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

        # triangles
        polys = [3, 5, 6, 7]

        # trajectories
        lines = []
        # trajectories from t0 to t
        lines += [2, 0, 4]
        lines += [2, 1, 5]
        lines += [2, 2, 6]
        lines += [2, 3, 7]

        return PolyData(points=points, lines=lines, polys=polys)


def make_ee_poly_data(df: pd.DataFrame, idx: int, t: float) -> PolyData:
    assert df.iloc[idx]["type"] == "EdgeEdge"
    assert t >= 0.0 and t <= 1.0

    d = df.iloc[idx]
    toi = d["toi"]

    if t >= toi:
        # vertices
        points = []
        # vertices at t0
        points += d["x00"].tolist()  # 0
        points += d["x10"].tolist()  # 1
        points += d["x20"].tolist()  # 2
        points += d["x30"].tolist()  # 3
        # vertices at collision
        points += interpolate(d["x00"], d["x01"], toi).tolist()  # 4
        points += interpolate(d["x10"], d["x11"], toi).tolist()  # 5
        points += interpolate(d["x20"], d["x21"], toi).tolist()  # 6
        points += interpolate(d["x30"], d["x31"], toi).tolist()  # 7
        # vertices after reflection
        tr = (t - toi) / (1.0 - toi)
        points += interpolate(d["x0r"], d["x01"], tr).tolist()  # 8
        points += interpolate(d["x1r"], d["x11"], tr).tolist()  # 9
        points += interpolate(d["x2r"], d["x21"], tr).tolist()  # 10
        points += interpolate(d["x3r"], d["x31"], tr).tolist()  # 11

        # edges
        lines = []
        # edge at collision
        lines += [2, 4, 5]
        lines += [2, 6, 7]
        # edge after reflection
        lines += [2, 8, 9]
        lines += [2, 10, 11]

        # trajectories
        # trajectories from t0 to toi
        lines += [2, 0, 4]
        lines += [2, 1, 5]
        lines += [2, 2, 6]
        lines += [2, 3, 7]
        # reflection trajectories from toi to t
        lines += [2, 4, 8]
        lines += [2, 5, 9]
        lines += [2, 6, 10]
        lines += [2, 7, 11]

        return PolyData(points=points, lines=lines)
    else:
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

        # edges
        lines = []
        lines += [2, 4, 5]
        lines += [2, 6, 7]

        # trajectories from t0 to t
        lines += [2, 0, 4]
        lines += [2, 1, 5]
        lines += [2, 2, 6]
        lines += [2, 3, 7]

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

    app = Dash()

    app.layout = html.Div(
        style={
            "display": "flex",
            "flexDirection": "column",
            "width": "100vw",
            "height": "100vh",
            "margin": 0,
            "padding": 0,
        },
        children=[
            html.Div(
                style={"flex": "0 0 auto", "padding": "5px"},
                children=[
                    dcc.Slider(
                        id="time-slider",
                        min=0,
                        max=1,
                        step=0.01,
                        value=0,
                        marks={i / 10: str(i / 10) for i in range(11)},
                    )
                ],
            ),
            html.Div(
                style={"flex": "1 1 auto", "overflow": "hidden"},
                children=[
                    View(
                        id="view",
                        style={"width": "100%", "height": "100%"},
                        children=[
                            GeometryRepresentation(
                                id="geometry",
                                children=[
                                    make_poly_data(df, 0, 0.0),
                                ],
                            )
                        ],
                    )
                ],
            ),
        ],
    )

    @app.callback(
        Output("geometry", "children"),
        [Input("time-slider", "value")],
    )
    def update_figure(t):
        return [make_poly_data(df, 0, t)]

    app.run(debug=True)
