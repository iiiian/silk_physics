import pandas as pd
from dash import Dash, dcc, html, dash_table
from dash.dependencies import Input, Output, State
import sys
from pathlib import Path
from dash_vtk import GeometryRepresentation, PolyData, View
import numpy as np
import dash


# ==============================================================================
# Data Processing
# ==============================================================================


def interpolate(x0: np.ndarray, x1: np.ndarray, t: float) -> np.ndarray:
    """Linearly interpolate between two points."""
    return x0 + t * (x1 - x0)


def get_vertex_positions_at_t(d: pd.Series, t: float) -> list:
    """Calculate the positions of all 4 vertices at a given time t."""
    assert "toi" in d and pd.notna(d["toi"])
    toi = d["toi"]

    if t < toi:
        p0 = interpolate(d["x00"], d["x01"], t)
        p1 = interpolate(d["x10"], d["x11"], t)
        p2 = interpolate(d["x20"], d["x21"], t)
        p3 = interpolate(d["x30"], d["x31"], t)
    else:
        # Position at time of impact
        p0_toi = interpolate(d["x00"], d["x01"], toi)
        p1_toi = interpolate(d["x10"], d["x11"], toi)
        p2_toi = interpolate(d["x20"], d["x21"], toi)
        p3_toi = interpolate(d["x30"], d["x31"], toi)

        # Calculate how far into the reflection we are
        reflection_alpha = 0
        if (1.0 - toi) > 1e-9:  # Avoid division by zero
            reflection_alpha = (t - toi) / (1.0 - toi)
        reflection_alpha = np.clip(reflection_alpha, 0, 1)

        # Interpolate from TOI to reflected position
        p0 = interpolate(p0_toi, d["x0r"], reflection_alpha)
        p1 = interpolate(p1_toi, d["x1r"], reflection_alpha)
        p2 = interpolate(p2_toi, d["x2r"], reflection_alpha)
        p3 = interpolate(p3_toi, d["x3r"], reflection_alpha)

    return [p0, p1, p2, p3]


def get_trajectory_data(d: pd.Series, t: float) -> PolyData:
    """Get the trajectory lines from t0 to t."""
    toi = d["toi"]
    points = []
    lines = []

    # Vertices at t0
    points.extend([d["x00"], d["x10"], d["x20"], d["x30"]])
    v_t0_indices = [0, 1, 2, 3]

    if t < toi:
        # Vertices at t
        points.extend(get_vertex_positions_at_t(d, t))
        v_t_indices = [4, 5, 6, 7]
        # Trajectories from t0 to t
        for i in range(4):
            lines.extend([2, v_t0_indices[i], v_t_indices[i]])
    else:
        # Vertices at toi
        v_toi = [
            interpolate(d["x00"], d["x01"], toi),
            interpolate(d["x10"], d["x11"], toi),
            interpolate(d["x20"], d["x21"], toi),
            interpolate(d["x30"], d["x31"], toi),
        ]
        points.extend(v_toi)
        v_toi_indices = [4, 5, 6, 7]

        # Vertices at t (after reflection)
        points.extend(get_vertex_positions_at_t(d, t))
        v_t_indices = [8, 9, 10, 11]

        # Trajectories from t0 to toi
        for i in range(4):
            lines.extend([2, v_t0_indices[i], v_toi_indices[i]])
        # Trajectories from toi to t
        for i in range(4):
            lines.extend([2, v_toi_indices[i], v_t_indices[i]])

    flat_points = [coord for point in points for coord in point]
    return PolyData(points=flat_points, lines=lines)


# ==============================================================================
# UI Component Creation
# ==============================================================================


def create_scene_geometries(d: pd.Series, t: float, point_size: float, edge_thickness: float, trajectory_thickness: float) -> list:
    """Create all the geometry representations for the scene."""
    vertices = get_vertex_positions_at_t(d, t)
    geometries = []

    if d["type"] == "PointTriangle":
        # Point (red)
        point_polydata = PolyData(points=vertices[0].tolist())
        geometries.append(
            GeometryRepresentation(
                children=[point_polydata],
                property={"color": [1, 0, 0], "pointSize": point_size},
            )
        )
        # Triangle (blue)
        tri_points = [vertices[1], vertices[2], vertices[3]]
        tri_polydata = PolyData(
            points=[p for p in tri_points for p in p],
            polys=[3, 0, 1, 2],
        )
        geometries.append(
            GeometryRepresentation(
                children=[tri_polydata],
                property={"color": [0, 0, 1], "lineWidth": edge_thickness},
                representation="wireframe",
            )
        )
    elif d["type"] == "EdgeEdge":
        # Edge 1 (red)
        edge1_points = [vertices[0], vertices[1]]
        edge1_polydata = PolyData(
            points=[p for p in edge1_points for p in p],
            lines=[2, 0, 1],
        )
        geometries.append(
            GeometryRepresentation(
                children=[edge1_polydata],
                property={"color": [1, 0, 0], "lineWidth": edge_thickness},
            )
        )
        # Edge 2 (blue)
        edge2_points = [vertices[2], vertices[3]]
        edge2_polydata = PolyData(
            points=[p for p in edge2_points for p in p],
            lines=[2, 0, 1],
        )
        geometries.append(
            GeometryRepresentation(
                children=[edge2_polydata],
                property={"color": [0, 0, 1], "lineWidth": edge_thickness},
            )
        )

    # Trajectories (yellow)
    trajectory_polydata = get_trajectory_data(d, t)
    geometries.append(
        GeometryRepresentation(
            children=[trajectory_polydata],
            property={"color": [1, 1, 0], "lineWidth": trajectory_thickness},
        )
    )

    return geometries


def create_vertex_table_data(d: pd.Series, t: float) -> list[dict]:
    """Create the data for the vertex position table."""
    vertices = get_vertex_positions_at_t(d, t)
    table_data = []
    names = []
    if d["type"] == "PointTriangle":
        names = ["Point", "T1_V1", "T1_V2", "T1_V3"]
    else:
        names = ["E1_V1", "E1_V2", "E2_V1", "E2_V2"]

    for i, v in enumerate(vertices):
        table_data.append(
            {
                "Vertex": names[i],
                "X": f"{v[0]:.4f}",
                "Y": f"{v[1]:.4f}",
                "Z": f"{v[2]:.4f}",
            }
        )
    return table_data


def parse_reflection_csv(path: Path) -> pd.DataFrame:
    """Parse the reflection CSV into a DataFrame."""
    def parse_coordinate(string):
        return np.fromiter(string.split(","), dtype=float)

    converters = {f"x{i}{j}": parse_coordinate for i in range(4) for j in range(2)}
    converters.update({f"x{i}r": parse_coordinate for i in range(4)})

    df = pd.read_csv(path, sep=";", converters=converters)
    return df


# ==============================================================================
# Main App
# ==============================================================================


def main():
    if len(sys.argv) != 2:
        print("Usage: python ./ccd_viewer.py path_to_csv")
        exit(1)

    csv_path = Path(sys.argv[1])
    df = parse_reflection_csv(csv_path)
    # For now, we only view the first entry.
    # A dropdown could be added later to select the entry.
    d = df.iloc[0]
    toi = d["toi"]

    # Create marks for the time slider
    time_slider_marks = {
        toi: {
            "label": f"TOI={toi:.2f}",
            "style": {"color": "#f50", "fontWeight": "bold"}
        }
    }

    app = Dash(__name__, external_stylesheets=['https://codepen.io/chriddyp/pen/bWLwgP.css'])

    app.layout = html.Div(
        style={"display": "flex", "height": "100vh", "width": "100vw", "margin": "0"},
        children=[
            # Left Panel
            html.Div(
                style={
                    "width": "350px",
                    "padding": "20px",
                    "display": "flex",
                    "flexDirection": "column",
                    "gap": "20px",
                    "borderRight": "1px solid #ccc",
                    "overflowY": "auto",
                },
                children=[
                    html.H4("Controls"),
                    html.Label("Time (t)"),
                    dcc.Slider(
                        id="time-slider",
                        min=0,
                        max=1,
                        step=0.01,
                        value=0,
                        marks=time_slider_marks,
                        tooltip={"placement": "bottom", "always_visible": True},
                    ),
                    html.Button('Play', id='play-pause-button', n_clicks=0, style={"width": "100%"}),
                    dcc.Interval(id='interval-component', interval=100, n_intervals=0, disabled=True),
                    html.Label("Point Size"),
                    dcc.Slider(
                        id="point-size-slider",
                        min=1,
                        max=20,
                        step=1,
                        value=10,
                        marks={i: str(i) for i in range(1, 21, 2)},
                        tooltip={"placement": "bottom", "always_visible": True},
                    ),
                    html.Label("Edge Thickness"),
                    dcc.Slider(
                        id="edge-thickness-slider",
                        min=1,
                        max=10,
                        step=1,
                        value=5,
                        marks={i: str(i) for i in range(1, 11)},
                        tooltip={"placement": "bottom", "always_visible": True},
                    ),
                    html.Label("Trajectory Thickness"),
                    dcc.Slider(
                        id="trajectory-thickness-slider",
                        min=1,
                        max=10,
                        step=1,
                        value=2,
                        marks={i: str(i) for i in range(1, 11)},
                        tooltip={"placement": "bottom", "always_visible": True},
                    ),
                    html.Hr(),
                    html.H4("Vertex Positions at t"),
                    dash_table.DataTable(
                        id="vertex-table",
                        columns=[
                            {"name": "Vertex", "id": "Vertex"},
                            {"name": "X", "id": "X"},
                            {"name": "Y", "id": "Y"},
                            {"name": "Z", "id": "Z"},
                        ],
                        style_cell={'textAlign': 'left'},
                        style_header={
                            'backgroundColor': 'rgb(230, 230, 230)',
                            'fontWeight': 'bold'
                        },
                    ),
                ],
            ),
            # Right Panel
            html.Div(
                style={"flex": "1", "overflow": "hidden"},
                children=[
                    View(id="view", style={"width": "100%", "height": "100%"}, background=[0.5, 0.5, 0.5]),
                ],
            ),
        ],
    )

    @app.callback(
        Output("view", "children"),
        Output("vertex-table", "data"),
        Input("time-slider", "value"),
        Input("point-size-slider", "value"),
        Input("edge-thickness-slider", "value"),
        Input("trajectory-thickness-slider", "value"),
    )
    def update_view(t, point_size, edge_thickness, trajectory_thickness):
        geometries = create_scene_geometries(d, t, point_size, edge_thickness, trajectory_thickness)
        table_data = create_vertex_table_data(d, t)
        return geometries, table_data

    @app.callback(
        Output('interval-component', 'disabled'),
        Output('play-pause-button', 'children'),
        Input('play-pause-button', 'n_clicks'),
        State('interval-component', 'disabled'),
    )
    def toggle_animation(n_clicks, is_disabled):
        if n_clicks:
            if is_disabled:
                return False, 'Pause'
            else:
                return True, 'Play'
        return True, 'Play'

    @app.callback(
        Output('time-slider', 'value'),
        Input('interval-component', 'n_intervals'),
        State('time-slider', 'value'),
    )
    def update_slider(n, current_time):
        if current_time >= 1:
            return 0
        return round(current_time + 0.02, 2)

    app.run(debug=True)


if __name__ == "__main__":
    main()
