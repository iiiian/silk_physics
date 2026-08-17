from pathlib import Path

import numpy as np
from plyfile import PlyData, PlyElement
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


SOURCE = Path(
    "build/broadphase-data/"
    "c0f5a2e54fc57a28b54f726eaf3030d7822ae77f/"
    "cloth-ball/frames/cloth_ball0.ply"
)
OUTPUT = Path("model/cloth_ball_sphere.ply")


def main() -> None:
    ply = PlyData.read(SOURCE)
    vertices = np.column_stack([ply["vertex"][axis] for axis in "xyz"])
    faces = np.vstack(ply["face"]["vertex_indices"]).astype(np.int32)

    rows = np.concatenate(
        [faces[:, 0], faces[:, 1], faces[:, 2], faces[:, 1], faces[:, 2], faces[:, 0]]
    )
    columns = np.concatenate(
        [faces[:, 1], faces[:, 2], faces[:, 0], faces[:, 0], faces[:, 1], faces[:, 2]]
    )
    graph = coo_matrix(
        (np.ones(len(rows), dtype=np.uint8), (rows, columns)),
        shape=(len(vertices), len(vertices)),
    )
    component_num, labels = connected_components(graph, directed=False)
    assert component_num == 2

    component_sizes = np.bincount(labels)
    sphere_component = int(np.argmin(component_sizes))
    vertex_ids = np.flatnonzero(labels == sphere_component)
    vertex_map = np.full(len(vertices), -1, dtype=np.int32)
    vertex_map[vertex_ids] = np.arange(len(vertex_ids), dtype=np.int32)
    sphere_faces = faces[np.all(labels[faces] == sphere_component, axis=1)]
    sphere_faces = vertex_map[sphere_faces][:, [0, 2, 1]]

    sphere_vertices = vertices[vertex_ids].astype(np.float32)
    sphere_vertices = sphere_vertices[:, [0, 2, 1]]
    sphere_vertices -= sphere_vertices.mean(axis=0)

    vertex_data = np.empty(
        len(sphere_vertices), dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")]
    )
    vertex_data["x"] = sphere_vertices[:, 0]
    vertex_data["y"] = sphere_vertices[:, 1]
    vertex_data["z"] = sphere_vertices[:, 2]
    face_data = np.empty(len(sphere_faces), dtype=[("vertex_indices", "O")])
    face_data["vertex_indices"] = [face for face in sphere_faces]
    PlyData(
        [
            PlyElement.describe(vertex_data, "vertex"),
            PlyElement.describe(face_data, "face"),
        ],
        text=False,
    ).write(OUTPUT)


if __name__ == "__main__":
    main()
