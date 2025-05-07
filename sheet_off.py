#!/usr/bin/env python3
"""
Generate a .off file for a vertical 10×10 square sheet mesh.
The sheet lies in the x–z plane at y=0, from x=0…10 and z=10…20.
It is subdivided into a 10×10 grid; each quad is split into two triangles.
"""

import sys

def generate_off(filename, width=10.0, height=10.0, divisions=10, top_z=20.0):
    nx = divisions
    nz = divisions
    bottom_z = top_z - height

    # 1) Generate vertices: (x, y=0, z)
    vertices = []
    for j in range(nz + 1):
        z = bottom_z + (height / nz) * j
        for i in range(nx + 1):
            x = (width / nx) * i
            vertices.append((x, 0.0, z))

    # 2) Generate faces: two triangles per cell
    faces = []
    for j in range(nz):
        for i in range(nx):
            # indices of the four corners of this cell
            v0 = j * (nx + 1) + i
            v1 = v0 + 1
            v2 = (j + 1) * (nx + 1) + i
            v3 = v2 + 1
            # split the quad (v0,v1,v3,v2) along the main diagonal v0→v3
            faces.append((v0, v1, v3))
            faces.append((v0, v3, v2))

    # 3) Write the OFF file
    with open(filename, 'w') as f:
        f.write("OFF\n")
        f.write(f"{len(vertices)} {len(faces)} 0\n")
        for x, y, z in vertices:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
        for a, b, c in faces:
            f.write(f"3 {a} {b} {c}\n")

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "sheet.off"
    generate_off(out)
    print(f"Wrote '{out}': { (10+1)*(10+1) } vertices, { 2*10*10 } triangles.")

