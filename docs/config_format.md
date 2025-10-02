# Simulation Config Format

The config file will be a json file containing below sections:

- global setting (Exists once in each config file)
  - acceleration xyz
  - max outer iteration
  - max inner iteration
  - dt
  - max time
- object specific setting (A json list, containing settings for each object)
  - cloth
    - object type (Cloth)
    - ClothConfig
    - CollisionConfig
    - 3D model file path (.off, .obj, .ply, .stl)
    - pin position (optional)
    - transformation (optional)
      - position offset (xyz)
      - rotation (xyz)
      - scaling (xyz)
  - obstacle
    - Object type (Obstacle)
    - CollisionConfig
    - 3D model file path (.off, .obj, .ply, .stl)
    - transformation (optional)
      - position offset (xyz)
      - rotation (xyz)
      - scaling (xyz)
