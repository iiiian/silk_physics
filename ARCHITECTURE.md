# Silk Physics Codebase Architecture

## Table of Contents
1. [Entry Points and Execution Flow](#entry-points-and-execution-flow)
2. [Overall Architecture](#overall-architecture)
3. [Core Physics Pipeline](#core-physics-pipeline)
4. [Entity-Component System](#entity-component-system)
5. [Physics Solver Flow](#physics-solver-flow)
6. [Collision Detection Pipeline](#collision-detection-pipeline)
7. [File Organization](#file-organization)
8. [GPU Refactoring](#gpu-refactoring)
9. [Data Flow Diagram](#data-flow-diagram)
10. [Key Dependencies](#key-dependencies)

---

## Entry Points and Execution Flow

### Main Entry Point: `demo/src/main.cpp`

The main executable supports two execution modes:

**CLI Arguments:**
- `-c, --config <path>`: Load simulation configuration from JSON file
- `-h, --headless`: Run in headless (non-GUI) mode
- `-o, --output <path>`: Output path for Alembic export (default: `out.abc`)

**Execution Flow:**
```
main.cpp
├─ Parse command-line arguments
├─ Load and validate JSON config (optional)
├─ Branch: Headless Mode vs GUI Mode
│  ├─ HEADLESS: headless_run() → simulation → Alembic export
│  └─ GUI: Demo::run() → Polyscope interactive visualization
└─ Return success/failure status
```

### Headless Pipeline
```
main.cpp
 └─> headless_run()
      ├─ Parse JSON config
      ├─ Load meshes (PLY/OBJ)
      ├─ Create silk::World
      ├─ Add cloth/obstacle objects
      ├─ SIMULATION LOOP:
      │   ├─ world.solver_step()  // Physics timestep
      │   └─ Cache positions
      └─ Export to Alembic (.abc file)
```

### GUI Pipeline
```
main.cpp
 └─> Demo::run()
      ├─ Initialize Polyscope viewport
      ├─ Load ImGui widgets (scene, settings, stats)
      ├─ USER INTERACTION LOOP:
      │   ├─ Edit objects/settings via UI
      │   ├─ Play/pause/step simulation
      │   └─ world.solver_step() when playing
      └─ Optional: Export to Alembic
```

---

## Overall Architecture

The codebase is a **cloth physics simulation engine** with two main components:
1. **silk/** - Core physics library (C++ API)
2. **demo/** - Application layer (GUI + headless CLI)

### Layer 1: Core Physics Library (silk/)

A C++ library (`silk::silk`) providing the core simulation API.

**Public Headers (silk/include/silk/):**
- `silk.hpp` - Main API with `World` class and configuration structs
- `result.hpp` - Error handling without exceptions

**Key Data Structures:**
```cpp
struct MeshConfig       // Triangle mesh definition
struct ClothConfig      // Physical properties (stiffness, damping, density)
struct CollisionConfig  // Collision behavior (restitution, friction, groups)
struct GlobalConfig     // Simulation parameters (dt, iterations, gravity)
class World            // Main simulation controller (pimpl pattern)
```

### Layer 2: Demo Application (demo/)

Interactive GUI and headless CLI built on top of the silk library.

**Entry Components:**
- `demo.cpp/hpp` - Main GUI application with ImGui widget panels
- `headless.cpp/hpp` - Headless runner for batch simulations
- `json_parse.cpp/hpp` - Configuration file parser

**Sub-Components:**
- `objects/` - Cloth and Obstacle wrapper classes
- `widgets/` - ImGui panels (simulator, settings, statistics, etc.)
- `alembic_writer.cpp/hpp` - Alembic (.abc) file export for animation

---

## Core Physics Pipeline

### Public API Layer: `silk::World`

```cpp
World world;
world.add_cloth(mesh, cloth_config, collision_config);
world.add_obstacle(mesh, collision_config);
world.solver_step();  // Advance simulation by dt
```

### Internal Implementation (Pimpl Pattern)

```
silk::World
 └─> WorldImpl
      ├─ Registry (Entity-Component System)
      │   └─ Stores all simulation data as components
      └─ CpuSolverPipeline
           ├─ CpuCollisionPipeline
           └─ Solver utilities
```

---

## Entity-Component System (ECS)

All simulation data is stored as **components** attached to **entities** (handled by `Registry`).

### ECS Component Registry

Defined via macro in `ecs.hpp`:
```cpp
ECS_X_MACRO(X) contains:
├── ClothConfig             - Physical parameters
├── CollisionConfig         - Collision settings
├── TriMesh                - Mesh topology
├── Pin                    - Pinned vertices
├── ClothTopology          - Static mesh analysis (mass, areas, Jacobians)
├── CpuClothSolverContext  - Dynamic solver state (H matrix, Cholesky factor)
├── ObjectState            - Current simulation state (positions, velocities)
├── ObstaclePosition       - Kinematic obstacle pose
└── CpuObjectCollider      - Collision geometry and culling structure
```

### Key Components

- **TriMesh** - Mesh geometry (vertices, edges, faces)
- **ClothConfig** - Physical properties (stiffness, damping, density)
- **CollisionConfig** - Collision settings (restitution, friction, groups)
- **ObjectState** - Current positions & velocities
- **ClothTopology** - Precomputed mesh data (masses, areas, Jacobians)
- **CpuClothSolverContext** - Solver state (system matrix, Cholesky factorization)
- **CpuObjectCollider** - Collision geometry (KD-tree spatial structure)
- **Pin** - Pinned vertex constraints
- **ObstaclePosition** - Kinematic obstacle positions

### Key Class Relationships

**World (Public API) → WorldImpl (Private Implementation)**
```
silk::World (pimpl pattern)
  └── WorldImpl
       ├── Registry (ECS container)
       │    └── Stores all Entity components
       └── CpuSolverPipeline
            ├── CpuCollisionPipeline (for collision detection)
            └── Batch solver utilities
```

---

## Physics Solver Flow (Per Timestep)

Located in: `silk/src/solver/cpu/pipeline.cpp`

```
CpuSolverPipeline::step()
│
├─ OUTER LOOP (up to max_outer_iteration):
│   │
│   ├─ 1. Compute barrier constraints (from collisions)
│   │
│   ├─ 2. Predict next state: x_next = x + dt·v + dt²·a
│   │
│   ├─ 3. INNER LOOP (up to max_inner_iteration):
│   │    ├─ Assemble linear system: Hx = b
│   │    │   where H = mass/dt² + elastic + bending + pins + barriers
│   │    ├─ Solve using CHOLMOD sparse Cholesky: x = H⁻¹b
│   │    └─ Check convergence: ||x - x_next|| < threshold
│   │
│   ├─ 4. Enforce barrier constraints (position clamping)
│   │
│   ├─ 5. Update collision geometry
│   │
│   ├─ 6. Collision Detection (broad + narrow phase CCD)
│   │
│   └─ 7. If collisions found:
│        ├─ Compute time-of-impact (TOI)
│        ├─ Roll back to 80% of TOI (safety margin)
│        └─ Continue to next outer iteration
│
└─ Write final solution back to ObjectState components
```

### Projective Dynamics Solver Details

**Energy Formulation (Per Cloth):**
- **Momentum**: ||v - (v_pred)||² → Inertial term
- **In-plane Elasticity**: ||JWJ||² → Stretching resistance
- **Bending**: ||CWC||² → Out-of-plane curvature resistance
- **Pin Constraints**: ||p - target||² → Pinned vertex targets
- **Collision Barriers**: Soft inequality constraints (if collisions exist)

**Linear System Solve:**
```
Hx = b

where:
H = I/dt² + [JWJ, CWC, pin_constraints, barrier_constraints]
  (Left-hand side: assembled from energy Hessians)
b = (accumulated right-hand sides from RHS computations)

x = solution (new positions)
```

**Solver Backends:**
- Currently: **CPU-based** using CHOLMOD for sparse Cholesky factorization
- **GPU Refactoring**: Architecture prepared with:
  - `solver/cpu/` → Future `solver/gpu/` structure ready
  - `collision/cpu/` → Future `collision/gpu/` structure ready
  - `SolverBackend` enum in GUI for future backend selection
  - Components prefixed with `Cpu` to distinguish from future GPU components

---

## Collision Detection Pipeline

Located in: `silk/src/collision/cpu/pipeline.cpp`

### Two-Phase Approach

**1. Broadphase (Spatial Acceleration):**
- Each `CpuObjectCollider` maintains a KD-Tree of `MeshCollider` primitives
- KD-Tree built from point (vertices) and triangle primitives
- Filters objects by collision group and collision status

**2. Narrowphase (Continuous Collision Detection):**
- Point-triangle (PT) and edge-edge (EE) CCD queries
- Uses `tight_inclusion` library for robust interval root finding
- Computes time-of-impact (TOI) and collision normal/depth
- Tracks collision state across substeps

### Collision Filtering Rules

```cpp
// Object-level filter (object_collision_filter)
- Both objects must have collision enabled (group != -1)
- Must be in same collision group
- At least one object must be dynamic (not pure obstacle)

// Mesh-level self-collision filter
- Rejects topologically adjacent primitives (to prevent overlap)
- Rejects collisions where all vertices are pinned (zero mass)
```

### Collision Data Structure

```cpp
struct Collision {
  CollisionType type;           // PointTriangle or EdgeEdge
  Handle entity_handle_a, b;    // Involved objects
  int state_offset_a, b;        // Where in global state array
  Eigen::Vector4i index;        // Vertex indices of primitives
  float toi;                    // Time of impact [0,1]
  float minimal_separation;     // Barrier distance threshold
  Eigen::Matrix<float,3,4> position_t0, t1;  // Positions at t0, t1
  Eigen::Matrix<float,3,4> velocity_t0, t1;  // Velocities before/after
};
```

---

## File Organization

### Core Library Structure (silk/)

```
silk/
├── include/silk/
│   ├── silk.hpp              ← PUBLIC API (World, configs)
│   └── result.hpp            ← Error handling (no exceptions)
│
└── src/
    ├── world.cpp             ← World::WorldImpl (main controller)
    │
    ├── ecs.hpp/cpp           ← Entity-Component System
    ├── manager.hpp           ← Component storage (dense arrays)
    ├── handle.hpp            ← Safe resource references
    │
    ├── mesh.hpp/cpp          ← TriMesh structure + validation
    ├── cloth_topology.cpp    ← Precompute cloth properties
    │
    ├── solver/cpu/
    │   ├── pipeline.cpp      ← MAIN SOLVER LOOP
    │   ├── cloth_solver_context.cpp  ← Per-cloth solver data
    │   ├── cloth_solver_utils.cpp    ← Cloth computations
    │   ├── obstacle_solver_utils.cpp ← Obstacle updates
    │   ├── barrier_constrain.hpp     ← Collision barriers
    │   └── cholmod_utils.cpp         ← Sparse linear solver
    │
    └── collision/cpu/
        ├── pipeline.cpp      ← COLLISION ORCHESTRATION
        ├── object_collider.cpp  ← Per-object collision geometry
        ├── mesh_collider.hpp    ← Per-triangle collision primitives
        ├── bbox.cpp          ← Axis-aligned bounding boxes
        ├── broadphase.hpp    ← KD-tree spatial queries
        └── narrowphase.cpp   ← CCD queries (PT/EE)
```

### Demo Application Structure (demo/)

```
demo/
├── src/
│   ├── main.cpp              ← ENTRY POINT
│   ├── headless.cpp          ← Batch simulation runner
│   ├── demo.cpp              ← GUI application
│   │
│   ├── objects/
│   │   ├── cloth.hpp         ← GUI cloth wrapper
│   │   ├── obstacle.hpp      ← GUI obstacle wrapper
│   │   ├── headless_cloth.hpp  ← Headless cloth wrapper
│   │   └── headless_obstacle.hpp
│   │
│   ├── widgets/              ← ImGui UI panels
│   │   ├── simulator.cpp     ← Play/pause controls
│   │   ├── scene.cpp         ← Object management
│   │   ├── sim_setting.cpp   ← Global parameters
│   │   ├── statistic.cpp     ← Performance stats
│   │   ├── gpu_solver.cpp    ← GPU backend toggle (UI only)
│   │   └── config.cpp        ← Load/save configs
│   │
│   ├── json_parse.cpp        ← JSON config parser
│   └── alembic_writer.cpp    ← Export to .abc format
│
└── test/                     ← GoogleTest-based tests
```

### GUI Application Components

**Main Components (`demo.cpp`):**
```
Demo
├── Context (holds silk_world and objects)
├── SceneWidget - Object/scene management
├── ObjectSettingWidget - Per-object property editor
├── SimSettingWidget - Global simulation parameters
├── SimulatorWidget - Play/pause/reset controls
├── StatisticWidget - FPS, iteration counts, timings
├── GpuSolverWidget - GPU backend toggle (UI only, not yet functional)
├── ConfigWidget - Load/save simulation configs
└── HelpBarWidget - Keyboard shortcuts and help
```

**Layout:**
- Left Panel (400px): Scene controls, statistics, object settings
- Right Panel (400px): Simulation settings, solver control, GPU toggle, console
- Center: 3D viewport (Polyscope visualization)

---

## GPU Refactoring

### Recent Changes (Commit 4aadec8)

The codebase was reorganized to prepare for GPU implementation:

**Directory Structure Changes:**
```
BEFORE:                          AFTER:
src/
├── solver_pipeline.cpp    →     solver/cpu/pipeline.cpp
├── collision_pipeline.cpp →     collision/cpu/pipeline.cpp
├── object_collider.cpp    →     collision/cpu/object_collider.cpp
├── narrowphase.cpp        →     collision/cpu/narrowphase.cpp
├── bbox.cpp               →     collision/cpu/bbox.cpp
└── solver utils           →     solver/cpu/
```

**Component Naming Convention:**
- CPU-specific components prefixed with `Cpu`:
  - `CpuSolverPipeline`
  - `CpuCollisionPipeline`
  - `CpuObjectCollider`
  - `CpuClothSolverContext`

**Future GPU Integration Points:**
```
Planned Structure:
├── solver/
│   ├── cpu/    - Current CPU implementation
│   └── gpu/    - Future GPU implementation (CUDA/HIP)
│
└── collision/
    ├── cpu/    - Current CPU broadphase/narrowphase
    └── gpu/    - Future GPU acceleration
```

**Backend Selection Mechanism:**
```cpp
enum class SolverBackend { Auto, CPU, GPU };

// In GpuSolverWidget - UI control ready for future implementation
// Main logic: World chooses appropriate pipeline based on backend
```

---

## Data Flow Diagram

```
User Input (CLI args or GUI)
        ↓
[Load JSON Config or Manual Setup]
        ↓
Create silk::World
        ↓
Add Cloth/Obstacle Objects
    ↓                    ↓
Registry stores:      Mesh Validation
- TriMesh             - Topology check
- ClothConfig         - Manifoldness
- CollisionConfig     - Connectivity
- ObjectState
        ↓
[Simulation Loop: world.solver_step()]
        ↓
    ┌───────────────────────────────────┐
    │  CpuSolverPipeline::step()        │
    │  ┌──────────────────────────────┐ │
    │  │ Outer Loop (Collision CCD)   │ │
    │  │  ├─ Predict state            │ │
    │  │  ├─ Inner Loop (Solver)      │ │
    │  │  │   └─ Solve Hx = b         │ │
    │  │  ├─ Collision Detection      │ │
    │  │  └─ CCD Rollback if needed   │ │
    │  └──────────────────────────────┘ │
    └───────────────────────────────────┘
        ↓
Update ObjectState in Registry
        ↓
[GUI: Render in Polyscope]
   OR
[Headless: Cache positions]
        ↓
[Export to Alembic .abc file]
```

---

## Additional Details

### Configuration System

**JSON Configuration Structure (`config.hpp`):**
```json
{
  "global": {
    "dt": 0.01,
    "max_outer_iteration": 120,
    "max_inner_iteration": 80,
    "acceleration": [0, 0, -9.8],
    "total_steps": 500
  },
  "objects": [
    {
      "type": "cloth",
      "name": "dress",
      "mesh": "path/to/model.ply",
      "cloth": {
        "elastic_stiffness": 150.0,
        "bending_stiffness": 0.0002,
        "density": 0.12,
        "damping": 0.02
      },
      "collision": {
        "enabled": true,
        "self_collision": true,
        "group": 1,
        "restitution": 0.25,
        "friction": 0.45
      },
      "transform": {
        "translation": [0, 0, 0.8],
        "rotation_euler_deg": [0, 30, 0],
        "scale": 1.2
      }
    },
    {
      "type": "obstacle",
      "name": "bunny",
      "mesh": "path/to/bunny.ply",
      ...
    }
  ]
}
```

### Handle System (Safe Resource References)

```cpp
struct Handle {
  uint32_t value; // 1 valid bit | 11 generation bits | 20 index bits
  // Max: 1M concurrent resources, 2K generations per slot
}
```

### Mesh Storage

```cpp
struct TriMesh {
  RMatrixX3f V;  // Nx3 vertex positions
  RMatrixX2i E;  // Mx2 edge connectivity
  RMatrixX3i F;  // Kx3 face connectivity (triangles)
  float avg_edge_length;
};
```

### Error Handling

**Result Type Pattern (No Exceptions):**
```cpp
class Result {
  bool success;
  ErrorCode code;
  std::string detail;
};

enum class ErrorCode {
  Unknown, InvalidConfig, TooManyBody, InvalidHandle, InvalidMesh,
  IncorrectPinNum, IncorrectPositionNum,
  CholeskyDecompositionFail,  // Numerical failure
  NeedInitSolverFirst, IterativeSolveFail
};
```

**Usage:**
```cpp
Result r = world.add_cloth(...);
if (!r) {
  std::cerr << r.to_string(); // Human-readable error message
}
```

### Alembic Export Functionality

**Purpose:** Export simulation results as `.abc` (Alembic) files for VFX post-processing.

**Implementation (`alembic_writer.cpp`):**
```cpp
write_scene(path, objects)
├─ Create output archive
├─ For each object with simulation cache:
│  ├─ Build time sampling from cache timestamps
│  ├─ Create OPolyMesh in archive
│  ├─ Write face topology (first frame)
│  └─ Write vertex positions for each time sample
└─ Handle exceptions and directory creation
```

**Cache Structure:**
```cpp
// In IObject interface:
using PositionCache = std::vector<std::pair<float, Vert>>;
// Stores (time, vertex_positions) pairs for entire simulation
```

### Mesh Validation and Loading

**Mesh Validation Levels:**

**Cloth Mesh (Comprehensive):**
- Schema: non-null pointers, correct array dimensions
- Geometry: finite positions, valid face indices, reasonable triangle quality
- Topology: single connected component, manifoldness checks

**Obstacle Mesh (Basic):**
- Schema: non-null pointers, correct array dimensions
- Geometry: finite positions, valid face indices

**Mesh Formats Supported:**
- PLY (via libigl)
- OBJ (via libigl)
- Other formats supported by libigl

---

## Key Dependencies

### Build System

**CMake Configuration:**
```
Root CMakeLists.txt
├── silk/ (library)
│   ├── Dependencies: Eigen, IGL, SuiteSparse, TBB, tight_inclusion
│   └── Builds silk::silk library
└── demo/ (executable)
    ├── Dependencies: silk::silk, Polyscope, ImGui, Alembic, nlohmann_json
    └── Builds 'demo' executable
    └── test/ (googletest-based tests)
```

### Core Dependencies

- **Eigen3** - Linear algebra (matrices, vectors)
- **libigl** - Mesh I/O and geometry utilities
- **SuiteSparse (CHOLMOD)** - Sparse Cholesky solver for Hx=b
- **TBB** - Parallel loops (Intel Threading Building Blocks)
- **tight_inclusion** - Robust CCD root finding
- **Polyscope** - 3D mesh visualization (GUI only)
- **ImGui** - UI toolkit (GUI only)
- **Alembic** - Animation export format
- **nlohmann_json** / custom parser - JSON config loading
- **spdlog** - Logging

---

## Recent Development History

**Key Commits:**
1. **496f026** (Latest): "Add headless CLI workflow with JSON config and Alembic export"
   - Implemented `-h/--headless` mode
   - Added `-c/--config` for JSON configuration
   - Added `-o/--output` for Alembic export path
   - Created `HeadlessCloth` and `HeadlessObstacle` wrappers

2. **a7b75df**: "fix bbox variable shadowing"

3. **226bd45**: "fix kd tree move ctor/assignment operator"

4. **4aadec8**: "refactor to prepare for GPU solver"
   - Major reorganization of collision/solver pipeline structure
   - Added `/cpu` subdirectories
   - Component naming convention (Cpu prefix)
   - Centralized ECS component definitions via X_MACRO

5. **0e70b12**: "implement on-demand growth for Manager"
   - Memory management optimization for ECS

---

## Summary

This codebase implements a **projective dynamics cloth simulator** with:
- Clean separation: library (`silk/`) vs application (`demo/`)
- ECS architecture for data management
- CPU-based physics solver (prepared for GPU)
- Dual-mode execution (GUI interactive + headless batch)
- Production-ready export to Alembic for VFX pipelines
- Robust collision detection with CCD

The execution flow is:
**Config → Meshes → World → Solver Loop → Export**
