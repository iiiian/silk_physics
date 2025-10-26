# GPU Solver - Quick Start Guide

## Prerequisites

Before you can run the GPU solver, ensure you have:

### Required Hardware
- ✅ **NVIDIA GPU** with Compute Capability 6.0 or higher
  - GTX 10xx series (Pascal) or newer
  - RTX 20xx/30xx/40xx series
  - Tesla P100 or newer
  - Quadro P-series or newer

### Required Software
- ✅ **CUDA Toolkit** 11.0 or later
  - Download from: https://developer.nvidia.com/cuda-downloads
  - Verify installation: `nvcc --version`

- ✅ **NVIDIA GPU Drivers**
  - Latest drivers recommended
  - Verify installation: `nvidia-smi`

- ✅ **CMake** 3.18 or later
  - Required for CUDA support
  - Verify: `cmake --version`

---

## Step 1: Build the Project

### On Windows (using Visual Studio):

```bash
# Navigate to project directory
cd C:\Users\mutha\Desktop\fall2025\gpu\reFactoredCode\silk_physics

# Create build directory
mkdir build
cd build

# Configure with CMake
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build (this will compile CUDA code)
cmake --build . --config Release -j

# You should see CUDA compilation messages like:
# [ 95%] Building CUDA object silk/CMakeFiles/silk.dir/src/solver/gpu/jacobi_solver.cu.o
# [ 96%] Building CUDA object silk/CMakeFiles/silk.dir/src/solver/gpu/cloth_solver_context.cu.o
# ...
```

### Troubleshooting Build Issues:

**If CMake doesn't find CUDA:**
```bash
# Specify CUDA compiler explicitly
cmake .. -DCMAKE_CUDA_COMPILER="C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.0/bin/nvcc.exe"
```

**If you get architecture errors:**
```bash
# The CMakeLists.txt targets SM 60-86
# If your GPU is newer/older, edit silk/CMakeLists.txt line 40:
# CUDA_ARCHITECTURES "60;70;75;80;86"  # Change to match your GPU
```

---

## Step 2: Run with GPU Solver

You have **3 ways** to use the GPU solver:

### Method 1: GUI Mode (Interactive)

This is the easiest way to test the GPU solver:

```bash
# From build directory
cd demo
./demo  # or demo.exe on Windows

# Or specify Release build explicitly
./Release/demo.exe  # Windows
./demo              # Linux
```

**In the GUI:**
1. **Load a scene**:
   - Click `File` → `Load Config`
   - Browse to `misc/example_config/bunny_and_cloth.json`
   - Or manually add cloth: `Scene` → `Add Cloth`

2. **Enable GPU solver**:
   - Look for **"Solver"** panel (usually on the right sidebar)
   - Check the box: ☑️ **"Use GPU solver"**
   - Console should show: `[UI] Solver backend switched to GPU (CUDA Jacobi)`

3. **Run simulation**:
   - Click **Play** button (or press Space)
   - Watch the cloth simulate using GPU!

4. **Verify it's using GPU**:
   - Open Task Manager (Ctrl+Shift+Esc) → Performance → GPU
   - Or run `nvidia-smi` in another terminal
   - You should see GPU utilization spike during simulation

5. **Switch back to CPU** (optional):
   - Uncheck ☐ **"Use GPU solver"**
   - Console shows: `[UI] Solver backend switched to CPU (CHOLMOD Cholesky)`

---

### Method 2: Headless Mode (Batch Processing)

Run simulations without GUI and export to Alembic:

```bash
# Basic headless run with GPU
./demo --headless --config ../misc/example_config/bunny_and_cloth.json --output gpu_result.abc
```

**But wait!** The headless mode doesn't have a way to select GPU from command line yet. You need to modify the code:

**Temporary workaround** - Edit `demo/src/headless.cpp`:

Find the `headless_run()` function and add this after creating the `World`:

```cpp
void headless_run(const config::SimConfig& config, const std::string& output_path) {
  silk::World silk_world;

  // ADD THIS LINE TO USE GPU:
  silk_world.set_solver_backend(silk::SolverBackend::GPU);

  // ... rest of the function ...
}
```

Then rebuild and run:
```bash
cmake --build . --config Release
./demo --headless --config ../misc/example_config/bunny_and_cloth.json --output gpu_result.abc
```

---

### Method 3: C++ API (Programmatic)

Use the GPU solver directly in your own C++ code:

```cpp
#include <silk/silk.hpp>

int main() {
  // Create world
  silk::World world;

  // *** ENABLE GPU SOLVER ***
  auto result = world.set_solver_backend(silk::SolverBackend::GPU);
  if (!result) {
    std::cerr << "Failed to enable GPU: " << result.to_string() << std::endl;
    return 1;
  }

  // Configure simulation
  silk::GlobalConfig global_config;
  global_config.dt = 1.0f / 60.0f;               // 60 FPS
  global_config.acceleration_z = -9.8f;          // Gravity
  global_config.max_outer_iteration = 100;       // Collision substeps
  global_config.max_inner_iteration = 100;       // GPU Jacobi iterations
  world.set_global_config(global_config);

  // Add cloth
  silk::ClothConfig cloth_config;
  cloth_config.elastic_stiffness = 150.0f;
  cloth_config.bending_stiffness = 0.0002f;
  cloth_config.density = 0.12f;
  cloth_config.damping = 0.02f;

  silk::CollisionConfig collision_config;
  collision_config.is_collision_on = true;
  collision_config.is_self_collision_on = true;

  // Load mesh (vertices and faces)
  std::vector<float> vertices = { /* ... */ };
  std::vector<int> faces = { /* ... */ };
  silk::MeshConfig mesh_config;
  mesh_config.verts = {vertices.data(), (int)vertices.size()};
  mesh_config.faces = {faces.data(), (int)faces.size()};

  // Pin some vertices
  std::vector<int> pin_indices = {0, 1, 2};  // Pin top corners

  uint32_t cloth_handle;
  result = world.add_cloth(cloth_config, collision_config, mesh_config,
                           {pin_indices.data(), (int)pin_indices.size()},
                           cloth_handle);
  if (!result) {
    std::cerr << "Failed to add cloth: " << result.to_string() << std::endl;
    return 1;
  }

  // Simulation loop
  for (int frame = 0; frame < 1000; ++frame) {
    // Step simulation (uses GPU!)
    result = world.solver_step();
    if (!result) {
      std::cerr << "Solver failed: " << result.to_string() << std::endl;
      break;
    }

    // Get cloth positions for rendering/export
    int num_vertices = vertices.size() / 3;
    std::vector<float> current_positions(num_vertices * 3);
    world.get_cloth_position(cloth_handle,
                            {current_positions.data(), (int)current_positions.size()});

    // Do something with positions (render, export, etc.)
  }

  // Optional: Switch back to CPU
  world.set_solver_backend(silk::SolverBackend::CPU);

  return 0;
}
```

---

## Step 3: Verify GPU is Being Used

### Check GPU Utilization (Windows):

**Option 1: Task Manager**
1. Press `Ctrl+Shift+Esc`
2. Go to `Performance` tab
3. Select `GPU` (might be "GPU 0" or "GPU 1")
4. Look at "3D" or "Compute" graph
5. Run simulation - should see activity spike

**Option 2: NVIDIA System Monitor**
1. Open NVIDIA Control Panel
2. Desktop → Display GPU Activity Icon in Notification Area
3. Click the GPU icon in system tray
4. Should show "CUDA - demo.exe" when running

**Option 3: Command Line**
```bash
# In a separate terminal, run this while simulation is running:
nvidia-smi -l 1

# You should see:
# +-----------------------------------------------------------------------------+
# | Processes:                                                                  |
# |  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
# |        ID   ID                                                   Usage      |
# |=============================================================================|
# |    0   N/A  N/A     12345    C     demo.exe                         128MiB  |
# +-----------------------------------------------------------------------------+
```

### Check GPU Utilization (Linux):

```bash
# Watch GPU usage in real-time
watch -n 1 nvidia-smi

# Or more detailed:
nvtop  # If installed (apt install nvtop)
```

---

## Performance Comparison

To compare CPU vs GPU performance:

### In GUI:
1. Load a scene (preferably large mesh, >3000 vertices)
2. Open `Statistics` panel
3. Note the **FPS** and **Solver Time**
4. **With CPU**: Run for 100 frames, note average FPS
5. **Switch to GPU**: Run for 100 frames, note average FPS
6. Compare!

### Example Output:
```
CPU Solver:
- FPS: 15-20 (large mesh)
- Solver Time: 45-60 ms per step

GPU Solver:
- FPS: 40-60 (large mesh)
- Solver Time: 15-25 ms per step

Speedup: ~2.5x on this mesh
```

**Note**: GPU speedup increases with mesh size. Small meshes (<1000 vertices) might be **slower** on GPU due to transfer overhead.

---

## Common Issues & Solutions

### Issue 1: "CUDA Error: no kernel image available"

**Cause**: Your GPU architecture isn't in the compiled CUDA code.

**Solution**:
1. Check your GPU compute capability:
   ```bash
   nvidia-smi --query-gpu=compute_cap --format=csv
   # Output: compute_cap
   #         8.6  (for RTX 3090, for example)
   ```

2. Edit `silk/CMakeLists.txt` line 40:
   ```cmake
   set_target_properties(silk PROPERTIES
       CUDA_ARCHITECTURES "60;70;75;80;86"  # Add your compute capability
   )
   ```

3. Rebuild:
   ```bash
   cd build
   cmake --build . --config Release
   ```

---

### Issue 2: "CUDA Error: out of memory"

**Cause**: GPU doesn't have enough VRAM for the mesh size.

**Solutions**:
- Use a smaller mesh
- Reduce `max_inner_iteration` in global config (less GPU memory needed)
- Close other GPU applications (browsers, games, etc.)
- Use a GPU with more VRAM

**Check GPU memory usage**:
```bash
nvidia-smi --query-gpu=memory.used,memory.total --format=csv
```

---

### Issue 3: GPU solver crashes or produces NaN

**Cause**: Numerical instability in Jacobi iteration.

**Solutions**:
1. **Reduce time step**:
   ```cpp
   global_config.dt = 1.0f / 120.0f;  // Smaller timestep (was 1/60)
   ```

2. **Increase damping**:
   ```cpp
   cloth_config.damping = 0.05f;  // More damping (was 0.02)
   ```

3. **Check for bad mesh**:
   - Ensure mesh has no degenerate triangles
   - Mesh should be manifold
   - All vertices should have mass (not all pinned)

4. **Switch to CPU solver** (more numerically stable):
   ```cpp
   world.set_solver_backend(silk::SolverBackend::CPU);
   ```

---

### Issue 4: Build fails with "CUDA language not enabled"

**Solution**: Update CMake to 3.18 or later.

```bash
# Check version
cmake --version

# On Ubuntu:
sudo apt remove cmake
sudo snap install cmake --classic

# On Windows:
# Download latest from: https://cmake.org/download/
```

---

### Issue 5: No speedup / GPU slower than CPU

**Causes & Solutions**:

1. **Mesh too small** (<1000 vertices)
   - GPU has overhead for small problems
   - Use CPU for small meshes

2. **CPU-GPU transfer bottleneck**
   - Currently D and b are uploaded every frame
   - This is expected; future optimization could keep more data on GPU

3. **Jacobi iterations not converging**
   - Increase `max_inner_iteration`:
     ```cpp
     global_config.max_inner_iteration = 200;  // Was 100
     ```

4. **Collision detection dominates**
   - Collision is still on CPU
   - For collision-heavy scenes, CPU/GPU time might be similar
   - Future: move collision to GPU

---

## Benchmark Example

Here's a simple benchmark script to compare CPU vs GPU:

```cpp
#include <silk/silk.hpp>
#include <chrono>
#include <iostream>

void benchmark_solver(silk::SolverBackend backend, int num_steps) {
  silk::World world;
  world.set_solver_backend(backend);

  // Setup world, add cloth, etc...
  // (same setup for both CPU and GPU)

  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < num_steps; ++i) {
    auto result = world.solver_step();
    if (!result) {
      std::cerr << "Solver failed at step " << i << std::endl;
      break;
    }
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

  std::cout << "Backend: " << (backend == silk::SolverBackend::GPU ? "GPU" : "CPU")
            << ", Steps: " << num_steps
            << ", Time: " << duration.count() << " ms"
            << ", Avg: " << (float)duration.count() / num_steps << " ms/step"
            << std::endl;
}

int main() {
  benchmark_solver(silk::SolverBackend::CPU, 100);
  benchmark_solver(silk::SolverBackend::GPU, 100);
  return 0;
}
```

Expected output:
```
Backend: CPU, Steps: 100, Time: 4523 ms, Avg: 45.23 ms/step
Backend: GPU, Steps: 100, Time: 1876 ms, Avg: 18.76 ms/step
Speedup: 2.4x
```

---

## Example Scenes to Try

### Small Test (Quick validation):
```bash
./demo --config ../misc/example_config/default.json
```

### Medium Test (Good for GPU):
```bash
./demo --config ../misc/example_config/bunny_and_cloth.json
```

### Large Test (Best GPU speedup):
```bash
# You may need to create this or use your own large mesh
./demo --config ../misc/example_config/dress_large.json
```

---

## Summary - TL;DR

**Quickest way to test GPU solver:**

1. **Build**:
   ```bash
   mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release
   cmake --build . --config Release
   ```

2. **Run GUI**:
   ```bash
   cd demo
   ./demo
   ```

3. **In GUI**:
   - Load scene: `File` → `Load Config` → `bunny_and_cloth.json`
   - Check: ☑️ **"Use GPU solver"** (in Solver panel)
   - Click: **Play** ▶️

4. **Watch GPU go brrr** 🚀
   - Open `nvidia-smi` in terminal
   - See GPU utilization spike!

---

## Next Steps

Once you have it running:

- **Profile performance**: Compare CPU vs GPU on different mesh sizes
- **Experiment with settings**: Try different `max_inner_iteration` values
- **Create custom scenes**: Load your own meshes
- **Optimize further**: See GPU_INTEGRATION.md for future enhancements

---

## Need Help?

- **Documentation**: See `GPU_INTEGRATION.md` for detailed technical info
- **Changes**: See `GPU_CHANGES_SUMMARY.md` for what was modified
- **Issues**: https://github.com/iiiian/silk_physics/issues

---

**Enjoy your GPU-accelerated cloth simulation! 🎉**
