![UI](assets/ui.png)

WIP C++17 physics engine with

- A working cloth simulation with an efficient collision pipeline
- A (still incomplete) GUI demo

## Quick start

### Prerequisites

- C++17 compiler and OpenMP
  - GCC >= 9 (Linux/MinGW)
  - Clang/AppleClang >= 10 (AppleClang >= 12 on macOS)
  - Windows: one of
    - MSVC (Visual Studio 2022) 17.2+ / MSVC 19.32+ with `/openmp:llvm`
    - or clang-cl >= 12 with LLVM OpenMP runtime
- CMake >= 3.24 and a build tool (e.g., Ninja or Make)
- Git
- BLAS and LAPACK (OpenBLAS recommended for simplicity)

### Installing BLAS and LAPACK

#### Linux

```bash
# ubuntu
sudo apt install libopenblas-dev
# fedora
sudo dnf install openblas-devel
```

#### MacOS

```bash
brew install openblas
```

#### Windows

- Recommended: clang-cl + LLVM OpenMP
  - Install LLVM (adds clang-cl and libomp).
  - Configure with: `cmake -S . -B build -G Ninja -DCMAKE_C_COMPILER=clang-cl -DCMAKE_CXX_COMPILER=clang-cl`
- MSVC backend (OpenMP 3.1 features, incl. tasks):
  - Requires Visual Studio 2022 17.2+.
  - Project enables `/openmp:llvm` automatically; no extra flags needed.

### Build and run the demo

First clone the repo and its submodules.

```bash
git clone https://github.com/iiiian/silk_physics.git
git submodule update --init --recursive
```

Configure and build.

```bash
cd silk_physics
cmake --preset release
cmake --build build/release --target demo
```

Run the demo.

```bash
./build/release/demo/demo ./model/dense_sheet.obj
```
