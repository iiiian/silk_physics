## Install Dependencies

Follow bellow command to install all dependencies:

```
# ubuntu
sudo apt install libopenblas-dev build-essential git cmake
```

To build GUI demo, you need additional dependencies

```
# ubuntu
apt install xorg-dev libwayland-dev libxkbcommon-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev
```

To build with GPU acceleration, you need cuda toolkit 13+.

## Clone the project

```
git clone https://github.com/iiiian/silk_physics.git
cd silk_physics
git submodule update --init --recursive
```

## Compile

**CPU only**

```
cmake --preset release
cmake --build build/release

**CPU and CUDA**

```
cmake --preset release -DSILK_ENABLE_CUDA=ON
cmake --build build/release
```
```

## Run the Demo

```
./build/release/demo/demo
```
