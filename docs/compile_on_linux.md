## Install Dependencies

Follow bellow command to install all dependencies:

```
# ubuntu
sudo apt install libopenblas-dev build-essential git cmake
```

to build GUI demo, you need additional dependencies

```
# ubuntu
apt install xorg-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev
```

## Clone the project

```
git clone https://github.com/iiiian/silk_physics.git
cd silk_physics
git submodule update --init --recursive
```

## Compile

```
cmake --preset release -DSILK_BUILD_TEST=OFF
cmake --build build/release
```

## Run the Demo

```
./build/release/demo/demo
```

## Build tests

You will need to install [Alembic](https://github.com/alembic/alembic?tab=readme-ov-file)
