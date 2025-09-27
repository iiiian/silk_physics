Currently, vcpkg fails to compile some pacakage on MacOS. Hence you need to install all dependencies maually using [homebrew](https://brew.sh/).

First install C++ tooling.

```
brew install cmake make ninja git
```

Then install all dependencies.

```
brew install openblas eigen suite-sparse spdlog tbb catch2 alembic
```

Now with all the dependencies in place, we can clone the repo and compile.

```
git clone https://github.com/iiiian/silk_physics.git
cd silk_physics
git submodule update --init --recursive
```

```
cmake --preset release
cmake --build ./build/release
```


