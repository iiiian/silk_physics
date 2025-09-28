The easiest way to compile on windows is through Visual Studio and vcpkg. 

## Install Visual Studio

Go to the [official website](https://visualstudio.microsoft.com/) to install visual studio.

During the install, please select **Desktop Development with C++**. This will setup the MSVC compiler tool-chain.

## Install vcpkg

Install vcpkg through their [official website](https://vcpkg.io/en/). 

## Clone the source code

```
git clone https://github.com/iiiian/silk_physics.git
git submodule update --init --recursive
```

## Setup CMake user preset

Copy the example preset located at `misc/cmake_user_preset_examples/windows/CMakeUserPresets.json` to the project root.

The `CMakeUserPresets.json` will looks like

```
{
  "version": 5,
  "configurePresets": [
    {
      "name": "vs_debug",
      "inherits": "debug",
      "displayName": "Windows VS Debug",
      "generator": "Visual Studio 17 2022",
      "architecture": "x64",
      "binaryDir": "${sourceDir}/build/debug",
      "cacheVariables": {
        "CMAKE_TOOLCHAIN_FILE": "$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
      }
    },
    {
      "name": "vs_release",
      "inherits": "release",
      "displayName": "Windows VS Release",
      "generator": "Visual Studio 17 2022",
      "architecture": "x64",
      "binaryDir": "${sourceDir}/build/release",
      "cacheVariables": {
        "CMAKE_TOOLCHAIN_FILE": "$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
      }
    }
  ]
}

```

Modify the "CMAKE_TOOLCHAIN_FILE" field to point to your vcpkg install. For example, if you clone the vcpkg repo at `C:/Users/Ian/vcpkg`, the preset file will look like 

```
"cacheVariables": {
	"CMAKE_TOOLCHAIN_FILE": "C:/Users/Ian/vcpkg/scripts/buildsystems/vcpkg.cmake"
}
```

After that, open the **Developer Command Prompt** through the start menu or within Visual Studio if you already open the project. In the terminal, type

```
cmake --preset vs_debug
```

This should trigger vcpkg to install all the dependencies and will take a while. After it finishes, build the program using

```
cmake --build build/debug
```
For running the demo, build the program using
```
cmake --build --preset vs_release
```
