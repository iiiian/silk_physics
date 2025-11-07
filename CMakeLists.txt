cmake_minimum_required(VERSION 3.24)

project(silk VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(_silk_fetch_dependencies_default ON)
if(WIN32)
    set(_silk_fetch_dependencies_default OFF)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ---------------------------------------------------------------
# Options
# ---------------------------------------------------------------

option(SILK_ENABLE_CUDA "Build with CUDA" ON)
option(SILK_BUILD_DEMO "Build the GUI demo application" ON)
option(SILK_BUILD_TEST "Build tests" ON)
option(SILK_FETCH_DEPENDENCIES "Download dependencies using CMake FetchContent" ${_silk_fetch_dependencies_default})

# ---------------------------------------------------------------

if(SILK_ENABLE_CUDA)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    set(CMAKE_CUDA_ARCHITECTURES all-major)
endif()

if(SILK_FETCH_DEPENDENCIES)
    include(DependenciesFetchContent)
else()
    include(DependenciesFindPackage)
endif()

add_subdirectory(silk)

if(SILK_BUILD_DEMO)
    add_subdirectory(demo)
endif()

if(SILK_BUILD_TEST)
    enable_testing()
    add_subdirectory(test)
endif()
