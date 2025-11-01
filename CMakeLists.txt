cmake_minimum_required(VERSION 3.24)

project(silk VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -------------------------------------------------
# CUDA Support (Optional)
# -------------------------------------------------
option(SILK_BUILD_GPU "Build GPU-accelerated solver with CUDA" OFF)

if(SILK_BUILD_GPU)
    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)

    if(NOT DEFINED CMAKE_CUDA_STANDARD)
        set(CMAKE_CUDA_STANDARD 17)
        set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    endif()

    message(STATUS "CUDA support enabled")
    message(STATUS "  CUDA Version: ${CUDAToolkit_VERSION}")
    message(STATUS "  CUDA Toolkit Root: ${CUDAToolkit_TARGET_DIR}")
endif()

set(_silk_fetch_dependencies_default ON)
if(WIN32)
    set(_silk_fetch_dependencies_default OFF)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ---------------------------------------------------------------
# Options
# ---------------------------------------------------------------

option(SILK_BUILD_DEMO "Build the GUI demo application" ON)
option(SILK_BUILD_TEST "Build tests" ON)
option(SILK_FETCH_DEPENDENCIES "Download dependencies using CMake FetchContent" ${_silk_fetch_dependencies_default})

# ---------------------------------------------------------------

if(SILK_FETCH_DEPENDENCIES)
    include(DependenciesFetchContent)
else()
    include(DependenciesFindPackage)
endif()

add_subdirectory(silk)

if(SILK_BUILD_GPU)
    add_subdirectory(silk/src/solver/gpu)
endif()

if(SILK_BUILD_DEMO)
    add_subdirectory(demo)
endif()

if(SILK_BUILD_TEST)
    enable_testing()
    add_subdirectory(test)
endif()
