cmake_minimum_required(VERSION 3.25)

project(silk VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(_silk_fetch_dependencies_default ON)
if(WIN32)
    set(_silk_fetch_dependencies_default OFF)
endif()

set(_silk_graph_partition_backend_default "kaminpar")
if(WIN32)
    set(_silk_graph_partition_backend_default "metis")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ---------------------------------------------------------------
# Options
# ---------------------------------------------------------------

option(SILK_ENABLE_CUDA "Build with CUDA" OFF)
option(SILK_ENABLE_TIMING "Enable hierarchical timing instrumentation" OFF)
option(SILK_BUILD_DEMO "Build the GUI demo application" ON)
option(SILK_BUILD_TEST "Build tests" ON)
option(SILK_BROADPHASE_BENCHMARKS "Build broadphase verification and benchmarks" OFF)
option(SILK_NARROWPHASE_BENCHMARKS "Build narrowphase verification and benchmarks" OFF)
option(SILK_FETCH_DEPENDENCIES "Download dependencies using CMake FetchContent" ${_silk_fetch_dependencies_default})
set(
    SILK_GRAPH_PARTITION_BACKEND
    "${_silk_graph_partition_backend_default}"
    CACHE STRING
    "Graph partition backend: kaminpar or metis"
)
set_property(CACHE SILK_GRAPH_PARTITION_BACKEND PROPERTY STRINGS kaminpar metis)

# ---------------------------------------------------------------

if(SILK_ENABLE_CUDA)
    # If CMAKE_CUDA_ARCHITECTURES was not specified, set it to native.
    if(DEFINED CMAKE_CUDA_ARCHITECTURES)
      message(STATUS "CMAKE_CUDA_ARCHITECTURES was specified, skipping auto-detection")
    else()
      message(STATUS "CMAKE_CUDA_ARCHITECTURES was not specified, set it to native")
      set(CMAKE_CUDA_ARCHITECTURES "native")
    endif()
    message(STATUS "Targeting CUDA_ARCHITECTURES \"${CMAKE_CUDA_ARCHITECTURES}\"")

    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 20)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
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

if(SILK_BUILD_TEST OR SILK_BROADPHASE_BENCHMARKS OR SILK_NARROWPHASE_BENCHMARKS)
    enable_testing()
    add_subdirectory(test)
endif()
