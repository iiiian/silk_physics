cmake_minimum_required(VERSION 3.24)

project(silk VERSION 0.1.0 LANGUAGES CXX Fortran)

set(CMAKE_CXX_STANDARD 17)

option(SILK_BUILD_DEMO "Build the GUI demo application" OFF)

find_package(OpenMP REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
add_subdirectory(extern/libigl)
add_subdirectory(extern/arpack-ng)

if(SILK_BUILD_DEMO)
    add_subdirectory(extern/spdlog)
    add_subdirectory(extern/polyscope)
    add_subdirectory(extern/portable-file-dialogs)
endif()

add_subdirectory(silk)

if(SILK_BUILD_DEMO)
    add_subdirectory(demo)
endif()

enable_testing()
add_subdirectory(extern/catch2)
add_subdirectory(test)
