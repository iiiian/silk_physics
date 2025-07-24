cmake_minimum_required(VERSION 3.24)

project(silk VERSION 0.1.0 LANGUAGES CXX Fortran)

set(CMAKE_CXX_STANDARD 17)

option(SILK_BUILD_DEMO "Build the GUI demo application" ON)
option(SILK_BUILD_TEST "Build tests" ON)

find_package(OpenMP REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Eigen3 3.4)
if(NOT TARGET Eigen3::Eigen)
    message(STATUS "Using vendored Eigen")
    set(EIGEN_BUILD_CMAKE_PACKAGE ON)
    add_subdirectory(extern/eigen)
endif()
add_subdirectory(extern/libigl)
add_subdirectory(extern/arpack-ng)

add_subdirectory(silk)

if(SILK_BUILD_DEMO OR SILK_BUILD_TEST)
    add_subdirectory(extern/spdlog)
endif()

# if(SILK_BUILD_DEMO)
#     add_subdirectory(extern/polyscope)
#     add_subdirectory(extern/portable-file-dialogs)
#
#     add_subdirectory(demo)
# endif()

if(SILK_BUILD_TEST)
    find_package(Alembic CONFIG REQUIRED)
    add_subdirectory(extern/catch2)

    enable_testing()
    add_subdirectory(test)
endif()
