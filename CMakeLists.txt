cmake_minimum_required(VERSION 3.24)

project(silk VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------
# Options
# ---------------------------------------------------------------

option(SILK_BUILD_DEMO "Build the GUI demo application" ON)
option(SILK_BUILD_TEST "Build tests" ON)

# ---------------------------------------------------------------
# System dependencies
# ---------------------------------------------------------------

find_package(OpenMP REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

# ---------------------------------------------------------------
# Internal dependencies
# ---------------------------------------------------------------

# Defined via fetch content or add_subdirectory.

include(FetchContent)

FetchContent_Declare(
    Eigen3
    # prefer github mirror as the connection is more stable
    GIT_REPOSITORY https://github.com/eigen-mirror/eigen.git
    GIT_TAG
        3147391d946bb4b6c68edd901f2add6ac1f31f8c # release 3.4.0
    FIND_PACKAGE_ARGS 3.4
)

FetchContent_Declare(
    Libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG
        40e7900ccbd767f1f360e0eb10f0f1a6432e0993 # release 2.6.0
    FIND_PACKAGE_ARGS 2.6
)

FetchContent_Declare(
    SuiteSparse
    GIT_REPOSITORY https://github.com/DrTimothyAldenDavis/SuiteSparse.git
    GIT_TAG
        b35a1f9318f4bd42085f4b5ea56f29c89d342d4d # release 7.11.0
    FIND_PACKAGE_ARGS 7.11
)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG
        644821ce28cb25d7992a4d0375b1d83214392592 # release 3.9.1
    FIND_PACKAGE_ARGS 3.9
)

FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG
        6fa36017cfd5731d617e1a934f0e5ea9c4445b13 # release 1.15.3
    FIND_PACKAGE_ARGS 1.15
)


set(SUITESPARSE_ENABLE_PROJECTS
    "suitesparse_config;amd;camd;colamd;ccolamd;cholmod;"
)

FetchContent_MakeAvailable(
    Eigen3
    Libigl
    SuiteSparse
    spdlog
)

set(TIGHT_INCLUSION_WITH_DOUBLE_PRECISION OFF)
add_subdirectory(extern/tight-inclusion)

add_subdirectory(silk)

if(SILK_BUILD_DEMO)
    add_subdirectory(extern/polyscope)
    add_subdirectory(extern/portable-file-dialogs)
    add_subdirectory(demo)
endif()

if(SILK_BUILD_TEST)
    FetchContent_MakeAvailable(Catch2)
    find_package(Alembic REQUIRED)
    enable_testing()
    add_subdirectory(test)
endif()
