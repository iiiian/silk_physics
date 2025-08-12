cmake_minimum_required(VERSION 3.24)

project(silk VERSION 0.1.0 LANGUAGES CXX Fortran)

set(CMAKE_CXX_STANDARD 17)

option(SILK_BUILD_DEMO "Build the GUI demo application" ON)
option(SILK_BUILD_TEST "Build tests" ON)

include(FetchContent)

FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
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
    arpackng
    GIT_REPOSITORY https://github.com/opencollab/arpack-ng.git
    GIT_TAG
        40329031ae8deb7c1e26baf8353fa384fc37c251 # release 3.9.1
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
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG
        6fa36017cfd5731d617e1a934f0e5ea9c4445b13 # release 1.15.3
    FIND_PACKAGE_ARGS 1.15
)

FetchContent_Declare(
    polyscope
    GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
    # pull from master branch for hidpi patch
    GIT_TAG aba50436d7d138ab6e1e9b6e498a3ae736366ae8
    # To add imgui_stdlib.cpp to the source list, patch polyscope's homemade CMakeLists.txt for DearIMGui.
    PATCH_COMMAND
        ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/misc/hacks/polyscope/deps/imgui/CMakeLists.txt
        <SOURCE_DIR>/deps/imgui/CMakeLists.txt
)

FetchContent_Declare(
    portable-file-dialogs
    GIT_REPOSITORY https://github.com/samhocevar/portable-file-dialogs.git
    GIT_TAG c12ea8c9a727f5320a2b4570aee863bbede2a204
)

FetchContent_Declare(
    tight-inclusion
    GIT_REPOSITORY
        https://github.com/Continuous-Collision-Detection/Tight-Inclusion.git
    GIT_TAG
        7377d78162f9fdb5fe44b54b6f9c01bc6ce0885a # release 1.0.6
    # the original CMakeLists.txt does not provides proper relative include dir and lacks export target.
    # patch the CMakeLists.txt file and add a simple tight_inclusionConfig.cmake.in to fix above.
    PATCH_COMMAND
        ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/misc/hacks/tight_inclusion/CMakeLists.txt
        <SOURCE_DIR>/CMakeLists.txt && ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/misc/hacks/tight_inclusion/tight_inclusionConfig.cmake.in
        <SOURCE_DIR>/cmake/tight_inclusionConfig.cmake.in
)

# base dependencies
find_package(OpenMP REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
FetchContent_MakeAvailable(
    Eigen3
    Libigl
    arpackng
    Catch2
    tight-inclusion
)

add_subdirectory(silk)

if(SILK_BUILD_DEMO)
    FetchContent_MakeAvailable(polyscope portable-file-dialogs spdlog)
    add_subdirectory(demo)
endif()

if(SILK_BUILD_TEST)
    find_package(Alembic REQUIRED)
    FetchContent_MakeAvailable(spdlog)
    enable_testing()
    add_subdirectory(test)
endif()
