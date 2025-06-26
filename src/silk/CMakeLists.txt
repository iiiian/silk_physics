cmake_minimum_required(VERSION 3.28)

project(silk CXX Fortran)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenMP REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

add_subdirectory(extern/spdlog)
add_subdirectory(extern/libigl)
add_subdirectory(extern/arpack-ng)

file(GLOB SOURCES "src/*.cpp")
add_library(silk ${SOURCES})

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(
        silk
        PRIVATE
            -Wall
            -Wextra
            -fdiagnostics-color
            -Wno-unused-parameter
            # turn this warning off else libigl will vomit warnings
            -Wno-sign-compare
    )
    target_compile_definitions(
        silk
        PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG
    )
endif()

target_link_libraries(silk PRIVATE spdlog::spdlog)
target_link_libraries(silk PRIVATE igl::core)
target_link_libraries(silk PRIVATE polyscope)
target_link_libraries(silk PRIVATE portable_file_dialogs)
target_link_libraries(silk PRIVATE arpack)
# target_link_libraries(silk PRIVATE embree)
target_link_libraries(silk PRIVATE OpenMP::OpenMP_CXX)

enable_testing()
add_subdirectory(extern/catch2)
add_subdirectory(test)
