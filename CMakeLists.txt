cmake_minimum_required(VERSION 3.24)

project(silk VERSION 0.1.0 LANGUAGES CXX FORTRAN)

set(CXX_STANDARD 17)

option(SILK_BUILD_DEMO "Build the GUI demo application" ON)

add_subdirectory(silk)
if(SILK_BUILD_DEMO)
    add_subdirectory(demo)
endif()

enable_testing()
add_subdirectory(extern/catch2)
add_subdirectory(test)
