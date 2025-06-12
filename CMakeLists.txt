cmake_minimum_required(VERSION 3.28)

project(mesh_deform CXX Fortran)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_SCAN_FOR_MODULES 0)

find_package(OpenMP REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

add_subdirectory(extern/spdlog)
add_subdirectory(extern/libigl)
add_subdirectory(extern/polyscope)
add_subdirectory(extern/portable-file-dialogs)
add_subdirectory(extern/arpack-ng)

add_executable(
    main
    "src/main.cpp"
    "src/solver.cpp"
    "src/demo.cpp"
    "src/kd_tree.cpp"
    # "src/collision_detector.cpp"
    "src/ccd.cpp"
    "src/ccd_poly.cpp"
    "src/collision_helper.cpp"
    "src/vectorized_jacobian.cpp"
    "src/widgets/selector_widget.cpp"
    "src/widgets/model_loader_widget.cpp"
    "src/widgets/cloth_simulator_widget.cpp"
    "src/widgets/help_bar_widget.cpp"
    "src/widgets/mesh_statistic_widget.cpp"
    "extern/polyscope/deps/imgui/imgui/misc/cpp/imgui_stdlib.cpp"
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(
        main
        PRIVATE
            -Wall
            -Wextra
            -fdiagnostics-color
            -Wno-unused-parameter
            # turn this warning off else libigl will vomit warnings
            -Wno-sign-compare
    )
    target_compile_definitions(
        main
        PRIVATE SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG
    )
endif()

target_link_libraries(main PRIVATE spdlog::spdlog)
target_link_libraries(main PRIVATE igl::core)
target_link_libraries(main PRIVATE polyscope)
target_link_libraries(main PRIVATE portable_file_dialogs)
target_link_libraries(main PRIVATE arpack)
# target_link_libraries(main PRIVATE embree)
target_link_libraries(main PRIVATE OpenMP::OpenMP_CXX)

enable_testing()
add_subdirectory(extern/catch2)
add_subdirectory(test)
