set(SILK_DIR ${CMAKE_SOURCE_DIR}/silk/src)

# ccd reflection test
add_executable(
    ccd_reflection
    ccd_reflection.cpp
    ccd_test_utils.cpp
    ${SILK_DIR}/collision_narrowphase.cpp
)
target_include_directories(
    ccd_reflection
    PRIVATE ${SILK_DIR} ${CMAKE_SOURCE_DIR}/extern/pdqsort
)
target_link_libraries(
    ccd_reflection
    PRIVATE
        Eigen3::Eigen
        spdlog::spdlog
        tight_inclusion::tight_inclusion
        OpenMP::OpenMP_CXX
)
target_compile_definitions(
    ccd_reflection
    PRIVATE SAMPLE_QUERY_ROOT="${CMAKE_SOURCE_DIR}/misc/ccd-sample-queries"
)

# sap kd tree test
add_executable(
    sap_test
    sap_test.cpp
    collision_broadphase_test_utils.cpp
    abc_file_loader.cpp
    ${SILK_DIR}/bbox.cpp
)
target_include_directories(
    sap_test
    PRIVATE ${SILK_DIR} ${CMAKE_SOURCE_DIR}/extern/pdqsort
)
target_link_libraries(
    sap_test
    PRIVATE Eigen3::Eigen
    PRIVATE spdlog::spdlog
    PRIVATE Alembic::Alembic
    PRIVATE Catch2::Catch2WithMain
    PRIVATE OpenMP::OpenMP_CXX
)
target_compile_definitions(
    sap_test
    PRIVATE PHYSICS_SCENE_ROOT="${CMAKE_SOURCE_DIR}/misc/physics_scene"
)

add_executable(
    sap_kd_tree_test
    sap_kd_tree_test.cpp
    collision_broadphase_test_utils.cpp
    abc_file_loader.cpp
    ${SILK_DIR}/bbox.cpp
)
target_include_directories(
    sap_kd_tree_test
    PRIVATE ${SILK_DIR} ${CMAKE_SOURCE_DIR}/extern/pdqsort
)
target_link_libraries(
    sap_kd_tree_test
    PRIVATE Eigen3::Eigen
    PRIVATE spdlog::spdlog
    PRIVATE Alembic::Alembic
    PRIVATE Catch2::Catch2WithMain
    PRIVATE OpenMP::OpenMP_CXX
)
target_compile_definitions(
    sap_kd_tree_test
    PRIVATE PHYSICS_SCENE_ROOT="${CMAKE_SOURCE_DIR}/misc/physics_scene"
)

add_executable(
    sap_benchmark
    sap_benchmark.cpp
    collision_broadphase_test_utils.cpp
    abc_file_loader.cpp
    ${SILK_DIR}/bbox.cpp
)
target_include_directories(
    sap_benchmark
    PRIVATE ${SILK_DIR} ${CMAKE_SOURCE_DIR}/extern/pdqsort
)
target_link_libraries(
    sap_benchmark
    PRIVATE Eigen3::Eigen
    PRIVATE spdlog::spdlog
    PRIVATE Alembic::Alembic
    PRIVATE Catch2::Catch2WithMain
    PRIVATE OpenMP::OpenMP_CXX
)
target_compile_definitions(
    sap_benchmark
    PRIVATE PHYSICS_SCENE_ROOT="${CMAKE_SOURCE_DIR}/misc/physics_scene"
)

add_executable(
    sap_kd_tree_benchmark
    sap_kd_tree_benchmark.cpp
    collision_broadphase_test_utils.cpp
    abc_file_loader.cpp
    ${SILK_DIR}/bbox.cpp
)
target_include_directories(
    sap_kd_tree_benchmark
    PRIVATE ${SILK_DIR} ${CMAKE_SOURCE_DIR}/extern/pdqsort
)
target_link_libraries(
    sap_kd_tree_benchmark
    PRIVATE Eigen3::Eigen
    PRIVATE spdlog::spdlog
    PRIVATE Alembic::Alembic
    PRIVATE Catch2::Catch2WithMain
    PRIVATE OpenMP::OpenMP_CXX
)
target_compile_definitions(
    sap_kd_tree_benchmark
    PRIVATE PHYSICS_SCENE_ROOT="${CMAKE_SOURCE_DIR}/misc/physics_scene"
)

include(CTest)
include(Catch)
catch_discover_tests(sap_test sap_kd_tree_test sap_benchmark sap_kd_tree_benchmark ccd_poly)
