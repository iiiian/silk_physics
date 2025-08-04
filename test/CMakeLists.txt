set(SILK_DIR ${CMAKE_SOURCE_DIR}/silk/src)

# ccd correctness test
add_executable(
    ccd_test
    ccd_test.cpp
    ccd_test_utils.cpp
    ${SILK_DIR}/ccd.cpp
    ${SILK_DIR}/ccd_poly.cpp
)
target_include_directories(ccd_test PRIVATE ${SILK_DIR})
target_link_libraries(
    ccd_test
    PRIVATE Catch2::Catch2WithMain
    PRIVATE Eigen3::Eigen
)
target_compile_definitions(
    ccd_test
    PRIVATE SAMPLE_QUERY_ROOT="${CMAKE_SOURCE_DIR}/misc/ccd-sample-queries"
)

# ccd reflection test
add_executable(
    ccd_reflection
    ccd_reflection.cpp
    ccd_test_utils.cpp
    ${SILK_DIR}/ccd.cpp
    ${SILK_DIR}/ccd_poly.cpp
)
target_include_directories(ccd_reflection PRIVATE ${SILK_DIR})
target_link_libraries(
    ccd_reflection
    PRIVATE Eigen3::Eigen
    PRIVATE spdlog::spdlog
)
target_compile_definitions(
    ccd_reflection
    PRIVATE SAMPLE_QUERY_ROOT="${CMAKE_SOURCE_DIR}/misc/ccd-sample-queries"
)

# ccd performance test
add_executable(
    ccd_benchmark
    ccd_benchmark.cpp
    ccd_test_utils.cpp
    ${SILK_DIR}/ccd.cpp
    ${SILK_DIR}/ccd_poly.cpp
)
target_include_directories(ccd_benchmark PRIVATE ${SILK_DIR})
target_link_libraries(
    ccd_benchmark
    PRIVATE Catch2::Catch2WithMain
    PRIVATE Eigen3::Eigen
)
target_compile_definitions(
    ccd_benchmark
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
catch_discover_tests(ccd_test ccd_benchmark sap_test sap_kd_tree_test sap_benchmark sap_kd_tree_benchmark ccd_poly)
