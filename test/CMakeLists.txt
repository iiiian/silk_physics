set(SILK_DIR ${CMAKE_SOURCE_DIR}/silk/src)

# ccd correctness test
add_executable(
    ccd_correctness_test
    ccd_correctness_test.cpp
    ccd_test_helper.cpp
    ${SILK_DIR}/ccd.cpp
    ${SILK_DIR}/ccd_poly.cpp
)
target_include_directories(ccd_correctness_test PRIVATE ${SILK_DIR})
target_link_libraries(ccd_correctness_test PRIVATE Catch2::Catch2WithMain)
target_link_libraries(ccd_correctness_test PRIVATE Eigen3::Eigen)
target_compile_definitions(
    ccd_correctness_test
    PRIVATE SAMPLE_QUERY_ROOT="${CMAKE_SOURCE_DIR}/misc/ccd-sample-queries"
)

# ccd performance test
add_executable(
    ccd_performance_test
    ccd_performance_test.cpp
    ccd_test_helper.cpp
    ${SILK_DIR}/ccd.cpp
    ${SILK_DIR}/ccd_poly.cpp
)
target_include_directories(ccd_performance_test PRIVATE ${SILK_DIR})
target_link_libraries(
    ccd_performance_test
    PRIVATE Catch2::Catch2WithMain
    PRIVATE Eigen3::Eigen
)
target_compile_definitions(
    ccd_performance_test
    PRIVATE SAMPLE_QUERY_ROOT="${CMAKE_SOURCE_DIR}/misc/ccd-sample-queries"
)

# sap kd tree test
add_executable(
    sap_test
    sap_test.cpp
    collision_broadphase_test_helper.cpp
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
    collision_broadphase_test_helper.cpp
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
    sap_performance_test
    sap_performance_test.cpp
    collision_broadphase_test_helper.cpp
    abc_file_loader.cpp
    ${SILK_DIR}/bbox.cpp
)
target_include_directories(
    sap_performance_test
    PRIVATE ${SILK_DIR} ${CMAKE_SOURCE_DIR}/extern/pdqsort
)
target_link_libraries(
    sap_performance_test
    PRIVATE Eigen3::Eigen
    PRIVATE spdlog::spdlog
    PRIVATE Alembic::Alembic
    PRIVATE Catch2::Catch2WithMain
    PRIVATE OpenMP::OpenMP_CXX
)
target_compile_definitions(
    sap_performance_test
    PRIVATE PHYSICS_SCENE_ROOT="${CMAKE_SOURCE_DIR}/misc/physics_scene"
)

add_executable(
    sap_kd_tree_performance_test
    sap_kd_tree_performance_test.cpp
    collision_broadphase_test_helper.cpp
    abc_file_loader.cpp
    ${SILK_DIR}/bbox.cpp
)
target_include_directories(
    sap_kd_tree_performance_test
    PRIVATE ${SILK_DIR} ${CMAKE_SOURCE_DIR}/extern/pdqsort
)
target_link_libraries(
    sap_kd_tree_performance_test
    PRIVATE Eigen3::Eigen
    PRIVATE spdlog::spdlog
    PRIVATE Alembic::Alembic
    PRIVATE Catch2::Catch2WithMain
    PRIVATE OpenMP::OpenMP_CXX
)
target_compile_definitions(
    sap_kd_tree_performance_test
    PRIVATE PHYSICS_SCENE_ROOT="${CMAKE_SOURCE_DIR}/misc/physics_scene"
)

add_executable(
    ccd_poly
    ccd_poly_test.cpp
    ${SILK_DIR}/ccd_poly.cpp
)
target_include_directories(ccd_poly PRIVATE ${SILK_DIR})
target_link_libraries(ccd_poly PRIVATE Catch2::Catch2WithMain Eigen3::Eigen)

include(CTest)
include(Catch)
catch_discover_tests(ccd_correctness_test ccd_performance_test sap_test sap_kd_tree_test sap_performance_test sap_kd_tree_performance_test ccd_poly)