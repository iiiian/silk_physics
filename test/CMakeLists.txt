set(SILK_SRC_DIR ${CMAKE_SOURCE_DIR}/silk/src)

# ccd correctness test
add_executable(
    ccd_correctness_test
    ccd_correctness_test.cpp
    ccd_test_helper.cpp
    ${SILK_SRC_DIR}/ccd.cpp
    ${SILK_SRC_DIR}/ccd_poly.cpp
)
target_include_directories(ccd_correctness_test PRIVATE ${SILK_SRC_DIR})
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
    ${SILK_SRC_DIR}/ccd.cpp
    ${SILK_SRC_DIR}/ccd_poly.cpp
)
target_include_directories(ccd_performance_test PRIVATE ${SILK_SRC_DIR})
target_link_libraries(
    ccd_performance_test
    PRIVATE Catch2::Catch2WithMain
    PRIVATE Eigen3::Eigen
)
target_compile_definitions(
    ccd_performance_test
    PRIVATE SAMPLE_QUERY_ROOT="${CMAKE_SOURCE_DIR}/misc/ccd-sample-queries"
)

# resource manager test
add_executable(
    resource_manager_test
    resource_manager_test.cpp
    ${SILK_SRC_DIR}/resource_manager.cpp
)
target_include_directories(resource_manager_test PRIVATE ${SILK_SRC_DIR})
target_link_libraries(resource_manager_test PRIVATE Catch2::Catch2WithMain)

# sap kd tree test
add_executable(
    collision_broadphase_test
    collision_broadphase_test.cpp
    abc_file_loader.cpp
    ${SILK_SRC_DIR}/bbox.cpp
)
target_include_directories(collision_broadphase_test PRIVATE ${SILK_SRC_DIR})
target_link_libraries(
    collision_broadphase_test
    PRIVATE Eigen3::Eigen
    PRIVATE spdlog::spdlog
    PRIVATE Alembic::Alembic
    PRIVATE Catch2::Catch2WithMain
)
target_compile_definitions(
    collision_broadphase_test
    PRIVATE PHYSICS_SCENE_ROOT="${CMAKE_SOURCE_DIR}/misc/physics_scene"
)

include(CTest)
include(Catch)
catch_discover_tests(ccd_correctness_test ccd_performance_test resource_manager_test collision_broadphase_test)
