set(SILK_DIR ${CMAKE_SOURCE_DIR}/silk/src)

# sap kd tree test
add_executable(
    sap_test
    sap_test.cpp
    collision_broadphase_test_utils.cpp
    abc_file_loader.cpp
    ${SILK_DIR}/backend/cpu/collision/bbox.cpp
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
    PRIVATE TBB::tbb
)
target_compile_definitions(
    sap_test
    PRIVATE PHYSICS_SCENE_ROOT="${CMAKE_SOURCE_DIR}/misc/physics_scene"
)

add_executable(
    manager_test
    manager_test.cpp
)
target_include_directories(
    manager_test
    PRIVATE ${SILK_DIR}
)
target_link_libraries(
    manager_test
    PRIVATE Catch2::Catch2WithMain
)

add_executable(
    sap_kd_tree_test
    sap_kd_tree_test.cpp
    collision_broadphase_test_utils.cpp
    abc_file_loader.cpp
    ${SILK_DIR}/backend/cpu/collision/bbox.cpp
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
    PRIVATE TBB::tbb
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
    ${SILK_DIR}/backend/cpu/collision/bbox.cpp
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
    PRIVATE TBB::tbb
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
    ${SILK_DIR}/backend/cpu/collision/bbox.cpp
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
    PRIVATE TBB::tbb
)
target_compile_definitions(
    sap_kd_tree_benchmark
    PRIVATE PHYSICS_SCENE_ROOT="${CMAKE_SOURCE_DIR}/misc/physics_scene"
)

if(SILK_ENABLE_CUDA)
    add_executable(
        elastic_rhs_test
        elastic_rhs_test.cpp
    )
    target_include_directories(
        elastic_rhs_test
        PRIVATE ${SILK_DIR}
    )
    target_link_libraries(
        elastic_rhs_test
        PRIVATE silk::silk
        PRIVATE Eigen3::Eigen
        PRIVATE spdlog::spdlog
        PRIVATE Catch2::Catch2WithMain
        PRIVATE TBB::tbb
        PRIVATE CUDA::cudart
    )
endif()

include(CTest)
include(Catch)

set(SILK_TEST_TARGETS
    manager_test
    sap_test
    sap_kd_tree_test
    sap_benchmark
    sap_kd_tree_benchmark
)

if(SILK_ENABLE_CUDA)
    add_executable(
        jacobi_solver_test
        jacobi_solver_test.cpp
    )
    target_include_directories(
        jacobi_solver_test
        PRIVATE ${SILK_DIR}
    )
    target_link_libraries(
        jacobi_solver_test
        PRIVATE silk::silk
        PRIVATE Eigen3::Eigen
        PRIVATE spdlog::spdlog
        PRIVATE Catch2::Catch2WithMain
        PRIVATE TBB::tbb
        PRIVATE CUDA::cudart
    )

    list(APPEND SILK_TEST_TARGETS elastic_rhs_test jacobi_solver_test)
endif()

catch_discover_tests(${SILK_TEST_TARGETS})
