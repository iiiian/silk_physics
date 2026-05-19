set(SILK_DIR ${CMAKE_SOURCE_DIR}/silk/src)

# sap kd tree test
add_executable(
    sap_test
    sap_test.cpp
    collision_broadphase_test_utils.cpp
    abc_file_loader.cpp
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
    sap_kd_tree_test
    sap_kd_tree_test.cpp
    collision_broadphase_test_utils.cpp
    abc_file_loader.cpp
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
        sym_inverse_benchmark
        sym_inverse_benchmark.cu
    )
    target_include_directories(
        sym_inverse_benchmark
        PRIVATE ${SILK_DIR}
    )
    target_compile_options(
        sym_inverse_benchmark
        PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda --expt-relaxed-constexpr>
    )
    target_link_libraries(
        sym_inverse_benchmark
        PRIVATE CUDA::cudart CCCL::CCCL Eigen3::Eigen
    )
endif()

include(CTest)
include(Catch)

set(SILK_TEST_TARGETS
    sap_test
    sap_kd_tree_test
    sap_benchmark
    sap_kd_tree_benchmark
)

catch_discover_tests(${SILK_TEST_TARGETS})
