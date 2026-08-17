set(SILK_DIR ${CMAKE_SOURCE_DIR}/silk/src)

if(SILK_BUILD_TEST AND SILK_ENABLE_CUDA)
    add_executable(graph_partition_test graph_partition_test.cpp)
    target_include_directories(graph_partition_test PRIVATE ${SILK_DIR})
    target_link_libraries(
        graph_partition_test
        PRIVATE silk CCCL::CCCL Catch2::Catch2WithMain
    )
    if(SILK_GRAPH_PARTITION_BACKEND STREQUAL "kaminpar")
        target_link_libraries(graph_partition_test PRIVATE KaMinPar::KaMinPar)
    elseif(SILK_GRAPH_PARTITION_BACKEND STREQUAL "metis")
        target_link_libraries(graph_partition_test PRIVATE metis::metis)
    endif()
    target_compile_definitions(
        graph_partition_test
        PRIVATE
            $<$<STREQUAL:${SILK_GRAPH_PARTITION_BACKEND},kaminpar>:SILK_GRAPH_PARTITION_BACKEND_KAMINPAR=1>
            $<$<STREQUAL:${SILK_GRAPH_PARTITION_BACKEND},metis>:SILK_GRAPH_PARTITION_BACKEND_METIS=1>
    )

    add_executable(sym_inverse_benchmark sym_inverse_benchmark.cu)
    target_include_directories(sym_inverse_benchmark PRIVATE ${SILK_DIR})
    target_compile_options(
        sym_inverse_benchmark
        PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda --expt-relaxed-constexpr>
    )
    target_compile_definitions(sym_inverse_benchmark PRIVATE EIGEN_NO_CUDA)
    target_link_libraries(
        sym_inverse_benchmark
        PRIVATE CUDA::cudart CCCL::CCCL Eigen3::Eigen
    )

    add_executable(friction_contact_test friction_contact_test.cu)
    target_include_directories(friction_contact_test PRIVATE ${SILK_DIR})
    target_compile_options(
        friction_contact_test
        PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda --expt-relaxed-constexpr>
    )
    target_compile_definitions(friction_contact_test PRIVATE EIGEN_NO_CUDA)
    target_link_libraries(
        friction_contact_test
        PRIVATE silk CUDA::cudart CCCL::CCCL Catch2::Catch2WithMain
    )

    include(Catch)
    catch_discover_tests(graph_partition_test)
    catch_discover_tests(friction_contact_test)
endif()

if(SILK_BROADPHASE_BENCHMARKS)
    add_subdirectory(broadphase)
endif()

if(SILK_NARROWPHASE_BENCHMARKS)
    add_subdirectory(narrowphase)
endif()
