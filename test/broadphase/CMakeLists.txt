set(SILK_BROADPHASE_DATA_REVISION
    c0f5a2e54fc57a28b54f726eaf3030d7822ae77f
)
set(
    SILK_BROADPHASE_DATA_ROOT
    "${CMAKE_SOURCE_DIR}/build/broadphase-data/${SILK_BROADPHASE_DATA_REVISION}"
)
include(download_data.cmake)
silk_download_broadphase_data(
    "${SILK_BROADPHASE_DATA_ROOT}" "${SILK_BROADPHASE_DATA_REVISION}"
)

add_executable(
    broadphase_benchmark
    main.cpp
    broadphase_common.cpp
    silk_kdtree_adapter.cpp
    ${SILK_DIR}/common/perf_region_control.cpp
    ${SILK_DIR}/backend/cpu/collision/bbox_soa.cpp
    scalable_sap_adapter.cpp
    embree_adapter.cpp
)
target_include_directories(
    broadphase_benchmark PRIVATE ${SILK_DIR} ${CMAKE_SOURCE_DIR}/extern/pdqsort
)
target_link_libraries(
    broadphase_benchmark
    PRIVATE
        Eigen3::Eigen
        TBB::tbb
        hwy::hwy
        ${SILK_IGL_CORE}
        argparse::argparse
        nlohmann_json::nlohmann_json
        spdlog::spdlog
        scalable_ccd::scalable_ccd
        embree
)
target_compile_definitions(
    broadphase_benchmark
    PRIVATE SILK_BROADPHASE_DATA_ROOT="${SILK_BROADPHASE_DATA_ROOT}"
)

if(SILK_ENABLE_CUDA)
    target_sources(
        broadphase_benchmark
        PRIVATE
            silk_oibvh_adapter.cu
            scalable_stq_adapter.cu
            cubql_adapter.cu
    )
    target_link_libraries(
        broadphase_benchmark
        PRIVATE CUDA::cudart CCCL::CCCL cuBQL_cuda_float3_static
    )
    target_compile_definitions(
        broadphase_benchmark PRIVATE SILK_BROADPHASE_HAS_CUDA=1 EIGEN_NO_CUDA
    )
    target_compile_options(
        broadphase_benchmark
        PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda --expt-relaxed-constexpr>
    )
endif()

add_test(NAME broadphase_correctness COMMAND broadphase_benchmark --verify)
set_tests_properties(broadphase_correctness PROPERTIES LABELS "broadphase;long")
