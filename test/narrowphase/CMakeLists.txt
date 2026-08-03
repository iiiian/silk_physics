set(SILK_NARROWPHASE_DATA_REVISION
    c0f5a2e54fc57a28b54f726eaf3030d7822ae77f
)
set(
    SILK_NARROWPHASE_DATA_ROOT
    "${CMAKE_SOURCE_DIR}/build/broadphase-data/${SILK_NARROWPHASE_DATA_REVISION}"
)
include(../broadphase/download_data.cmake)
silk_download_broadphase_data(
    "${SILK_NARROWPHASE_DATA_ROOT}" "${SILK_NARROWPHASE_DATA_REVISION}"
)

add_executable(
    narrowphase_benchmark
    main.cpp
    narrowphase_common.cpp
    cpu_adapters.cpp
    ../broadphase/broadphase_common.cpp
    ../broadphase/embree_adapter.cpp
    ${SILK_DIR}/common/perf_region_control.cpp
    ${SILK_DIR}/backend/cpu/collision/ccd.cpp
    ${SILK_DIR}/backend/cpu/collision/interval.cpp
    ${SILK_DIR}/backend/cpu/collision/interval_root_finder.cpp
)
target_include_directories(narrowphase_benchmark PRIVATE ${SILK_DIR})
target_link_libraries(
    narrowphase_benchmark
    PRIVATE
        Eigen3::Eigen
        TBB::tbb
        embree
        ${SILK_IGL_CORE}
        argparse::argparse
        nlohmann_json::nlohmann_json
        spdlog::spdlog
        tight_inclusion::tight_inclusion
)
target_compile_definitions(
    narrowphase_benchmark
    PRIVATE SILK_NARROWPHASE_DATA_ROOT="${SILK_NARROWPHASE_DATA_ROOT}"
)

if(SILK_ENABLE_CUDA)
    target_sources(narrowphase_benchmark PRIVATE scalable_gpu_adapter.cu)
    target_link_libraries(
        narrowphase_benchmark
        PRIVATE CUDA::cudart CCCL::CCCL CCDGPU
    )
    target_compile_definitions(
        narrowphase_benchmark PRIVATE SILK_NARROWPHASE_HAS_CUDA=1 EIGEN_NO_CUDA
    )
    target_compile_options(
        narrowphase_benchmark
        PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda --expt-relaxed-constexpr>
    )
endif()

add_test(NAME narrowphase_correctness COMMAND narrowphase_benchmark --verify)
set_tests_properties(narrowphase_correctness PROPERTIES LABELS "narrowphase;long")
