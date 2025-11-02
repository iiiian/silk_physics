# -------------------------------------------------
# GPU Solver Library (CUDA)
# -------------------------------------------------

# GPU solver sources
set(GPU_SOLVER_SOURCES
    cloth_solver_kernels.cu
    gpu_cloth_solver_context.cu
    cloth_solver_utils.cu
    pipeline.cu
)

# Create GPU solver library
add_library(silk_gpu_solver ${GPU_SOLVER_SOURCES})
add_library(silk::gpu_solver ALIAS silk_gpu_solver)

# Set CUDA properties
set_target_properties(silk_gpu_solver PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    POSITION_INDEPENDENT_CODE ON
)

# Add CUDA compiler flags to avoid GCC intrinsic header issues
target_compile_options(silk_gpu_solver PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:
        --expt-relaxed-constexpr
        -Xcompiler=-mno-avx512f,-mno-avx512cd,-mno-avx512er,-mno-avx512pf,-mno-avx512vl,-mno-avx512bw,-mno-avx512dq,-mno-amx-tile,-mno-amx-int8,-mno-amx-bf16
    >
)

# Include directories
target_include_directories(silk_gpu_solver
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../..>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/silk/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_SOURCE_DIR}/silk/src
        ${CMAKE_SOURCE_DIR}/extern/pdqsort
)

# Link dependencies
target_link_libraries(silk_gpu_solver
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        spdlog::spdlog
        SuiteSparse::CHOLMOD
)

# CUDA architectures (adjust based on your GPU)
# Common options:
#   61 - GTX 1080, GTX 1070
#   75 - RTX 2080, RTX 2070
#   80 - A100
#   86 - RTX 3090, RTX 3080
#   89 - RTX 4090, RTX 4080
set_property(TARGET silk_gpu_solver PROPERTY CUDA_ARCHITECTURES 75 86 89)

# -------------------------------------------------
# Standalone Test Executable
# -------------------------------------------------

add_executable(test_gpu_solver test_gpu_solver.cu)

set_target_properties(test_gpu_solver PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
)

# Add same CUDA flags to test executable
target_compile_options(test_gpu_solver PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:
        --expt-relaxed-constexpr
        -Xcompiler=-mno-avx512f,-mno-avx512cd,-mno-avx512er,-mno-avx512pf,-mno-avx512vl,-mno-avx512bw,-mno-avx512dq,-mno-amx-tile,-mno-amx-int8,-mno-amx-bf16
    >
)

target_link_libraries(test_gpu_solver
    PRIVATE
        silk_gpu_solver
        Eigen3::Eigen
)

# Install GPU solver library
install(
    TARGETS silk_gpu_solver
    EXPORT silkTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install headers
install(
    FILES
        cloth_solver_kernels.cuh
        gpu_cloth_solver_context.hpp
        cloth_solver_utils.hpp
        pipeline.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/silk/solver/gpu
)
