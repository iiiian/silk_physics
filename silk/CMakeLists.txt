# -------------------------------------------------
# Library config
# -------------------------------------------------

set(SILK_SOURCES 
    src/backend/cpu/collision/bbox.cpp
    src/backend/cpu/collision/ccd.cpp
    src/backend/cpu/collision/interval.cpp
    src/backend/cpu/collision/interval_root_finder.cpp
    src/backend/cpu/collision/narrowphase.cpp
    src/backend/cpu/collision/object_collider.cpp
    src/backend/cpu/collision/pipeline.cpp
    src/backend/cpu/cpu_backend.cpp
    src/backend/cpu/ecs.cpp
    src/backend/cpu/solver/cholmod_utils.cpp
    src/backend/cpu/solver/cloth_solver_context.cpp
    src/backend/cpu/solver/cloth_solver_utils.cpp
    src/backend/cpu/solver/obstacle_solver_utils.cpp
    src/backend/cpu/solver/pipeline.cpp
    src/common/cloth_topology.cpp
    src/common/mesh.cpp
    src/common/result.cpp
    src/common/world.cpp
    src/common/check_cuda_support.cpp
)

set(SILK_CUDA_SOURCES
    src/backend/cuda/collision/bbox.cpp
    src/backend/cuda/collision/ccd.cpp
    src/backend/cuda/collision/interval.cpp
    src/backend/cuda/collision/interval_root_finder.cpp
    src/backend/cuda/collision/narrowphase.cpp
    src/backend/cuda/collision/object_collider.cpp
    src/backend/cuda/collision/pipeline.cpp
    src/backend/cuda/csr_matrix.cpp
    src/backend/cuda/sparse_matrix_interop.cpp
    src/backend/cuda/cuda_backend.cpp
    src/backend/cuda/ecs.cpp
    src/backend/cuda/object_state.cpp
    src/backend/cuda/solver/barrier_constrain.cpp
    src/backend/cuda/solver/cloth_solver_context.cpp
    src/backend/cuda/solver/cloth_solver_utils.cpp
    src/backend/cuda/solver/obstacle_solver_utils.cpp
    src/backend/cuda/solver/pipeline.cpp
    src/backend/cuda/solver/cloth_solver_kernel.cu
    src/backend/cuda/solver/pipeline_kernel.cu
    src/backend/cuda/solver/svd.cu
    src/backend/cuda/solver/compute_subspace_u.cpp
    src/backend/cuda/solver/inexact_solver_kernel.cu
    src/backend/cuda/solver/inexact_solver.cpp
    src/backend/cuda/solver/a_jacobi_solver.cu
)

add_library(silk ${SILK_SOURCES})
add_library(silk::silk ALIAS silk)

set_target_properties(silk PROPERTIES VERSION 0.1.0)
if(SILK_ENABLE_CUDA)
    set_target_properties(silk PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif()

include(${CMAKE_SOURCE_DIR}/cmake/SilkDebugFlags.cmake)
silk_debug_warnings(silk)

target_include_directories(
    silk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_SOURCE_DIR}/extern/pdqsort
        ${CMAKE_CURRENT_LIST_DIR}/src
)

# Define SILK_WITH_CUDA macro if cuda is enabled.
target_compile_definitions(
    silk
    PUBLIC
        $<$<BOOL:${SILK_ENABLE_CUDA}>:SILK_WITH_CUDA=1>
)

target_link_libraries(
    silk
    PRIVATE
        Eigen3::Eigen
        ${SILK_IGL_CORE}
        spdlog::spdlog
        TBB::tbb
        # dependencies for SuiteSparse cholmod
        SuiteSparse::SuiteSparseConfig
        SuiteSparse::AMD
        SuiteSparse::CAMD
        SuiteSparse::COLAMD
        SuiteSparse::CCOLAMD
        SuiteSparse::CHOLMOD
)

if(SILK_ENABLE_CUDA)
    target_link_libraries(silk PRIVATE CUDA::cudart CUDA::cusparse Spectra::Spectra)
    target_sources(silk PRIVATE ${SILK_CUDA_SOURCES})
endif()

# -------------------------------------------------
# Installation rules
# -------------------------------------------------

include(GNUInstallDirs)

install(
    TARGETS silk
    EXPORT silkTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# -------------------------------------------------
# Generate and install package config files
# -------------------------------------------------

include(CMakePackageConfigHelpers)

configure_package_config_file(
    cmake/silkConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/silkConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/silk
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/silkConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    EXPORT silkTargets
    FILE silkTargets.cmake
    NAMESPACE silk::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/silk
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/silkConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/silkConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/silk
)
