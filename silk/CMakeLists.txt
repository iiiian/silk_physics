# -------------------------------------------------
# Library config
# -------------------------------------------------

file(GLOB SILK_SOURCES 
    src/cloth_topology.cpp
    src/collision/cpu/bbox.cpp
    src/collision/cpu/narrowphase.cpp
    src/collision/cpu/object_collider.cpp
    src/collision/cpu/pipeline.cpp
    src/collision/cpu/interval.cpp
    src/collision/cpu/interval_root_finder.cpp
    src/collision/cpu/ccd.cpp
    src/ecs.cpp
    src/mesh.cpp
    src/result.cpp
    src/solver/cpu/cholmod_utils.cpp
    src/solver/cpu/cloth_solver_context.cpp
    src/solver/cpu/cloth_solver_utils.cpp
    src/solver/cpu/obstacle_solver_utils.cpp
    src/solver/cpu/pipeline.cpp
    src/world.cpp
)

file(GLOB SILK_GPU_SOURCE
    src/solver/gpu/cloth_solver_kernels.cu
    src/solver/gpu/gpu_cloth_solver_context.cu
    src/solver/gpu/cloth_solver_utils.cu
    src/solver/gpu/pipeline.cpp
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
    PRIVATE ${CMAKE_SOURCE_DIR}/extern/pdqsort ${CMAKE_CURRENT_LIST_DIR}/src
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
    target_link_libraries(silk PRIVATE CUDA::cudart)
    target_sources(silk PRIVATE ${SILK_GPU_SOURCE})
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
