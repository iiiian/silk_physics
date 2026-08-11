# -------------------------------------------------
# Library config
# -------------------------------------------------

file(GLOB_RECURSE SILK_SOURCES
    include/*.hpp include/*.cpp
    src/common/*.hpp src/common/*.cpp
    src/ecs/*.hpp src/ecs/*.cpp
    src/backend/cpu/*.hpp src/backend/cpu/*.cpp
)

file(GLOB_RECURSE SILK_CUDA_SOURCES
    src/backend/cuda/*.hpp src/backend/cuda/*.cpp
    src/backend/cuda/*.cuh src/backend/cuda/*.cu
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
        ${CMAKE_CURRENT_LIST_DIR}/src
)

# Define SILK_WITH_CUDA macro if cuda is enabled.
target_compile_definitions(
    silk
    PRIVATE
        SILK_ENABLE_TIMING=$<BOOL:${SILK_ENABLE_TIMING}>
        $<$<BOOL:${SILK_ENABLE_CUDA}>:SILK_WITH_CUDA=1>
        $<$<STREQUAL:${SILK_GRAPH_PARTITION_BACKEND},kaminpar>:SILK_GRAPH_PARTITION_BACKEND_KAMINPAR=1>
        $<$<STREQUAL:${SILK_GRAPH_PARTITION_BACKEND},metis>:SILK_GRAPH_PARTITION_BACKEND_METIS=1>
        # Eigen's aligned malloc/free helpers are header-only and can be
        # instantiated in both host C++ and CUDA translation units.  If Eigen
        # auto-detects different EIGEN_MALLOC_ALREADY_ALIGNED values in those
        # TUs, an Eigen object allocated in one TU can be destroyed in another
        # with the wrong free path, e.g. handmade_aligned_malloc paired with
        # plain free(), causing "double free or corruption" during teardown.
        # Force the conservative handmade aligned allocator for CUDA builds so
        # allocation and deallocation use the same path everywhere.
        $<$<BOOL:${SILK_ENABLE_CUDA}>:EIGEN_MALLOC_ALREADY_ALIGNED=0>
        $<$<BOOL:${SILK_ENABLE_CUDA}>:EIGEN_NO_CUDA>
)

target_link_libraries(
    silk
    PRIVATE
        Eigen3::Eigen
        ${SILK_IGL_CORE}
        spdlog::spdlog
        TBB::tbb
        $<BUILD_INTERFACE:hwy::hwy>
        # dependencies for SuiteSparse cholmod
        SuiteSparse::SuiteSparseConfig
        SuiteSparse::AMD
        SuiteSparse::CAMD
        SuiteSparse::COLAMD
        SuiteSparse::CCOLAMD
        SuiteSparse::CHOLMOD
)

if(SILK_ENABLE_TIMING)
    target_link_libraries(silk PRIVATE nlohmann_json::nlohmann_json)
endif()

if(SILK_ENABLE_CUDA)
    target_sources(silk PRIVATE ${SILK_CUDA_SOURCES})
    target_compile_options(silk PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda --expt-relaxed-constexpr>)
    target_link_libraries(silk PRIVATE CUDA::cudart CUDA::cusparse CCCL::CCCL)

    if(SILK_GRAPH_PARTITION_BACKEND STREQUAL "kaminpar")
        target_link_libraries(silk PRIVATE KaMinPar::KaMinPar)
    elseif(SILK_GRAPH_PARTITION_BACKEND STREQUAL "metis")
        target_link_libraries(silk PRIVATE metis::metis)
    endif()
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
