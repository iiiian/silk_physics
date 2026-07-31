include_guard(GLOBAL)

# ---------------------------------------------------------------
# System dependencies
# ---------------------------------------------------------------

# SuiteSparse reuqires BLAS and LAPACK.
# Building BLAS/LAPCK from source is not trivial because:
# 1. lapack target name conflict with Eigen.
# 2. Buggy SuiteSparse cmake export setup.
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

if (SILK_ENABLE_CUDA)
  find_package(CUDAToolkit REQUIRED)
endif()

# ---------------------------------------------------------------
# Other dependencies
# ---------------------------------------------------------------

include(FetchContent)

# Eigen's BUILD_TESTING is a very generic name, which conflicts with GkLib BUILD_TESTING.
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    Eigen3
    SYSTEM
    GIT_REPOSITORY https://github.com/eigen-mirror/eigen.git
    GIT_TAG 3147391d946bb4b6c68edd901f2add6ac1f31f8c # release 3.4.0
)

FetchContent_Declare(
    Libigl
    SYSTEM
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG 40e7900ccbd767f1f360e0eb10f0f1a6432e0993 # release 2.6.0
)

set(SUITESPARSE_ENABLE_PROJECTS "suitesparse_config;amd;camd;colamd;ccolamd;cholmod;")
set(CHOLMOD_USE_CUDA OFF)
FetchContent_Declare(
    SuiteSparse
    SYSTEM
    GIT_REPOSITORY https://github.com/DrTimothyAldenDavis/SuiteSparse.git
    GIT_TAG b35a1f9318f4bd42085f4b5ea56f29c89d342d4d # release 7.11.0
)

FetchContent_Declare(
    Catch2
    SYSTEM
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG 644821ce28cb25d7992a4d0375b1d83214392592 # release 3.9.1
)

set(SPDLOG_INSTALL ON)
FetchContent_Declare(
    spdlog
    SYSTEM
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG 6fa36017cfd5731d617e1a934f0e5ea9c4445b13 # release 1.15.3
)

set(TBB_TEST OFF)
set(TBB_STRICT OFF) # prevent tbb from throwing error due to gcc 15
FetchContent_Declare(
    tbb
    SYSTEM
    GIT_REPOSITORY https://github.com/uxlfoundation/oneTBB.git
    GIT_TAG 6f468b0385b2104a9f485e49bb55508d0024e32d
)

set(HWY_ENABLE_CONTRIB OFF CACHE BOOL "" FORCE)
set(HWY_ENABLE_EXAMPLES OFF CACHE BOOL "" FORCE)
set(HWY_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
set(HWY_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(HWY_FORCE_STATIC_LIBS ON CACHE BOOL "" FORCE)
FetchContent_Declare(
    hwy
    SYSTEM
    GIT_REPOSITORY https://github.com/google/highway.git
    GIT_TAG 2607d3b5b0113992fe84d3848859eae13b3b52c1 # version 1.4.0
)

FetchContent_Declare(
    nlohmann_json
    SYSTEM
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG 55f93686c01528224f448c19128836e7df245f72 # version 3.12.0
)

FetchContent_Declare(
    argparse
    SYSTEM
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
    GIT_TAG 3eda91b2e1ce7d569f84ba295507c4cd8fd96910 # version 3.2
)

FetchContent_Declare(
    ScalableCCD
    SYSTEM
    GIT_REPOSITORY https://github.com/Continuous-Collision-Detection/Scalable-CCD.git
    GIT_TAG 8f9347c1afc36f2dda17424c15ff5b68087fe8dc
)

FetchContent_Declare(
    Embree
    SYSTEM
    GIT_REPOSITORY https://github.com/RenderKit/embree.git
    GIT_TAG f590db83ef6559387df7f6d8725c34fb7acf851d
)

FetchContent_Declare(
    cuBQL
    SYSTEM
    GIT_REPOSITORY https://github.com/NVIDIA/cuBQL.git
    GIT_TAG 812464c6510cbfc42f455f0008d3ccff78662993
)

# A dependency of Alembic.
# Newest version does not work with Alembic, be careful when update.
FetchContent_Declare(
    Imath
    SYSTEM
    GIT_REPOSITORY https://github.com/AcademySoftwareFoundation/Imath.git
    GIT_TAG c0396a055a01bc537d32f435aee11a9b7ed6f0b5 # version 3.1.12
    OVERRIDE_FIND_PACKAGE
)

set(USE_BINARIES OFF CACHE BOOL "Build Alembic command line tools" FORCE)
set(USE_TESTS OFF CACHE BOOL "Build Alembic tests" FORCE)
FetchContent_Declare(
    Alembic
    SYSTEM
    GIT_REPOSITORY https://github.com/alembic/alembic.git
    GIT_TAG 43a1489a0f5e15420e4be7225df86e819884b6fa # version 1.8.8
)

FetchContent_Declare(
    CCCL
    SYSTEM
    GIT_REPOSITORY https://github.com/NVIDIA/cccl.git
    GIT_TAG d84981c797eb186e45f883f85423df94f9ac8bf4 # version 3.3.3
)

if(SILK_GRAPH_PARTITION_BACKEND STREQUAL "kaminpar")
    set(KAMINPAR_BUILD_APPS OFF)
    set(KAMINPAR_BUILD_IO OFF)
    set(KAMINPAR_BUILD_WITH_SPARSEHASH OFF)
    set(KAMINPAR_BUILD_WITH_DEBUG_SYMBOLS OFF)
    set(KAMINPAR_ENABLE_TIMERS OFF)
    # Below hack force kaminpar to use our vendored tbb
    set(KAMINPAR_DOWNLOAD_TBB ON)
    set(TBB_FOUND TRUE)
    FetchContent_Declare(
        KaMinPar
        SYSTEM
        GIT_REPOSITORY https://github.com/KaHIP/KaMinPar.git
        GIT_TAG 00fa1ef4150b558a1260918fe8dc49dae048ea62 # version 3.7.3
    )
elseif(SILK_GRAPH_PARTITION_BACKEND STREQUAL "metis")
    FetchContent_Declare(
        METIS
        SYSTEM
        GIT_REPOSITORY https://github.com/scivision/METIS.git
        GIT_TAG 777472ae3cd15a8e6d1e5b7d6c347d21947e3ab2
    )
endif()


FetchContent_MakeAvailable(
    Eigen3
    Libigl
    SuiteSparse
    spdlog
    tbb
    hwy
)

add_library(hwy::hwy ALIAS hwy)

# The original libigl export library as igl::core, but vcpkg patch it to igl::igl_core.
# So we check which one is available. 
if (TARGET igl::core)
  set(SILK_IGL_CORE igl::core)
elseif (TARGET igl::igl_core)
  set(SILK_IGL_CORE igl::igl_core)
else()
  message(FATAL_ERROR "Could not find libigl core target")
endif()

if (SILK_ENABLE_CUDA)
    if(SILK_GRAPH_PARTITION_BACKEND STREQUAL "kaminpar")
        FetchContent_MakeAvailable(CCCL KaMinPar)
        # So we could link KaMinPar as shared lib.
        if (TARGET KaMinParCommon)
            set_target_properties(KaMinParCommon PROPERTIES
                POSITION_INDEPENDENT_CODE ON
            )
        endif()
    elseif(SILK_GRAPH_PARTITION_BACKEND STREQUAL "metis")
        FetchContent_MakeAvailable(CCCL METIS)
    endif()
endif()

if(SILK_BUILD_DEMO OR SILK_BROADPHASE_BENCHMARKS)
    FetchContent_MakeAvailable(argparse nlohmann_json)
endif()

if(SILK_BUILD_DEMO)
    FetchContent_MakeAvailable(Imath Alembic)
    add_subdirectory(extern/polyscope)
    add_subdirectory(extern/portable-file-dialogs)
endif()

if(SILK_BROADPHASE_BENCHMARKS)
    set(SCALABLE_CCD_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(SCALABLE_CCD_WITH_PROFILER OFF CACHE BOOL "" FORCE)
    set(SCALABLE_CCD_WITH_CUDA ${SILK_ENABLE_CUDA} CACHE BOOL "" FORCE)
    set(SCALABLE_CCD_USE_DOUBLE OFF CACHE BOOL "" FORCE)

    set(EMBREE_TUTORIALS OFF CACHE BOOL "" FORCE)
    set(EMBREE_ISPC_SUPPORT OFF CACHE BOOL "" FORCE)
    set(EMBREE_SYCL_SUPPORT OFF CACHE BOOL "" FORCE)
    set(EMBREE_STATIC_LIB ON CACHE BOOL "" FORCE)
    set(EMBREE_TASKING_SYSTEM TBB CACHE STRING "" FORCE)
    set(EMBREE_GEOMETRY_TRIANGLE OFF CACHE BOOL "" FORCE)
    set(EMBREE_GEOMETRY_QUAD OFF CACHE BOOL "" FORCE)
    set(EMBREE_GEOMETRY_CURVE OFF CACHE BOOL "" FORCE)
    set(EMBREE_GEOMETRY_SUBDIVISION OFF CACHE BOOL "" FORCE)
    set(EMBREE_GEOMETRY_INSTANCE OFF CACHE BOOL "" FORCE)
    set(EMBREE_GEOMETRY_INSTANCE_ARRAY OFF CACHE BOOL "" FORCE)
    set(EMBREE_GEOMETRY_GRID OFF CACHE BOOL "" FORCE)
    set(EMBREE_GEOMETRY_POINT OFF CACHE BOOL "" FORCE)
    set(EMBREE_GEOMETRY_USER ON CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(ScalableCCD Embree)

    if(SILK_ENABLE_CUDA)
        set(CUBQL_DISABLE_CUDA OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(cuBQL)
    endif()
endif()

if(SILK_BUILD_TEST)
  FetchContent_MakeAvailable(Catch2 Imath Alembic)
endif()
