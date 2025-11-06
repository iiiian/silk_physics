include_guard(GLOBAL)

# ---------------------------------------------------------------
# System dependencies
# ---------------------------------------------------------------

find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

if (SILK_ENABLE_CUDA)
  find_package(CUDAToolkit REQUIRED)
endif()

# ---------------------------------------------------------------
# Other dependencies
# ---------------------------------------------------------------

include(FetchContent)

FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY https://github.com/eigen-mirror/eigen.git
    GIT_TAG 3147391d946bb4b6c68edd901f2add6ac1f31f8c # release 3.4.0
)

FetchContent_Declare(
    Libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG 40e7900ccbd767f1f360e0eb10f0f1a6432e0993 # release 2.6.0
)

set(SUITESPARSE_ENABLE_PROJECTS "suitesparse_config;amd;camd;colamd;ccolamd;cholmod;")
set(CHOLMOD_USE_CUDA OFF)
FetchContent_Declare(
    SuiteSparse
    GIT_REPOSITORY https://github.com/DrTimothyAldenDavis/SuiteSparse.git
    GIT_TAG b35a1f9318f4bd42085f4b5ea56f29c89d342d4d # release 7.11.0
)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG 644821ce28cb25d7992a4d0375b1d83214392592 # release 3.9.1
)

set(SPDLOG_INSTALL ON)
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG 6fa36017cfd5731d617e1a934f0e5ea9c4445b13 # release 1.15.3
)

set(TBB_TEST OFF)
set(TBB_STRICT OFF) # prevent tbb from throwing error due to gcc 15
FetchContent_Declare(
    tbb
    GIT_REPOSITORY https://github.com/uxlfoundation/oneTBB.git
    GIT_TAG 6f468b0385b2104a9f485e49bb55508d0024e32d
)

FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG 55f93686c01528224f448c19128836e7df245f72 # version 3.12.0
)

FetchContent_Declare(
    argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
    GIT_TAG 3eda91b2e1ce7d569f84ba295507c4cd8fd96910 # version 3.2
)

# A dependency of Alembic.
# Newest version does not work with Alembic, be careful when update.
FetchContent_Declare(
    Imath
    GIT_REPOSITORY https://github.com/AcademySoftwareFoundation/Imath.git
    GIT_TAG c0396a055a01bc537d32f435aee11a9b7ed6f0b5 # version 3.1.12
)

FetchContent_Declare(
    Alembic
    GIT_REPOSITORY https://github.com/alembic/alembic.git
    GIT_TAG 43a1489a0f5e15420e4be7225df86e819884b6fa # version 1.8.8
)

FetchContent_MakeAvailable(
    Eigen3
    Libigl
    SuiteSparse
    spdlog
    tbb
)

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
    add_subdirectory(extern/cuBQL)
endif()

if(SILK_BUILD_DEMO)
    FetchContent_MakeAvailable(argparse nlohmann_json Imath Alembic)
    add_subdirectory(extern/polyscope)
    add_subdirectory(extern/portable-file-dialogs)
endif()

if(SILK_BUILD_TEST)
  FetchContent_MakeAvailable(Catch2 Imath Alembic)
endif()
