include_guard(GLOBAL)

if (SILK_ENABLE_CUDA)
  find_package(CUDAToolkit REQUIRED)
  find_package(CCCL 3.2 REQUIRED)
  if(SILK_GRAPH_PARTITION_BACKEND STREQUAL "kaminpar")
    find_package(KaMinPar REQUIRED)
  elseif(SILK_GRAPH_PARTITION_BACKEND STREQUAL "metis")
    find_package(metis REQUIRED)
  endif()
endif()

find_package(Eigen3 REQUIRED)

find_package(libigl REQUIRED)
# The original libigl export library as igl::core, but vcpkg patch it to igl::igl_core.
# So we check which one is available. 
if (TARGET igl::core)
  set(SILK_IGL_CORE igl::core)
elseif (TARGET igl::igl_core)
  set(SILK_IGL_CORE igl::igl_core)
else()
  message(FATAL_ERROR "Could not find libigl core target")
endif()

find_package(spdlog REQUIRED)
find_package(TBB REQUIRED)
find_package(hwy CONFIG REQUIRED)

# suite sparse
find_package(SuiteSparse_config REQUIRED)
find_package(AMD REQUIRED)
find_package(CAMD REQUIRED)
find_package(COLAMD REQUIRED)
find_package(CCOLAMD REQUIRED)
find_package(CHOLMOD REQUIRED)

if(SILK_BUILD_DEMO OR SILK_BROADPHASE_BENCHMARKS OR SILK_NARROWPHASE_BENCHMARKS)
    find_package(argparse REQUIRED)
    find_package(nlohmann_json REQUIRED)
endif()

if(SILK_BUILD_DEMO)
    find_package(Alembic REQUIRED)
    add_subdirectory(extern/polyscope)
    add_subdirectory(extern/portable-file-dialogs)
endif()

if(SILK_BROADPHASE_BENCHMARKS)
    find_package(ScalableCCD CONFIG REQUIRED)
endif()

if(SILK_NARROWPHASE_BENCHMARKS AND SILK_ENABLE_CUDA)
    find_package(CCDGPU CONFIG REQUIRED)
endif()

if(SILK_BROADPHASE_BENCHMARKS)
    find_package(embree 4 CONFIG REQUIRED)
    if(SILK_ENABLE_CUDA)
        find_package(cuBQL CONFIG REQUIRED)
    endif()
endif()

if(SILK_NARROWPHASE_BENCHMARKS)
    find_package(TightInclusion CONFIG REQUIRED)
endif()

if(SILK_BUILD_TEST)
    find_package(Catch2 REQUIRED)
    find_package(Alembic REQUIRED)
endif()
