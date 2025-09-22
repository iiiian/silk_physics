include_guard(GLOBAL)

find_package(eigen3 REQUIRED)

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
find_package(tbb REQUIRED)

# suite sparse
find_package(SuiteSparse_config REQUIRED)
find_package(AMD REQUIRED)
find_package(CAMD REQUIRED)
find_package(COLAMD REQUIRED)
find_package(CCOLAMD REQUIRED)
find_package(CHOLMOD REQUIRED)

set(TIGHT_INCLUSION_WITH_DOUBLE_PRECISION OFF)
add_subdirectory(extern/tight-inclusion)

if(SILK_BUILD_DEMO)
    add_subdirectory(extern/polyscope)
    add_subdirectory(extern/portable-file-dialogs)
endif()

if(SILK_BUILD_TEST)
    find_package(catch2 REQUIRED)
    find_package(alembic REQUIRED)
endif()

