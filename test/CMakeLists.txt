add_executable(ccd_test ccd_test.cpp ../src/ccd.cpp ../src/ccd_poly.cpp)
target_include_directories(ccd_test PRIVATE ../src)
target_link_libraries(ccd_test PRIVATE Catch2::Catch2WithMain)
target_link_libraries(ccd_test PRIVATE Eigen3::Eigen)

target_compile_definitions(
    ccd_test
    PRIVATE SAMPLE_QUERY_ROOT="${CMAKE_SOURCE_DIR}/misc/ccd-sample-queries"
)

include(CTest)
include(Catch)
catch_discover_tests(ccd_test)
