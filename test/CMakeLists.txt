# ccd correctness test
add_executable(
    ccd_correctness_test
    ccd_correctness_test.cpp
    ccd_test_helper.cpp
    ../src/ccd.cpp
    ../src/ccd_poly.cpp
)
target_include_directories(ccd_correctness_test PRIVATE ../src)
target_link_libraries(ccd_correctness_test PRIVATE Catch2::Catch2WithMain)
target_link_libraries(ccd_correctness_test PRIVATE Eigen3::Eigen)
target_compile_definitions(
    ccd_correctness_test
    PRIVATE SAMPLE_QUERY_ROOT="${CMAKE_SOURCE_DIR}/misc/ccd-sample-queries"
)

# ccd performance test
add_executable(
    ccd_performance_test
    ccd_performance_test.cpp
    ccd_test_helper.cpp
    ../src/ccd.cpp
    ../src/ccd_poly.cpp
)
target_include_directories(ccd_performance_test PRIVATE ../src)
target_link_libraries(ccd_performance_test PRIVATE Catch2::Catch2WithMain)
target_link_libraries(ccd_performance_test PRIVATE Eigen3::Eigen)
target_compile_definitions(
    ccd_performance_test
    PRIVATE SAMPLE_QUERY_ROOT="${CMAKE_SOURCE_DIR}/misc/ccd-sample-queries"
)

# resource manager test
add_executable(
    resource_manager_test
    resource_manager_test.cpp
    ../src/resource_manager.cpp
)
target_include_directories(resource_manager_test PRIVATE ../src)
target_link_libraries(resource_manager_test PRIVATE Catch2::Catch2WithMain)

include(CTest)
include(Catch)
catch_discover_tests(ccd_correctness_test ccd_performance_test resource_manager_test)
