add_subdirectory(${CMAKE_SOURCE_DIR}/extern/spdlog)
add_subdirectory(${CMAKE_SOURCE_DIR}/extern/libigl)
add_subdirectory(${CMAKE_SOURCE_DIR}/extern/polyscope)
add_subdirectory(${CMAKE_SOURCE_DIR}/extern/portable-file-dialogs)

file(GLOB SOURCES "src/*.cpp")
add_executable(
    demo
    "${CMAKE_SOURCE_DIR}/extern/polyscope/deps/imgui/imgui/misc/cpp/imgui_stdlib.cpp"
    ${SOURCES}
)

# set debug flag
target_compile_options(
    demo
    PRIVATE
        $<$<CONFIG:Debug>:
        -Wall
        -Wextra
        -fdiagnostics-color
        -Wno-unused-parameter
        -Wno-sign-compare # prevent libigl vomiting error
        >
)

# set spdlog log level based on build type
target_compile_definitions(
    demo
    PRIVATE
        $<$<CONFIG:Debug>:
        SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG>
        $<$<CONFIG:RelWithDebInfo>:SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO>
        $<$<CONFIG:Release>:
        SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_WARN>
)

target_link_libraries(demo PRIVATE spdlog::spdlog)
target_link_libraries(demo PRIVATE igl::core)
target_link_libraries(demo PRIVATE polyscope)
target_link_libraries(demo PRIVATE portable_file_dialogs)
target_link_libraries(demo PRIVATE silk::silk)
