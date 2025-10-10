set(DEMO_SOURCES
    src/demo.cpp
    src/main.cpp
    src/polyscope_silk_interop.cpp
    src/objects/cloth.cpp
    src/objects/obstacle.cpp
    src/objects/draw_utils.cpp
    src/widgets/help_bar_widget.cpp
    src/widgets/object_setting_widget.cpp
    src/widgets/gpu_solver_widget.cpp
    src/widgets/scene_widget.cpp
    src/widgets/sim_setting_widget.cpp
    src/widgets/simulator_widget.cpp
    src/widgets/statistic_widget.cpp
)

add_executable(demo ${DEMO_SOURCES})

include(${CMAKE_SOURCE_DIR}/cmake/SilkDebugFlags.cmake)
silk_debug_warnings(silk)

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

target_link_libraries(
    demo
    PRIVATE spdlog::spdlog
    PRIVATE Eigen3::Eigen
    PRIVATE ${SILK_IGL_CORE}
    PRIVATE polyscope
    PRIVATE portable_file_dialogs
    PRIVATE silk::silk
)
