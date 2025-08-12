set(DEMO_SOURCES
    src/demo.cpp
    src/kd_tree.cpp
    src/main.cpp
    src/widgets/help_bar_widget.cpp
    src/widgets/object_setting_widget.cpp
    src/widgets/scene_widget.cpp
    src/widgets/sim_setting_widget.cpp
    src/widgets/simulator_widget.cpp
    src/widgets/statistic_widget.cpp
)

add_executable(demo ${DEMO_SOURCES})

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

target_link_libraries(
    demo
    PRIVATE spdlog::spdlog
    PRIVATE Eigen3::Eigen
    PRIVATE igl::core
    PRIVATE polyscope
    PRIVATE portable_file_dialogs
    PRIVATE silk::silk
)
