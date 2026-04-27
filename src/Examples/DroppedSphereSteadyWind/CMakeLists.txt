# src/Examples/DroppedSphereSteadyWind/CMakeLists.txt
# NASA TM-2015-218675 Atmospheric Scenario 7 — Dropped Sphere with Steady Wind

add_executable(DroppedSphereSteadyWind
    ${CMAKE_CURRENT_SOURCE_DIR}/DroppedSphereSteadyWind.cpp
)

target_compile_features(DroppedSphereSteadyWind PRIVATE cxx_std_23)

target_compile_options(DroppedSphereSteadyWind PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>
)

target_link_libraries(DroppedSphereSteadyWind
    PRIVATE
        Aetherion::Aetherion
        Aetherion::Simulation
)

add_custom_command(TARGET DroppedSphereSteadyWind POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/data/Atmos_07_DroppedSphereSteadyWind/nasa_2015_scenario7_dropped_sphere_steady_wind.json"
        "$<TARGET_FILE_DIR:DroppedSphereSteadyWind>/nasa_2015_scenario7_dropped_sphere_steady_wind.json"
    COMMENT "Copying NASA Atmos-07 config to build directory"
)
