# src/Examples/DroppedSphere2DWindShear/CMakeLists.txt
# NASA TM-2015-218675 Atmospheric Scenario 8 — Dropped Sphere with 2D Wind Shear

add_executable(DroppedSphere2DWindShear
    ${CMAKE_CURRENT_SOURCE_DIR}/DroppedSphere2DWindShear.cpp
)

target_compile_features(DroppedSphere2DWindShear PRIVATE cxx_std_23)

target_compile_options(DroppedSphere2DWindShear PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>
)

target_link_libraries(DroppedSphere2DWindShear
    PRIVATE
        Aetherion::Aetherion
        Aetherion::Simulation
)

add_custom_command(TARGET DroppedSphere2DWindShear POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/data/Atmos_08_DroppedSphere2DWindShear/nasa_2015_scenario8_dropped_sphere_2d_wind_shear.json"
        "$<TARGET_FILE_DIR:DroppedSphere2DWindShear>/nasa_2015_scenario8_dropped_sphere_2d_wind_shear.json"
    COMMENT "Copying NASA Atmos-08 config to build directory"
)
