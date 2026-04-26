# src/Examples/EastwardCannonball/CMakeLists.txt
# NASA TM-2015-218675 Atmospheric Scenario 9 — Eastward Cannonball

add_executable(EastwardCannonball
    ${CMAKE_CURRENT_SOURCE_DIR}/EastwardCannonball.cpp
)

target_compile_features(EastwardCannonball PRIVATE cxx_std_23)

target_compile_options(EastwardCannonball PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>
)

target_link_libraries(EastwardCannonball
    PRIVATE
        Aetherion::Aetherion
        Aetherion::Simulation
)

add_custom_command(TARGET EastwardCannonball POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/data/Atmos_09_EastwardCannonball/nasa_2015_scenario9_eastward_cannonball.json"
        "$<TARGET_FILE_DIR:EastwardCannonball>/nasa_2015_scenario9_eastward_cannonball.json"
    COMMENT "Copying NASA Atmos-09 config to build directory"
)
