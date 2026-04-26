# src/Examples/NorthwardCannonball/CMakeLists.txt
# NASA TM-2015-218675 Atmospheric Scenario 10 — Northward Cannonball

add_executable(NorthwardCannonball
    ${CMAKE_CURRENT_SOURCE_DIR}/NorthwardCannonball.cpp
)

target_compile_features(NorthwardCannonball PRIVATE cxx_std_23)

target_compile_options(NorthwardCannonball PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>
)

target_link_libraries(NorthwardCannonball
    PRIVATE
        Aetherion::Aetherion
        Aetherion::Simulation
)

add_custom_command(TARGET NorthwardCannonball POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/data/Atmos_10_NorthwardCannonball/nasa_2015_scenario10_northward_cannonball.json"
        "$<TARGET_FILE_DIR:NorthwardCannonball>/nasa_2015_scenario10_northward_cannonball.json"
    COMMENT "Copying NASA Atmos-10 config to build directory"
)
