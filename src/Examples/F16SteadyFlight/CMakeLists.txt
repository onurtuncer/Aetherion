# src/Examples/F16SteadyFlight/CMakeLists.txt
# NASA TM-2015-218675 Atmospheric Scenario 11 — F-16 Steady Straight-and-Level Flight

set(F16_DATA_DIR "${CMAKE_SOURCE_DIR}/data/F16_S119_source")

add_executable(F16SteadyFlight
    ${CMAKE_CURRENT_SOURCE_DIR}/F16SteadyFlight.cpp
)

target_compile_features(F16SteadyFlight PRIVATE cxx_std_23)

target_compile_options(F16SteadyFlight PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>
)

target_compile_definitions(F16SteadyFlight PRIVATE
    "F16_INERTIA_FILE=\"${F16_DATA_DIR}/F16_inertia.dml\""
    "F16_AERO_FILE=\"${F16_DATA_DIR}/F16_aero.dml\""
    "F16_PROP_FILE=\"${F16_DATA_DIR}/F16_prop.dml\""
)

target_include_directories(F16SteadyFlight PRIVATE
    "${CMAKE_SOURCE_DIR}/vendor/CppAD/include"
    "${CMAKE_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml"
)

target_link_libraries(F16SteadyFlight
    PRIVATE
        Aetherion::Aetherion
        Aetherion::Simulation
        CppAD::cppad
)

# Copy NASA Atmos-11 reference CSVs next to the executable for comparison
set(F16_REF_DIR "${CMAKE_SOURCE_DIR}/data/Atmos_11_TrimCheckSubsonicF16")
foreach(_csv Atmos_11_sim_02.csv Atmos_11_sim_04.csv Atmos_11_sim_05.csv)
    add_custom_command(TARGET F16SteadyFlight POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${F16_REF_DIR}/${_csv}"
            "$<TARGET_FILE_DIR:F16SteadyFlight>/${_csv}"
        COMMENT "Copying NASA Atmos-11 reference ${_csv}"
    )
endforeach()
