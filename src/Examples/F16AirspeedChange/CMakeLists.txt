# src/Examples/F16AirspeedChange/CMakeLists.txt NASA TM-2015-218675 Atmospheric Scenario 13.2 — F-16 Subsonic Airspeed
# Change Closed-loop with LQR SAS + autopilot; reuses F16AltitudeChangeSimulator.
#
# NOTE: the closed-loop LQR is stiff — use timeStep <= 0.02 s for accurate results. Recommended run command:
# F16AirspeedChange --endTime 20 --timeStep 0.02 --outputFileName f16_s13p2.csv

set(F16_DATA_DIR "${CMAKE_SOURCE_DIR}/data/F16_S119_source")

add_executable(F16AirspeedChange ${CMAKE_CURRENT_SOURCE_DIR}/F16AirspeedChange.cpp)

target_compile_features(F16AirspeedChange PRIVATE cxx_std_23)

target_compile_options(F16AirspeedChange PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>)

target_compile_definitions(
  F16AirspeedChange
  PRIVATE "F16_INERTIA_FILE=\"${F16_DATA_DIR}/F16_inertia.dml\""
          "F16_AERO_FILE=\"${F16_DATA_DIR}/F16_aero.dml\""
          "F16_PROP_FILE=\"${F16_DATA_DIR}/F16_prop.dml\""
          "F16_CONTROL_FILE=\"${F16_DATA_DIR}/F16_control.dml\"")

target_include_directories(F16AirspeedChange PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include"
                                                     "${CMAKE_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml")

target_link_libraries(F16AirspeedChange PRIVATE Aetherion::Aetherion Aetherion::Simulation CppAD::cppad)

# Copy NASA Atmos-13.2 reference CSVs next to the executable
set(F16_REF_DIR "${CMAKE_SOURCE_DIR}/data/Atmos_13p2_SubsonicAirspeedChangeF16")
foreach(_csv Atmos_13p2_sim_02.csv Atmos_13p2_sim_04.csv Atmos_13p2_sim_05.csv)
  add_custom_command(
    TARGET F16AirspeedChange
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${F16_REF_DIR}/${_csv}"
            "$<TARGET_FILE_DIR:F16AirspeedChange>/${_csv}"
    COMMENT "Copying NASA Atmos-13.2 reference ${_csv}")
endforeach()

# Copy the Case-13.2 plotting script next to the executable
add_custom_command(
  TARGET F16AirspeedChange
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_SOURCE_DIR}/scripts/plot_f16_s13p2_nasa02.py"
          "$<TARGET_FILE_DIR:F16AirspeedChange>/plot_f16_s13p2_nasa02.py"
  COMMENT "Copying Case-13.2 plot script plot_f16_s13p2_nasa02.py")
