# src/Examples/F16AltitudeChange/CMakeLists.txt
# NASA TM-2015-218675 Atmospheric Scenario 13.1 — F-16 Subsonic Altitude Change
# Closed-loop with LQR SAS + autopilot (F16_control.dml).
#
# NOTE: the closed-loop LQR is stiff — use timeStep <= 0.02 s for accurate results.
# dt = 0.1 s produces numerical instability (roll divergence, 12 ft altitude error).
# dt = 0.02 s fully converges: 100.6 ft altitude change vs 100 ft command, roll < 0.11 deg.
# Recommended run command:
#   F16AltitudeChange --endTime 20 --timeStep 0.02 --outputFileName f16_s13p1.csv

set(F16_DATA_DIR "${CMAKE_SOURCE_DIR}/data/F16_S119_source")

add_executable(F16AltitudeChange ${CMAKE_CURRENT_SOURCE_DIR}/F16AltitudeChange.cpp)

target_compile_features(F16AltitudeChange PRIVATE cxx_std_23)

target_compile_options(F16AltitudeChange PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>)

target_compile_definitions(
  F16AltitudeChange
  PRIVATE
    "F16_INERTIA_FILE=\"${F16_DATA_DIR}/F16_inertia.dml\""
    "F16_AERO_FILE=\"${F16_DATA_DIR}/F16_aero.dml\""
    "F16_PROP_FILE=\"${F16_DATA_DIR}/F16_prop.dml\""
    "F16_CONTROL_FILE=\"${F16_DATA_DIR}/F16_control.dml\"")

target_include_directories(
  F16AltitudeChange
  PRIVATE
    "${CMAKE_SOURCE_DIR}/vendor/CppAD/include"
    "${CMAKE_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml")

target_link_libraries(F16AltitudeChange
  PRIVATE Aetherion::Aetherion Aetherion::Simulation CppAD::cppad)

# Copy NASA Atmos-13.1 reference CSVs next to the executable
set(F16_REF_DIR "${CMAKE_SOURCE_DIR}/data/Atmos_13p1_SubsonicAltitudeChangeF16")
foreach(_csv Atmos_13p1_sim_02.csv Atmos_13p1_sim_04.csv Atmos_13p1_sim_05.csv)
  add_custom_command(
    TARGET F16AltitudeChange
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${F16_REF_DIR}/${_csv}"
      "$<TARGET_FILE_DIR:F16AltitudeChange>/${_csv}"
    COMMENT "Copying NASA Atmos-13.1 reference ${_csv}")
endforeach()

# Copy the Case-13.1 plotting script next to the executable
add_custom_command(
  TARGET F16AltitudeChange
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_SOURCE_DIR}/scripts/plot_f16_s13p1_nasa02.py"
    "$<TARGET_FILE_DIR:F16AltitudeChange>/plot_f16_s13p1_nasa02.py"
  COMMENT "Copying Case-13.1 plot script plot_f16_s13p1_nasa02.py")
