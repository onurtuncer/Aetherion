# src/Examples/F16AltitudeChange/CMakeLists.txt
# NASA TM-2015-218675 Atmospheric Scenario 13.1 — F-16 Subsonic Altitude Change
# Closed-loop with LQR SAS + autopilot (F16_control.dml).

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
