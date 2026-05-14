# src/Examples/F16HeadingChange/CMakeLists.txt NASA TM-2015-218675 Atmospheric Scenario 13.3 — F-16 Subsonic Heading
# Change Closed-loop with LQR SAS + autopilot; reuses F16AltitudeChangeSimulator.
#
# NOTE: the closed-loop LQR is stiff — use timeStep <= 0.02 s for accurate results. Recommended run command:
# F16HeadingChange --endTime 30 --timeStep 0.02 --outputFileName f16_s13p3_sim.csv

set(F16_DATA_DIR "${CMAKE_SOURCE_DIR}/data/F16_S119_source")

add_executable(F16HeadingChange ${CMAKE_CURRENT_SOURCE_DIR}/F16HeadingChange.cpp)

target_compile_features(F16HeadingChange PRIVATE cxx_std_23)

target_compile_options(F16HeadingChange PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>)

target_compile_definitions(
  F16HeadingChange
  PRIVATE "F16_INERTIA_FILE=\"${F16_DATA_DIR}/F16_inertia.dml\""
          "F16_AERO_FILE=\"${F16_DATA_DIR}/F16_aero.dml\""
          "F16_PROP_FILE=\"${F16_DATA_DIR}/F16_prop.dml\""
          "F16_CONTROL_FILE=\"${F16_DATA_DIR}/F16_control.dml\"")

target_include_directories(F16HeadingChange PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include"
                                                    "${CMAKE_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml")

target_link_libraries(F16HeadingChange PRIVATE Aetherion::Aetherion Aetherion::Simulation CppAD::cppad)

# Copy NASA Atmos-13.3 reference CSVs next to the executable
set(F16_REF_DIR "${CMAKE_SOURCE_DIR}/data/Atmos_13p3_SubsonicHeadingChangeF16")
foreach(_csv Atmos_13p3_sim_02.csv Atmos_13p3_sim_04.csv Atmos_13p3_sim_05.csv)
  add_custom_command(
    TARGET F16HeadingChange
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${F16_REF_DIR}/${_csv}" "$<TARGET_FILE_DIR:F16HeadingChange>/${_csv}"
    COMMENT "Copying NASA Atmos-13.3 reference ${_csv}")
endforeach()

# Copy the Case-13.3 plotting script next to the executable
add_custom_command(
  TARGET F16HeadingChange
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_SOURCE_DIR}/scripts/plot_f16_s13p3_nasa02.py"
          "$<TARGET_FILE_DIR:F16HeadingChange>/plot_f16_s13p3_nasa02.py"
  COMMENT "Copying Case-13.3 plot script plot_f16_s13p3_nasa02.py")
