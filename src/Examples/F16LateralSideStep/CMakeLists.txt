# src/Examples/F16LateralSideStep/CMakeLists.txt NASA TM-2015-218675 Atmospheric Scenario 13.4 — F-16 Subsonic Lateral
# Side Step Closed-loop with LQR SAS + autopilot; reuses F16AltitudeChangeSimulator with dynamic latOffset computation.
#
# The simulator tracks the aircraft's lateral deviation from the original 45 deg NE courseline and applies a 2 000 ft
# right-of-course offset at t = 20 s per NASA TM-2015-218675 section B-13.4.
#
# Recommended run command: F16LateralSideStep --endTime 60 --timeStep 0.02 --outputFileName f16_s13p4_sim.csv

set(F16_DATA_DIR "${CMAKE_SOURCE_DIR}/data/F16_S119_source")

add_executable(F16LateralSideStep ${CMAKE_CURRENT_SOURCE_DIR}/F16LateralSideStep.cpp)

target_compile_features(F16LateralSideStep PRIVATE cxx_std_23)

target_compile_options(F16LateralSideStep PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>)

target_compile_definitions(
  F16LateralSideStep
  PRIVATE "F16_INERTIA_FILE=\"${F16_DATA_DIR}/F16_inertia.dml\""
          "F16_AERO_FILE=\"${F16_DATA_DIR}/F16_aero.dml\""
          "F16_PROP_FILE=\"${F16_DATA_DIR}/F16_prop.dml\""
          "F16_CONTROL_FILE=\"${F16_DATA_DIR}/F16_control.dml\"")

target_include_directories(F16LateralSideStep PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include"
                                                      "${CMAKE_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml")

target_link_libraries(F16LateralSideStep PRIVATE Aetherion::Aetherion Aetherion::Simulation CppAD::cppad)

# Copy NASA Atmos-13.4 reference CSVs next to the executable
set(F16_REF_DIR "${CMAKE_SOURCE_DIR}/data/Atmos_13p4_SubsonicLateralSideStepF16")
foreach(_csv Atmos_13p4_sim_02.csv Atmos_13p4_sim_04.csv Atmos_13p4_sim_05.csv)
  add_custom_command(
    TARGET F16LateralSideStep
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${F16_REF_DIR}/${_csv}"
            "$<TARGET_FILE_DIR:F16LateralSideStep>/${_csv}"
    COMMENT "Copying NASA Atmos-13.4 reference ${_csv}")
endforeach()

# Copy the Case-13.4 plotting script next to the executable
add_custom_command(
  TARGET F16LateralSideStep
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_SOURCE_DIR}/scripts/plot_f16_s13p4_nasa02.py"
          "$<TARGET_FILE_DIR:F16LateralSideStep>/plot_f16_s13p4_nasa02.py"
  COMMENT "Copying Case-13.4 plot script plot_f16_s13p4_nasa02.py")
