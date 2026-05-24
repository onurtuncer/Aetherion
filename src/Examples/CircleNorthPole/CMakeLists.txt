# src/Examples/CircleNorthPole/CMakeLists.txt NASA TM-2015-218675 Atmospheric Scenario 15 — F-16 North Pole
# Circumnavigation Closed-loop with LQR SAS + GNC autopilot (F16_gnc.dml, selectCircumnavigator=1.0).
#
# NOTE: the closed-loop LQR is stiff — use timeStep <= 0.02 s for accurate results. Recommended run command:
# CircleNorthPole --endTime 180 --timeStep 0.02 --outputFileName atmos_15_output.csv

set(F16_DATA_DIR "${CMAKE_SOURCE_DIR}/data/F16_S119_source")

add_executable(CircleNorthPole ${CMAKE_CURRENT_SOURCE_DIR}/CircleNorthPole.cpp)

target_compile_features(CircleNorthPole PRIVATE cxx_std_23)

target_compile_options(CircleNorthPole PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>)

target_compile_definitions(
  CircleNorthPole
  PRIVATE "F16_INERTIA_FILE=\"${F16_DATA_DIR}/F16_inertia.dml\""
          "F16_AERO_FILE=\"${F16_DATA_DIR}/F16_aero.dml\""
          "F16_PROP_FILE=\"${F16_DATA_DIR}/F16_prop.dml\""
          "F16_CONTROL_FILE=\"${F16_DATA_DIR}/F16_gnc.dml\"")

target_include_directories(CircleNorthPole PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include"
                                                   "${CMAKE_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml")

target_link_libraries(CircleNorthPole PRIVATE Aetherion::Aetherion Aetherion::Simulation CppAD::cppad)

# Copy NASA Atmos-15 reference CSVs next to the executable
set(F16_REF_DIR "${CMAKE_SOURCE_DIR}/data/Atmos_15_CircleNorthPole")
foreach(_csv Atmos_15_sim_02.csv Atmos_15_sim_04.csv Atmos_15_sim_05.csv)
  add_custom_command(
    TARGET CircleNorthPole
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${F16_REF_DIR}/${_csv}" "$<TARGET_FILE_DIR:CircleNorthPole>/${_csv}"
    COMMENT "Copying NASA Atmos-15 reference ${_csv}")
endforeach()
