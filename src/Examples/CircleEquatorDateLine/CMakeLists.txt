# src/Examples/CircleEquatorDateLine/CMakeLists.txt  NASA TM-2015-218675 Atmospheric Scenario 16 — F-16 CCW
# circumnavigation of the equator/international date line intersection. Closed-loop with LQR SAS + GNC autopilot
# (F16_gnc.dml, circlePoleSW=0.0).
#
# NOTE: the closed-loop LQR is stiff — use timeStep <= 0.02 s for accurate results. Recommended: CircleEquatorDateLine
# --endTime 180 --timeStep 0.02 --outputFileName atmos_16_output.csv

set(F16_DATA_DIR "${CMAKE_SOURCE_DIR}/data/F16_S119_source")

add_executable(CircleEquatorDateLine ${CMAKE_CURRENT_SOURCE_DIR}/CircleEquatorDateLine.cpp)

target_compile_features(CircleEquatorDateLine PRIVATE cxx_std_23)

target_compile_options(CircleEquatorDateLine PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>)

target_compile_definitions(
  CircleEquatorDateLine
  PRIVATE "F16_INERTIA_FILE=\"${F16_DATA_DIR}/F16_inertia.dml\""
          "F16_AERO_FILE=\"${F16_DATA_DIR}/F16_aero.dml\""
          "F16_PROP_FILE=\"${F16_DATA_DIR}/F16_prop.dml\""
          "F16_CONTROL_FILE=\"${F16_DATA_DIR}/F16_gnc.dml\"")

target_include_directories(CircleEquatorDateLine PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include"
                                                         "${CMAKE_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml")

target_link_libraries(CircleEquatorDateLine PRIVATE Aetherion::Aetherion Aetherion::Simulation CppAD::cppad)

# Copy NASA Atmos-16 reference CSVs next to the executable
set(F16_REF_DIR "${CMAKE_SOURCE_DIR}/data/Atmos_16_CircleEquatorDateLine")
foreach(_csv Atmos_16_sim_02.csv Atmos_16_sim_04.csv Atmos_16_sim_05.csv)
  add_custom_command(
    TARGET CircleEquatorDateLine
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${F16_REF_DIR}/${_csv}"
            "$<TARGET_FILE_DIR:CircleEquatorDateLine>/${_csv}"
    COMMENT "Copying NASA Atmos-16 reference ${_csv}")
endforeach()
