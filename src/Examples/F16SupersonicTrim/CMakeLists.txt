# src/Examples/F16SupersonicTrim/CMakeLists.txt NASA TM-2015-218675 Atmospheric Check-Case 12 — F-16 Supersonic Trim
# Open-loop steady straight-and-level flight at Mach ≈ 2.01, 30 013 ft. Reuses F16SteadyFlightSimulator; no closed-loop
# controller.
#
# Recommended run command: F16SupersonicTrim --endTime 200 --timeStep 0.05 --outputFileName f16_s12_sim.csv

set(F16_DATA_DIR "${CMAKE_SOURCE_DIR}/data/F16_S119_source")

add_executable(F16SupersonicTrim ${CMAKE_CURRENT_SOURCE_DIR}/F16SupersonicTrim.cpp)

target_compile_features(F16SupersonicTrim PRIVATE cxx_std_23)

target_compile_options(F16SupersonicTrim PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>)

target_compile_definitions(
  F16SupersonicTrim
  PRIVATE "F16_INERTIA_FILE=\"${F16_DATA_DIR}/F16_inertia.dml\"" "F16_AERO_FILE=\"${F16_DATA_DIR}/F16_aero.dml\""
          "F16_PROP_FILE=\"${F16_DATA_DIR}/F16_prop.dml\"")

target_include_directories(F16SupersonicTrim PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include"
                                                     "${CMAKE_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml")

target_link_libraries(F16SupersonicTrim PRIVATE Aetherion::Aetherion Aetherion::Simulation CppAD::cppad)

# Copy NASA Atmos-12 reference CSVs next to the executable
set(F16_REF_DIR "${CMAKE_SOURCE_DIR}/data/Atmos_12_TrimCheckSupersonicF16")
foreach(_csv Atmos_12_sim_02.csv Atmos_12_sim_04.csv Atmos_12_sim_05.csv)
  add_custom_command(
    TARGET F16SupersonicTrim
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${F16_REF_DIR}/${_csv}"
            "$<TARGET_FILE_DIR:F16SupersonicTrim>/${_csv}"
    COMMENT "Copying NASA Atmos-12 reference ${_csv}")
endforeach()

# Copy the Case-12 plotting script next to the executable
add_custom_command(
  TARGET F16SupersonicTrim
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_SOURCE_DIR}/scripts/plot_f16_s12_nasa02.py"
          "$<TARGET_FILE_DIR:F16SupersonicTrim>/plot_f16_s12_nasa02.py"
  COMMENT "Copying Case-12 plot script plot_f16_s12_nasa02.py")
