# src/Examples/TwoStageRocket/CMakeLists.txt NASA TM-2015-218675 Atmospheric Scenario 17 — Two-Stage Rocket to Orbit

set(TWOSTAGE_DATA_DIR "${CMAKE_SOURCE_DIR}/data/TwoStageRocket_S119_source")

add_executable(
  TwoStageRocket ${CMAKE_CURRENT_SOURCE_DIR}/TwoStageRocket.cpp ${CMAKE_CURRENT_SOURCE_DIR}/RocketStageModel.cpp
                 ${CMAKE_CURRENT_SOURCE_DIR}/TwoStageRocketSimulator.cpp)

target_compile_features(TwoStageRocket PRIVATE cxx_std_23)

target_compile_options(TwoStageRocket PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /bigobj>)

target_compile_definitions(
  TwoStageRocket
  PRIVATE "TWOSTAGE_INERTIA_FILE=\"${TWOSTAGE_DATA_DIR}/twostage_inertia.dml\""
          "TWOSTAGE_PROP_FILE=\"${TWOSTAGE_DATA_DIR}/twostage_prop.dml\""
          "TWOSTAGE_AERO_FILE=\"${TWOSTAGE_DATA_DIR}/twostage_aero.dml\"")

target_include_directories(TwoStageRocket PRIVATE "${CMAKE_SOURCE_DIR}/vendor/CppAD/include"
                                                  "${CMAKE_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml")

target_link_libraries(TwoStageRocket PRIVATE Aetherion::Aetherion Aetherion::Simulation CppAD::cppad)

# Copy NASA Atmos-17 reference CSVs next to the executable for comparison
set(TWOSTAGE_REF_DIR "${CMAKE_SOURCE_DIR}/data/Atmos_17_TwoStageRocketToOrbit")
foreach(_csv Atmos_17_sim_06.csv)
  add_custom_command(
    TARGET TwoStageRocket
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${TWOSTAGE_REF_DIR}/${_csv}"
            "$<TARGET_FILE_DIR:TwoStageRocket>/${_csv}"
    COMMENT "Copying NASA Atmos-17 reference ${_csv}")
endforeach()
