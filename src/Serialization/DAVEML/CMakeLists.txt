# src/Serialization/DAVEML/CMakeLists.txt
#
# Minimal DAVE-ML reader — variableDef initial values + simple MathML arithmetic.
# Uses pugixml (vendored via Janus ThirdParty, MIT license).

set(PUGIXML_DIR "${CMAKE_SOURCE_DIR}/vendor/Janus/ThirdParty/pugixml")

target_sources(Aetherion PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/DAVEMLReader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/LoadInertiaFromDAVEML.cpp
    ${PUGIXML_DIR}/pugixml.cpp         # compile pugixml into Aetherion
)

target_include_directories(Aetherion
    PUBLIC
        ${CMAKE_SOURCE_DIR}/Aetherion   # already set, but safe to repeat
    PRIVATE
        ${PUGIXML_DIR}                  # pugixml.hpp visible to .cpp files
)
