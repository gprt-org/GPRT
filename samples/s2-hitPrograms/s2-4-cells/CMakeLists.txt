embed_devicecode(
  OUTPUT_TARGET
    s2_4_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../gprt/math/cellInterpolators.slang
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s2_4_cells hostCode.cpp)
target_link_libraries(s2_4_cells
  PRIVATE
    s2_4_deviceCode
    gprt::gprt
)
target_compile_definitions(s2_4_cells
  PRIVATE ASSETS_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/../../../3rdParty/GPRT-SampleAssets/"
)
