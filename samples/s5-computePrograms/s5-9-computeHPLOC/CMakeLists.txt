embed_devicecode(
  OUTPUT_TARGET
    s5_9_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)


embed_devicecode(
  OUTPUT_TARGET
    s5_9_hplocBuildDeviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/hplocShared.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/hplocBuild.slang
)

add_executable(s5_9_computeHPLOC hostCode.cpp)
target_link_libraries(s5_9_computeHPLOC
  PRIVATE
    s5_9_hplocBuildDeviceCode
    s5_9_deviceCode
    gprt::gprt
    generator
)
