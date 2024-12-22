embed_devicecode(
  OUTPUT_TARGET
    s0_0_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s0_0_singleRayGen hostCode.cpp)

target_link_libraries(s0_0_singleRayGen
  PRIVATE
    s0_0_deviceCode
    gprt::gprt
)
