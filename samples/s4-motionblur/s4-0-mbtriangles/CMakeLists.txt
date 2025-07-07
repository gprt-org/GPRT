embed_devicecode(
  OUTPUT_TARGET
    s4_0_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s4_0_mbtriangles hostCode.cpp)
target_link_libraries(s4_0_mbtriangles
  PRIVATE
    s4_0_deviceCode
    gprt::gprt
)
