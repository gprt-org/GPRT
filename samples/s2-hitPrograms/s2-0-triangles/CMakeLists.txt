embed_devicecode(
  OUTPUT_TARGET
    s2_0_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s2_0_triangles hostCode.cpp)
target_link_libraries(s2_0_triangles
  PRIVATE
    s2_0_deviceCode
    gprt::gprt
)
