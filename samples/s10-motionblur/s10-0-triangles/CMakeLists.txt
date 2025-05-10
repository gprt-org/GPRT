embed_devicecode(
  OUTPUT_TARGET
    s10_0_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s10_0_triangles hostCode.cpp)
target_link_libraries(s10_0_triangles
  PRIVATE
    s10_0_deviceCode
    gprt::gprt
)
