embed_devicecode(
  OUTPUT_TARGET
    s2_6_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s2_6_dptriangles hostCode.cpp)
target_link_libraries(s2_6_dptriangles
  PRIVATE
    s2_6_deviceCode
    gprt::gprt
)
