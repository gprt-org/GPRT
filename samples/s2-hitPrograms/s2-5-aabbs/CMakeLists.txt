embed_devicecode(
  OUTPUT_TARGET
    s2_5_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s2_5_aabbs hostCode.cpp)
target_link_libraries(s2_5_aabbs
  PRIVATE
    s2_5_deviceCode
    gprt::gprt
)
