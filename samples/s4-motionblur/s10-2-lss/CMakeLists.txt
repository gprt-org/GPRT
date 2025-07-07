embed_devicecode(
  OUTPUT_TARGET
    s2_2_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s2_2_lss hostCode.cpp)
target_link_libraries(s2_2_lss
  PRIVATE
    s2_2_deviceCode
    gprt::gprt
)
