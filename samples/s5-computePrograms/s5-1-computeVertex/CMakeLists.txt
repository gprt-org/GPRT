embed_devicecode(
  OUTPUT_TARGET
    s5_1_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s5_1_computeVertex hostCode.cpp)
target_link_libraries(s5_1_computeVertex
  PRIVATE
    s5_1_deviceCode
    gprt::gprt
)
