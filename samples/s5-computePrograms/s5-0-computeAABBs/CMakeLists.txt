embed_devicecode(
  OUTPUT_TARGET
    s5_0_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s5_0_computeAABBs hostCode.cpp)
target_link_libraries(s5_0_computeAABBs
  PRIVATE
    s5_0_deviceCode
    gprt::gprt
)
