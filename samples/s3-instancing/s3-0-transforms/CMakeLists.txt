embed_devicecode(
  OUTPUT_TARGET
    s3_0_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s3_0_instances hostCode.cpp)
target_link_libraries(s3_0_instances
  PRIVATE
    s3_0_deviceCode
    gprt::gprt
)
