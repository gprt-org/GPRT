embed_devicecode(
  OUTPUT_TARGET
    s3_2_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s3_2_multipleGeometry hostCode.cpp)
target_link_libraries(s3_2_multipleGeometry
  PRIVATE
    s3_2_deviceCode
    gprt::gprt
    generator
)
