embed_devicecode(
  OUTPUT_TARGET
    s3_1_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s3_1_visibilityMasks hostCode.cpp)
target_link_libraries(s3_1_visibilityMasks
  PRIVATE
    s3_1_deviceCode
    gprt::gprt
)
