embed_devicecode(
  OUTPUT_TARGET
    s5_2_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s5_2_computeTransform hostCode.cpp)
target_link_libraries(s5_2_computeTransform
  PRIVATE
    s5_2_deviceCode
    gprt::gprt
    generator
)
