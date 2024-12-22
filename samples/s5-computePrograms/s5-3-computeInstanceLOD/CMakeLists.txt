embed_devicecode(
  OUTPUT_TARGET
    s5_3_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s5_3_computeInstanceLOD hostCode.cpp)
target_link_libraries(s5_3_computeInstanceLOD
  PRIVATE
    s5_3_deviceCode
    gprt::gprt
    generator
)
