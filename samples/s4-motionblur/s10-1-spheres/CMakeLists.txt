embed_devicecode(
  OUTPUT_TARGET
    s2_1_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s2_1_spheres hostCode.cpp)
target_link_libraries(s2_1_spheres
  PRIVATE
    s2_1_deviceCode
    gprt::gprt
)
