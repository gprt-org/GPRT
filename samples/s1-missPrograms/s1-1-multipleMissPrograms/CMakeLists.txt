embed_devicecode(
  OUTPUT_TARGET
    s1_1_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s1_1_multipleMissPrograms hostCode.cpp)

target_link_libraries(s1_1_multipleMissPrograms
  PRIVATE
    s1_1_deviceCode
    gprt::gprt
)
