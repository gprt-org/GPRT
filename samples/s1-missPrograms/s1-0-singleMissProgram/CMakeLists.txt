embed_devicecode(
  OUTPUT_TARGET
    s1_0_deviceCode
  HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/sharedCode.h
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang
)

add_executable(s1_0_singleMissProgram hostCode.cpp)

target_link_libraries(s1_0_singleMissProgram
  PRIVATE
    s1_0_deviceCode
    gprt::gprt
)
