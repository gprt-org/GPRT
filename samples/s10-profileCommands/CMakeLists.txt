embed_devicecode(OUTPUT_TARGET profileDeviceCode
  SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/deviceCode.slang)
add_executable(s10_0_profileCommands hostCode.cpp)
target_link_libraries(s10_0_profileCommands PRIVATE profileDeviceCode gprt::gprt)
