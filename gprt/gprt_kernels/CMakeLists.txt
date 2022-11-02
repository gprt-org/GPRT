embed_devicecode(
  OUTPUT_TARGET
    gprtDeviceCode
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/gprt_device_code.hlsl
)

embed_devicecode(
  OUTPUT_TARGET
    gprtRadixSortDeviceCode
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/ParallelSortCS.hlsl
)
