# MIT License

# Copyright (c) 2022 Nathan V. Morrical

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.12)

find_program(CMAKE_DXC_COMPILER dxc
  DOC "Path to the dxc executable."
  HINTS ${Vulkan_INCLUDE_DIR}/../bin/ ${Vulkan_INCLUDE_DIR}/../Bin/ ${DXC_BIN_DIR}
)

# set(CMAKE_SPIRV_OPTIMIZER ${Vulkan_BIN_DIR}/spirv-opt)
# set(CMAKE_SPIRV_ASSEMBLER ${Vulkan_BIN_DIR}/spirv-as)
# set(CMAKE_SPIRV_DISASSEMBLER ${Vulkan_BIN_DIR}/spirv-dis)

if(NOT CMAKE_DXC_COMPILER)
  message(FATAL_ERROR "dxc not found.")
endif()

set(EMBED_DEVICECODE_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")

function(embed_devicecode)
  # processes arguments given to a function, and defines a set of variables
  #   which hold the values of the respective options
  set(oneArgs OUTPUT_TARGET)
  set(multiArgs SPIRV_LINK_LIBRARIES SOURCES EMBEDDED_SYMBOL_NAMES ENTRY_POINTS)
  cmake_parse_arguments(EMBED_DEVICECODE "" "${oneArgs}" "${multiArgs}" ${ARGN})

  unset(EMBED_DEVICECODE_OUTPUTS)
  set(ENTRY_POINT_TYPES
    "RAYGEN"
    "INTERSECTION"
    "ANYHIT"
    "CLOSESTHIT"
    "MISS"
    "CALLABLE"
    "COMPUTE"
  )
  list(LENGTH ENTRY_POINT_TYPES NUM_ENTRY_POINT_TYPES)
  math(EXPR NUM_ENTRY_POINT_TYPES_MINUS_ONE "${NUM_ENTRY_POINT_TYPES}-1")

  foreach(idx RANGE ${NUM_ENTRY_POINT_TYPES_MINUS_ONE})

    list(GET ENTRY_POINT_TYPES ${idx} ENTRY_POINT_TYPE)

    
    # Compile hlsl to SPIRV
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_DEVICECODE_OUTPUT_TARGET}_${ENTRY_POINT_TYPE}.spv
      COMMAND ${CMAKE_DXC_COMPILER}
      -HV 2021
      -enable-16bit-types
      -spirv
      -fspv-target-env=vulkan1.1spirv1.4
      -HV 2021
      -T lib_6_3
      -I ${GPRT_INCLUDE_DIR}
      -D GPRT_DEVICE
      -D ${ENTRY_POINT_TYPE}
      -fspv-extension=SPV_KHR_ray_tracing
      -fspv-extension=SPV_KHR_ray_query
      -fspv-extension=SPV_KHR_non_semantic_info
      -fspv-extension=SPV_KHR_physical_storage_buffer
      -fspv-extension=SPV_KHR_vulkan_memory_model
      -fspv-extension=SPV_EXT_descriptor_indexing
      -fcgl
      ${EMBED_DEVICECODE_SOURCES}
      -Fo ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_DEVICECODE_OUTPUT_TARGET}_${ENTRY_POINT_TYPE}.spv
      DEPENDS ${EMBED_DEVICECODE_SOURCES} ${GPRT_INCLUDE_DIR}/gprt_device.h ${GPRT_INCLUDE_DIR}/gprt.h
    )

    list(APPEND EMBED_DEVICECODE_OUTPUTS ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_DEVICECODE_OUTPUT_TARGET}_${ENTRY_POINT_TYPE}.spv)
  endforeach()

  # Embed spirv files as binary in a .cpp file
  set(EMBED_DEVICECODE_RUN ${EMBED_DEVICECODE_DIR}/bin2c.cmake)
  set(EMBED_DEVICECODE_CPP_FILE ${EMBED_DEVICECODE_OUTPUT_TARGET}.cpp)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_DEVICECODE_OUTPUT_TARGET}.cpp
    COMMAND ${CMAKE_COMMAND}
      "-DINPUT_FILES=${EMBED_DEVICECODE_OUTPUTS}"
      "-DINPUT_NAMES=${ENTRY_POINT_TYPES}"
      "-DOUTPUT_C=${CMAKE_CURRENT_BINARY_DIR}/${EMBED_DEVICECODE_CPP_FILE}"
      "-DOUTPUT_VAR=${EMBED_DEVICECODE_OUTPUT_TARGET}"
      # "-DOUTPUT_H=${EMBED_DEVICECODE_H_FILE}"
      -P ${EMBED_DEVICECODE_RUN}
    VERBATIM
    DEPENDS ${EMBED_DEVICECODE_OUTPUTS}
    )
  
  add_library(${EMBED_DEVICECODE_OUTPUT_TARGET} OBJECT)
  target_sources(${EMBED_DEVICECODE_OUTPUT_TARGET} PRIVATE ${EMBED_DEVICECODE_CPP_FILE}) #${EMBED_DEVICECODE_H_FILE} #${EMBED_DEVICECODE_SOURCES}
endfunction()
