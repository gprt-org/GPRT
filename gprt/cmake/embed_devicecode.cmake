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

set(EMBED_DEVICECODE_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")

function(embed_devicecode)
  # processes arguments given to a function, and defines a set of variables
  #   which hold the values of the respective options
  set(oneArgs OUTPUT_TARGET)
  set(multiArgs SPIRV_LINK_LIBRARIES SOURCES HEADERS)
  cmake_parse_arguments(EMBED_DEVICECODE "" "${oneArgs}" "${multiArgs}" ${ARGN})

  unset(EMBED_DEVICECODE_OUTPUTS)

  set(EMBED_DEVICECODE_DEBUG_OPT_FLAG $<$<CONFIG:Debug>:-O0>)
  set(EMBED_DEVICECODE_RELEASE_OPT_FLAG $<$<CONFIG:RelWithDebInfo,Release>:-O3>)

  set(EMBED_DEVICECODE_DEBUG_DEFINES $<$<CONFIG:Debug>:-D__DEBUG__>)

  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_DEVICECODE_OUTPUT_TARGET}.spv
    COMMAND ${CMAKE_SLANG_COMPILER}
    ${EMBED_DEVICECODE_SOURCES}
    -profile sm_6_7
    -target spirv
    -emit-spirv-directly
    -fspv-reflect
    -force-glsl-scalar-layout
    -fvk-use-entrypoint-name
    -matrix-layout-row-major
    -ignore-capabilities
    # -zero-initialize # zero-initialize all variables # seems to no longer be an option
    -Wno-39001 # for VK_EXT_mutable_descriptor_type, allows overlapping bindings
    -fp-mode fast
    # -g3
    ${EMBED_DEVICECODE_DEBUG_OPT_FLAG}
    ${EMBED_DEVICECODE_RELEASE_OPT_FLAG}
    -I ${GPRT_INCLUDE_DIR}
    ${EMBED_DEVICECODE_DEBUG_DEFINES}
    -o ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_DEVICECODE_OUTPUT_TARGET}.spv
    DEPENDS ${EMBED_DEVICECODE_SOURCES} ${EMBED_DEVICECODE_HEADERS} ${GPRT_INCLUDE_DIR}/gprt.slang ${GPRT_INCLUDE_DIR}/gprt.h
  )

  list(APPEND EMBED_DEVICECODE_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_DEVICECODE_OUTPUT_TARGET}.spv)

  # Embed spirv as binary in a .cpp file
  set(EMBED_DEVICECODE_RUN ${EMBED_DEVICECODE_DIR}/bin2c.cmake)
  set(EMBED_DEVICECODE_CPP_FILE ${EMBED_DEVICECODE_OUTPUT_TARGET}.cpp)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_DEVICECODE_OUTPUT_TARGET}.cpp
    COMMAND ${CMAKE_COMMAND}
      "-DINPUT_FILE=${EMBED_DEVICECODE_OUTPUT}"
      "-DOUTPUT_C=${CMAKE_CURRENT_BINARY_DIR}/${EMBED_DEVICECODE_CPP_FILE}"
      "-DOUTPUT_VAR=${EMBED_DEVICECODE_OUTPUT_TARGET}"
      -P ${EMBED_DEVICECODE_RUN}
    VERBATIM
    DEPENDS ${EMBED_DEVICECODE_OUTPUT}
    )

  add_library(${EMBED_DEVICECODE_OUTPUT_TARGET} OBJECT)
  target_sources(${EMBED_DEVICECODE_OUTPUT_TARGET} PRIVATE ${EMBED_DEVICECODE_CPP_FILE}) #${EMBED_DEVICECODE_H_FILE} #${EMBED_DEVICECODE_SOURCES}
endfunction()