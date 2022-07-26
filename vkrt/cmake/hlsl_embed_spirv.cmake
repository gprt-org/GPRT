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

set(Vulkan_BIN_DIR ${Vulkan_INCLUDE_DIR}/../Bin)

set(CMAKE_DXC_COMPILER ${Vulkan_BIN_DIR}/dxc)

# find_program(CMAKE_DXC_COMPILER dxc DOC "Path to the dxc executable." 
    # HINTS ${Vulkan_BIN_DIR})
if(NOT CMAKE_DXC_COMPILER)
  message(FATAL_ERROR "dxc not found.")
endif()

set(EMBED_SPIRV_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")

# dxc.exe -spirv -T lib_6_3 -fspv-extension=SPV_NV_ray_tracing -fspv-extension=SPV_KHR_ray_query -fspv-extension=SPV_KHR_non_semantic_info raytracing.hlsl

function(embed_spirv)
  # processes arguments given to a function, and defines a set of variables 
  #   which hold the values of the respective options
  set(oneArgs OUTPUT_TARGET)
  set(multiArgs SPIRV_LINK_LIBRARIES SOURCES EMBEDDED_SYMBOL_NAMES ENTRY_POINTS)
  cmake_parse_arguments(EMBED_SPIRV "" "${oneArgs}" "${multiArgs}" ${ARGN})

#   list(LENGTH EMBED_SPIRV_ENTRY_POINTS NUM_ENTRY_POINTS)
  
  # Compile entry point to SPIRV
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.spv
    COMMAND ${CMAKE_DXC_COMPILER} 
    -spirv
    -fspv-target-env=vulkan1.1spirv1.4
    -T lib_6_3
    -I ${PROJECT_SOURCE_DIR}/vkrt
    -D VKRT_DEVICE
    -fspv-extension=SPV_KHR_ray_tracing 
    -fspv-extension=SPV_KHR_ray_query 
    -fspv-extension=SPV_KHR_non_semantic_info
    ${EMBED_SPIRV_SOURCES} 
    > ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.spv 
    # -Fo ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.spv 
    DEPENDS ${EMBED_SPIRV_SOURCES}
    COMMENT "compile SPIRV ${EMBED_SPIRV_OUTPUT_TARGET} from ${EMBED_SPIRV_SOURCES}"
  )


#   # For every entry point, compile the shader into SPIR.
#   # Store the resulting spirv files in a list
#   unset(EMBED_SPIRV_SLANG_OUTPUTS)
#   math(EXPR NUM_ENTRY_POINTS_MINUS_ONE "${NUM_ENTRY_POINTS}-1")
#   foreach(idx RANGE ${NUM_ENTRY_POINTS_MINUS_ONE})
#     list(GET EMBED_SPIRV_ENTRY_POINTS ${idx} EMBED_SPIRV_ENTRY_POINT)
#     set(SPIRV_TARGET ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_ENTRY_POINT}.spv)
#     list(APPEND EMBED_SPIRV_SLANG_OUTPUTS ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_ENTRY_POINT}.spv)
#   endforeach()

  # Embed all spirv files as binary in a .c file
  set(EMBED_SPIRV_RUN ${EMBED_SPIRV_DIR}/bin2c.cmake)
  set(EMBED_SPIRV_C_FILE ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.c)
  # set(EMBED_SPIRV_H_FILE ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.h)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.c
    COMMAND ${CMAKE_COMMAND}
      "-DINPUT_FILES=${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.spv"
      "-DOUTPUT_C=${EMBED_SPIRV_C_FILE}"
      # "-DOUTPUT_H=${EMBED_SPIRV_H_FILE}"
      -P ${EMBED_SPIRV_RUN}
    VERBATIM
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.spv
    COMMENT "Generating embedded SPIRV file: ${EMBED_SPIRV_OUTPUT_TARGET}"
  )

  add_library(${EMBED_SPIRV_OUTPUT_TARGET} OBJECT)
  target_sources(${EMBED_SPIRV_OUTPUT_TARGET} PRIVATE ${EMBED_SPIRV_C_FILE}) #${EMBED_SPIRV_H_FILE} #${EMBED_SPIRV_SOURCES} 
endfunction()




# # MIT License

# # Copyright (c) 2022 Nathan V. Morrical

# # Permission is hereby granted, free of charge, to any person obtaining a copy
# # of this software and associated documentation files (the "Software"), to deal
# # in the Software without restriction, including without limitation the rights
# # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# # copies of the Software, and to permit persons to whom the Software is
# # furnished to do so, subject to the following conditions:

# # The above copyright notice and this permission notice shall be included in all
# # copies or substantial portions of the Software.

# # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# # SOFTWARE.

# cmake_minimum_required(VERSION 3.12)

# set(Vulkan_BIN_DIR ${Vulkan_INCLUDE_DIR}/../Bin)

# set(CMAKE_DXC_COMPILER ${Vulkan_BIN_DIR}/dxc)

# # find_program(CMAKE_DXC_COMPILER dxc DOC "Path to the dxc executable." 
#     # HINTS ${Vulkan_BIN_DIR})
# if(NOT CMAKE_DXC_COMPILER)
#   message(FATAL_ERROR "dxc not found.")
# endif()

# set(EMBED_SPIRV_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")

# # dxc.exe -spirv -T lib_6_3 -fspv-extension=SPV_NV_ray_tracing -fspv-extension=SPV_KHR_ray_query -fspv-extension=SPV_KHR_non_semantic_info raytracing.hlsl

# function(embed_spirv)
#   # processes arguments given to a function, and defines a set of variables 
#   #   which hold the values of the respective options
#   set(oneArgs OUTPUT_TARGET)
#   set(multiArgs SPIRV_LINK_LIBRARIES SOURCES EMBEDDED_SYMBOL_NAMES ENTRY_POINTS)
#   cmake_parse_arguments(EMBED_SPIRV "" "${oneArgs}" "${multiArgs}" ${ARGN})

#   list(LENGTH EMBED_SPIRV_ENTRY_POINTS NUM_ENTRY_POINTS)

#   # For every entry point, compile the shader into SPIR.
#   # Store the resulting spirv files in a list
#   unset(EMBED_SPIRV_SLANG_OUTPUTS)
#   math(EXPR NUM_ENTRY_POINTS_MINUS_ONE "${NUM_ENTRY_POINTS}-1")
#   foreach(idx RANGE ${NUM_ENTRY_POINTS_MINUS_ONE})
#     list(GET EMBED_SPIRV_ENTRY_POINTS ${idx} EMBED_SPIRV_ENTRY_POINT)
#     set(SPIRV_TARGET ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_ENTRY_POINT}.spv)

#     # Compile entry point to SPIRV
#     add_custom_command(
#         OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_ENTRY_POINT}.spv
#         COMMAND ${CMAKE_DXC_COMPILER} 
#         -spirv
#         -fspv-target-env=vulkan1.1spirv1.4
#         -T lib_6_3
#         -fspv-extension=SPV_KHR_ray_tracing 
#         -fspv-extension=SPV_KHR_ray_query 
#         -fspv-extension=SPV_KHR_non_semantic_info
#         # -E ${EMBED_SPIRV_ENTRY_POINT}
#         ${EMBED_SPIRV_SOURCES} 
#         #-Fo ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_ENTRY_POINT}.spv 
#         DEPENDS ${EMBED_SPIRV_SOURCES}
#         COMMENT "compile SPIRV ${EMBED_SPIRV_ENTRY_POINT} from ${EMBED_SPIRV_SOURCES}"
#     )

#     list(APPEND EMBED_SPIRV_SLANG_OUTPUTS ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_ENTRY_POINT}.spv)
#   endforeach()

#   # Embed all spirv files as binary in a .c file
#   set(EMBED_SPIRV_RUN ${EMBED_SPIRV_DIR}/bin2c.cmake)
#   set(EMBED_SPIRV_C_FILE ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.c)
#   # set(EMBED_SPIRV_H_FILE ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.h)
#   add_custom_command(
#     OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.c
#     COMMAND ${CMAKE_COMMAND}
#       "-DINPUT_FILES=${EMBED_SPIRV_SLANG_OUTPUTS}"
#       "-DOUTPUT_C=${EMBED_SPIRV_C_FILE}"
#       # "-DOUTPUT_H=${EMBED_SPIRV_H_FILE}"
#       -P ${EMBED_SPIRV_RUN}
#     VERBATIM
#     DEPENDS ${EMBED_SPIRV_SLANG_OUTPUTS}
#     COMMENT "Generating embedded SPIRV file: ${EMBED_SPIRV_OUTPUT_TARGET}"
#   )

#   add_library(${EMBED_SPIRV_OUTPUT_TARGET} OBJECT)
#   target_sources(${EMBED_SPIRV_OUTPUT_TARGET} PRIVATE ${EMBED_SPIRV_C_FILE}) #${EMBED_SPIRV_H_FILE} #${EMBED_SPIRV_SOURCES} 
# endfunction()
