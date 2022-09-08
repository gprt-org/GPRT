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

# set(CMAKE_DXC_COMPILER ${Vulkan_BIN_DIR}/dxc)
# set(CMAKE_DXC_COMPILER C:/Users/natevm/git/DirectXShaderCompiler/build/Debug/bin/dxc)
# Ideally we'd use the DXC that comes with Vulkan, but we need vk::RayBufferStore, which is ToT atm...
find_program(CMAKE_DXC_COMPILER dxc 
  DOC "Path to the dxc executable." 
  HINTS ${Vulkan_BIN_DIR}
  NO_DEFAULT_PATH
)

find_program(CMAKE_SPIRV_DISASSEMBLER dxc 
  DOC "Path to the dxc executable." 
  HINTS ${Vulkan_BIN_DIR}
  NO_DEFAULT_PATH
)

# set(CMAKE_SPIRV_OPTIMIZER ${Vulkan_BIN_DIR}/spirv-opt)
# set(CMAKE_SPIRV_ASSEMBLER ${Vulkan_BIN_DIR}/spirv-as)
# set(CMAKE_SPIRV_DISASSEMBLER ${Vulkan_BIN_DIR}/spirv-dis)

if(NOT CMAKE_DXC_COMPILER)
  message(FATAL_ERROR "dxc not found.")
endif()

if (NOT CMAKE_SPIRV_DISASSEMBLER)
  message(FATAL_ERROR "spirv-disc not found.")
endif()

set(EMBED_SPIRV_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")

# dxc.exe -spirv -T lib_6_3 -fspv-extension=SPV_NV_ray_tracing -fspv-extension=SPV_KHR_ray_query -fspv-extension=SPV_KHR_non_semantic_info raytracing.hlsl

function(embed_spirv)
  # processes arguments given to a function, and defines a set of variables 
  #   which hold the values of the respective options
  set(oneArgs OUTPUT_TARGET)
  set(multiArgs SPIRV_LINK_LIBRARIES SOURCES EMBEDDED_SYMBOL_NAMES ENTRY_POINTS)
  cmake_parse_arguments(EMBED_SPIRV "" "${oneArgs}" "${multiArgs}" ${ARGN})

  
  
  
  # list(LENGTH EMBED_SPIRV_ENTRY_POINTS NUM_ENTRY_POINTS)
  # math(EXPR NUM_ENTRY_POINTS_MINUS_ONE "${NUM_ENTRY_POINTS}-1")
  # RDAT_ENUM_VALUE_NODEF(Pixel)
  # RDAT_ENUM_VALUE_NODEF(Vertex)
  # RDAT_ENUM_VALUE_NODEF(Geometry)
  # RDAT_ENUM_VALUE_NODEF(Hull)
  # RDAT_ENUM_VALUE_NODEF(Domain)
  # RDAT_ENUM_VALUE_NODEF(Compute)
  # RDAT_ENUM_VALUE_NODEF(Library)
  # RDAT_ENUM_VALUE_NODEF(RayGeneration)
  # RDAT_ENUM_VALUE_NODEF(Intersection)
  # RDAT_ENUM_VALUE_NODEF(AnyHit)
  # RDAT_ENUM_VALUE_NODEF(ClosestHit)
  # RDAT_ENUM_VALUE_NODEF(Miss)
  # RDAT_ENUM_VALUE_NODEF(Callable)
  # RDAT_ENUM_VALUE_NODEF(Mesh)
  # RDAT_ENUM_VALUE_NODEF(Amplification)

  # "raygeneration", "intersection", "anyhit", "closesthit", "miss", "callable",

  
  unset(EMBED_SPIRV_OUTPUTS)
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
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}_${ENTRY_POINT_TYPE}.spv
      COMMAND ${CMAKE_DXC_COMPILER} 
      -spirv
      -fspv-target-env=vulkan1.1spirv1.4
      -T lib_6_3
      -I ${PROJECT_SOURCE_DIR}/vkrt
      -D VKRT_DEVICE
      -D ${ENTRY_POINT_TYPE}
      -fspv-extension=SPV_KHR_ray_tracing
      -fspv-extension=SPV_KHR_ray_query
      -fspv-extension=SPV_KHR_non_semantic_info 
      -fspv-extension=SPV_KHR_physical_storage_buffer 
      -fcgl
      ${EMBED_SPIRV_SOURCES}
      -Fo ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}_${ENTRY_POINT_TYPE}.spv 
      DEPENDS ${EMBED_SPIRV_SOURCES} ${PROJECT_SOURCE_DIR}/vkrt/vkrt_device.hlsli ${PROJECT_SOURCE_DIR}/vkrt/vkrt.h
      COMMENT "compile SPIRV ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}_${ENTRY_POINT_TYPE}.spv from ${EMBED_SPIRV_SOURCES}"
    )

    list(APPEND EMBED_SPIRV_OUTPUTS ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}_${ENTRY_POINT_TYPE}.spv)
  endforeach()

  # # Dissassemble to allow for readable spirv in build folder...
  # add_custom_command(
  #   OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.spv
  #   COMMAND ${CMAKE_SPIRV_DISASSEMBLER} 
  #   ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}_as.spv
  #   -o ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.spv
  #   DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}_as.spv
  #   COMMENT "disassemble SPIRV ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}_as.spv to ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.spv"
  # )
  
  # Embed spirv files as binary in a .cpp file
  set(EMBED_SPIRV_RUN ${EMBED_SPIRV_DIR}/bin2c.cmake)
  set(EMBED_SPIRV_CPP_FILE ${EMBED_SPIRV_OUTPUT_TARGET}.cpp)
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.cpp
    COMMAND ${CMAKE_COMMAND}
      "-DINPUT_FILES=${EMBED_SPIRV_OUTPUTS}"
      "-DINPUT_NAMES=${ENTRY_POINT_TYPES}"
      "-DOUTPUT_C=${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_CPP_FILE}"
      "-DOUTPUT_VAR=${EMBED_SPIRV_OUTPUT_TARGET}"
      # "-DOUTPUT_H=${EMBED_SPIRV_H_FILE}"
      -P ${EMBED_SPIRV_RUN}
    VERBATIM
    DEPENDS ${EMBED_SPIRV_OUTPUTS}
    COMMENT "Generating embedded SPIRV file: ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}"
  )

  add_library(${EMBED_SPIRV_OUTPUT_TARGET} OBJECT)
  target_sources(${EMBED_SPIRV_OUTPUT_TARGET} PRIVATE ${EMBED_SPIRV_CPP_FILE}) #${EMBED_SPIRV_H_FILE} #${EMBED_SPIRV_SOURCES} 
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
#   set(EMBED_SPIRV_CPP_FILE ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.c)
#   # set(EMBED_SPIRV_H_FILE ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.h)
#   add_custom_command(
#     OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.c
#     COMMAND ${CMAKE_COMMAND}
#       "-DINPUT_FILES=${EMBED_SPIRV_SLANG_OUTPUTS}"
#       "-DOUTPUT_C=${EMBED_SPIRV_CPP_FILE}"
#       # "-DOUTPUT_H=${EMBED_SPIRV_H_FILE}"
#       -P ${EMBED_SPIRV_RUN}
#     VERBATIM
#     DEPENDS ${EMBED_SPIRV_SLANG_OUTPUTS}
#     COMMENT "Generating embedded SPIRV file: ${EMBED_SPIRV_OUTPUT_TARGET}"
#   )

#   add_library(${EMBED_SPIRV_OUTPUT_TARGET} OBJECT)
#   target_sources(${EMBED_SPIRV_OUTPUT_TARGET} PRIVATE ${EMBED_SPIRV_CPP_FILE}) #${EMBED_SPIRV_H_FILE} #${EMBED_SPIRV_SOURCES} 
# endfunction()
