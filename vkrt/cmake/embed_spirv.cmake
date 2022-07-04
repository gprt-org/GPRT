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
set(EMBED_SPIRV_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")

function(embed_spirv)
  set(oneArgs OUTPUT_TARGET SPIRV_TARGET)
  set(multiArgs SPIRV_LINK_LIBRARIES SOURCES EMBEDDED_SYMBOL_NAMES)
  cmake_parse_arguments(EMBED_SPIRV "" "${oneArgs}" "${multiArgs}" ${ARGN})

#   if (EMBED_SPIRV_EMBEDDED_SYMBOL_NAMES)
#     list(LENGTH EMBED_SPIRV_EMBEDDED_SYMBOL_NAMES NUM_NAMES)
#     list(LENGTH EMBED_SPIRV_SOURCES NUM_SOURCES)
#     if (NOT ${NUM_SOURCES} EQUAL ${NUM_NAMES})
#       message(FATAL_ERROR
#         "embed_ptx(): the number of names passed as EMBEDDED_SYMBOL_NAMES must \
#         match the number of files in SOURCES."
#       )
#     endif()
#   else()
#     unset(EMBED_SPIRV_EMBEDDED_SYMBOL_NAMES)
#     foreach(source ${EMBED_SPIRV_SOURCES})
#       get_filename_component(name ${source} NAME_WE)
#       list(APPEND EMBED_SPIRV_EMBEDDED_SYMBOL_NAMES ${name}_ptx)
#     endforeach()
#   endif()

#   ## Find bin2c and CMake script to feed it ##

#   # We need to wrap bin2c with a script for multiple reasons:
#   #   1. bin2c only converts a single file at a time
#   #   2. bin2c has only standard out support, so we have to manually redirect to
#   #      a cmake buffer
#   #   3. We want to pack everything into a single output file, so we need to use
#   #      the --name option

#   get_filename_component(CUDA_COMPILER_BIN "${CMAKE_CUDA_COMPILER}" DIRECTORY)
#   find_program(BIN_TO_C NAMES bin2c PATHS ${CUDA_COMPILER_BIN})
#   if(NOT BIN_TO_C)
#     message(FATAL_ERROR
#       "bin2c not found:\n"
#       "  CMAKE_CUDA_COMPILER='${CMAKE_CUDA_COMPILER}'\n"
#       "  CUDA_COMPILER_BIN='${CUDA_COMPILER_BIN}'\n"
#       )
#   endif()

#   set(EMBED_SPIRV_RUN ${EMBED_SPIRV_DIR}/run_bin2c.cmake)

#   ## Create SPIRV object target ##

#   if (NOT EMBED_SPIRV_SPIRV_TARGET)
#     set(SPIRV_TARGET ${EMBED_SPIRV_OUTPUT_TARGET}_ptx)
#   else()
#     set(SPIRV_TARGET ${EMBED_SPIRV_SPIRV_TARGET})
#   endif()

#   add_library(${SPIRV_TARGET} OBJECT)
#   target_sources(${SPIRV_TARGET} PRIVATE ${EMBED_SPIRV_SOURCES})
#   target_link_libraries(${SPIRV_TARGET} PRIVATE ${EMBED_SPIRV_SPIRV_LINK_LIBRARIES})
#   set_property(TARGET ${SPIRV_TARGET} PROPERTY CUDA_SPIRV_COMPILATION ON)
#   set_property(TARGET ${SPIRV_TARGET} PROPERTY CUDA_ARCHITECTURES OFF)
#   target_compile_options(${SPIRV_TARGET} PRIVATE "-lineinfo")

#   ## Create command to run the bin2c via the CMake script ##

#   set(EMBED_SPIRV_C_FILE ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_SPIRV_OUTPUT_TARGET}.c)
#   get_filename_component(OUTPUT_FILE_NAME ${EMBED_SPIRV_C_FILE} NAME)
#   add_custom_command(
#     OUTPUT ${EMBED_SPIRV_C_FILE}
#     COMMAND ${CMAKE_COMMAND}
#       "-DBIN_TO_C_COMMAND=${BIN_TO_C}"
#       "-DOBJECTS=$<TARGET_OBJECTS:${SPIRV_TARGET}>"
#       "-DSYMBOL_NAMES=${EMBED_SPIRV_EMBEDDED_SYMBOL_NAMES}"
#       "-DOUTPUT=${EMBED_SPIRV_C_FILE}"
#       -P ${EMBED_SPIRV_RUN}
#     VERBATIM
#     DEPENDS $<TARGET_OBJECTS:${SPIRV_TARGET}> ${SPIRV_TARGET}
#     COMMENT "Generating embedded SPIRV file: ${OUTPUT_FILE_NAME}"
#   )

#   add_library(${EMBED_SPIRV_OUTPUT_TARGET} OBJECT)
#   target_sources(${EMBED_SPIRV_OUTPUT_TARGET} PRIVATE ${EMBED_SPIRV_C_FILE})
endfunction()
