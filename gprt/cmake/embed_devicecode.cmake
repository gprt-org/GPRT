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

include(FetchContent)

find_program(CMAKE_SLANG_COMPILER slangc)

if (CMAKE_SLANG_COMPILER)
message("Found slangc: ${CMAKE_SLANG_COMPILER}")
else ()
  message("Slang compiler not defined...")
  message("Downloading Slang Compiler...")
  if (WIN32)
    FetchContent_Declare(SlangCompiler
      URL https://github.com/shader-slang/slang/releases/download/v2023.5.4/slang-2023.5.4-win64.zip
      URL_HASH SHA256=48201c552c32787212d4afd436bc452dc7d19ecef958a41c5036e9fcb9410a9c
      DOWNLOAD_NO_EXTRACT true
      DOWNLOAD_DIR "${CMAKE_BINARY_DIR}/"
    )
    FetchContent_Populate(SlangCompiler)
    message("Extracting...")
    execute_process(COMMAND powershell Expand-Archive -Force -Path "${CMAKE_BINARY_DIR}/slang-2023.5.4-win64.zip" -DestinationPath "${CMAKE_BINARY_DIR}/slang-2023.5.4-win64" RESULT_VARIABLE EXTRACT_RESULT)
    if(NOT EXTRACT_RESULT EQUAL 0)
        message(FATAL_ERROR "Extraction failed with error code: ${EXTRACT_RESULT}")
    else()
        message("Done.")
    endif()
    set(CMAKE_SLANG_COMPILER "${CMAKE_BINARY_DIR}/slang-2023.5.4-win64/bin/windows-x64/release/slangc.exe" CACHE INTERNAL "CMAKE_SLANG_COMPILER")
  else() # linux
    FetchContent_Declare(HLSLCompiler
      URL https://github.com/shader-slang/slang/releases/download/v2023.5.4/slang-2023.5.4-linux-x86_64.tar.gz
      URL_HASH SHA256=55f2329bfc91d21b8af43006daf078dd77c0041373cf4264953187bedf685f99
      DOWNLOAD_NO_EXTRACT true
      DOWNLOAD_DIR "${CMAKE_BINARY_DIR}/"
    )
    FetchContent_Populate(HLSLCompiler)
    message("Extracting...")
    execute_process(COMMAND mkdir -p "${CMAKE_BINARY_DIR}/slang-2023.5.4-linux-x86_64" RESULT_VARIABLE EXTRACT_RESULT)
    if(NOT EXTRACT_RESULT EQUAL 0)
        message(FATAL_ERROR "mkdir failed with error code: ${EXTRACT_RESULT}")
    endif()
    execute_process(COMMAND tar xzf "${CMAKE_BINARY_DIR}/slang-2023.5.4-linux-x86_64.tar.gz" -C "${CMAKE_BINARY_DIR}/slang-2023.5.4-linux-x86_64" RESULT_VARIABLE EXTRACT_RESULT)
    if(NOT EXTRACT_RESULT EQUAL 0)
        message(FATAL_ERROR "tar failed with error code: ${EXTRACT_RESULT}")
    else()
        message("Done.")
    endif()
    execute_process(COMMAND chmod +x "${CMAKE_BINARY_DIR}/slang-2023.5.4-linux-x86_64/bin/linux-x64/release/slangc")
    set(CMAKE_SLANG_COMPILER "${CMAKE_BINARY_DIR}/slang-2023.5.4-linux-x86_64/bin/linux-x64/release/slangc" CACHE INTERNAL "CMAKE_SLANG_COMPILER")
  endif()

  # Test to see if compiler is working
  execute_process(COMMAND ${CMAKE_SLANG_COMPILER} "-v" RESULT_VARIABLE EXTRACT_RESULT)
  if(NOT EXTRACT_RESULT EQUAL 0)
  message("Envoking Slang compiler failed with error code: ${EXTRACT_RESULT}")
  endif()
endif()

set(EMBED_DEVICECODE_DIR ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")

function(embed_devicecode)
  # processes arguments given to a function, and defines a set of variables
  #   which hold the values of the respective options
  set(oneArgs OUTPUT_TARGET)
  set(multiArgs SPIRV_LINK_LIBRARIES SOURCES HEADERS)
  cmake_parse_arguments(EMBED_DEVICECODE "" "${oneArgs}" "${multiArgs}" ${ARGN})

  unset(EMBED_DEVICECODE_OUTPUTS)
  set(ENTRY_POINT_TYPES "SLANG")

  add_custom_command(
  # add_custom_target(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_DEVICECODE_OUTPUT_TARGET}.spv
    COMMAND ${CMAKE_SLANG_COMPILER}
    ${EMBED_DEVICECODE_SOURCES}
    -profile sm_6_7
    -target spirv -emit-spirv-directly
    -force-glsl-scalar-layout
    -fvk-use-entrypoint-name
    -matrix-layout-row-major
    -I ${GPRT_INCLUDE_DIR}
    -o ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_DEVICECODE_OUTPUT_TARGET}.spv
    DEPENDS ${EMBED_DEVICECODE_SOURCES} ${EMBED_DEVICECODE_HEADERS} ${GPRT_INCLUDE_DIR}/gprt.slangh ${GPRT_INCLUDE_DIR}/gprt.h
  )

  list(APPEND EMBED_DEVICECODE_OUTPUTS ${CMAKE_CURRENT_BINARY_DIR}/${EMBED_DEVICECODE_OUTPUT_TARGET}.spv)

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