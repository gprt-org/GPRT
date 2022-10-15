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

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.18)
  cmake_policy(SET CMP0104 NEW)
endif()

if (NOT (${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR}))
  set(GPRT_IS_SUBPROJECT ON)
else()
  set(GPRT_IS_SUBPROJECT OFF)
endif()

set(CMAKE_BUILD_TYPE_INIT "Release")

# project command is required to come after cmake_minimum_required command.
project(GPRT VERSION 1.0.0 LANGUAGES C CXX)

if (NOT GPRT_IS_SUBPROJECT)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
endif()

# ------------------------------------------------------------------
# Configuration Options
# ------------------------------------------------------------------
option(GPRT_BUILD_SHARED "Build GPRT as a shared library? (otherwise static)" OFF)
set(BUILD_SHARED_LIBS ${GPRT_BUILD_SHARED}) # use 'GPRT_' naming convention

option(GPRT_BUILD_SAMPLES "Build the Samples?" ON)

# ------------------------------------------------------------------
# first, include some dependencies
# ------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/gprt/cmake/")
if (GPRT_IS_SUBPROJECT)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} PARENT_SCOPE)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (GPRT_BUILD_SHARED)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

option(OWL_BUILD_SAMPLES "Build the Samples?" ON)

add_subdirectory(3rdParty)


# ------------------------------------------------------------------
# gprt library itself
# ------------------------------------------------------------------
add_subdirectory(gprt)

# ------------------------------------------------------------------
# tutorial/samples
# ------------------------------------------------------------------
if (GPRT_BUILD_SAMPLES)
  add_subdirectory(samples)
endif()

if (GPRT_IS_SUBPROJECT)
  return()
endif()

# include(CTest)
# if (BUILD_TESTING)
#   enable_testing()
# endif()

# # ------------------------------------------------------------------
# # some basic testing
# # ------------------------------------------------------------------
# if (GPRT_BUILD_ADVANCED_TESTS)
#   add_subdirectory(tests)
# endif()
