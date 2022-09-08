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


# Find Vulkan
if (NOT CMAKE_VERSION VERSION_LESS 3.7.0)
  message(STATUS "Using module to find Vulkan")
  find_package(Vulkan)
endif()

IF(WIN32)
  IF (NOT Vulkan_FOUND)
    find_library(Vulkan_LIBRARY NAMES vulkan-1 vulkan PATHS ${CMAKE_SOURCE_DIR}/libs/vulkan)
    IF (Vulkan_LIBRARY)
      set(Vulkan_FOUND ON)
      MESSAGE("Using bundled Vulkan library version")
    ENDIF()
  ENDIF()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_USE_PLATFORM_WIN32_KHR")

ELSEIF(LINUX)
  IF (NOT Vulkan_FOUND)
    find_library(Vulkan_LIBRARY NAMES vulkan HINTS "$ENV{VULKAN_SDK}/lib" "${CMAKE_SOURCE_DIR}/libs/vulkan" REQUIRED)
    IF (Vulkan_LIBRARY)
      set(Vulkan_FOUND ON)
      MESSAGE("Using bundled Vulkan library version")
    ENDIF()
  ENDIF()
  find_package(Threads REQUIRED)
  IF(USE_D2D_WSI)
    MESSAGE("Using direct to display extension...")
    add_definitions(-D_DIRECT2DISPLAY)
  ELSEIF(USE_DIRECTFB_WSI)
    find_package(DirectFB REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_USE_PLATFORM_DIRECTFB_EXT")
    include_directories(${DIRECTFB_INCLUDE_DIR})
  ELSEIF(USE_WAYLAND_WSI)
    find_program(PKG_CONFIG pkg-config)
    if (NOT PKG_CONFIG)
      message(FATAL_ERROR "pkg-config binary not found")
    endif ()
    find_package(Wayland REQUIRED)
    if (NOT WAYLAND_FOUND)
      message(FATAL_ERROR "Wayland development package not found")
    endif ()
    pkg_check_modules(WAYLAND_PROTOCOLS REQUIRED wayland-protocols)
    if (NOT WAYLAND_PROTOCOLS_FOUND)
      message(FATAL_ERROR "Wayland protocols package not found")
    endif ()
    find_program(WAYLAND_SCANNER wayland-scanner)
    if (NOT WAYLAND_SCANNER)
      message(FATAL_ERROR "wayland-scanner binary not found")
    endif ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_USE_PLATFORM_WAYLAND_KHR")
    include_directories(${WAYLAND_INCLUDE_DIR})
    pkg_get_variable(protocol_dir wayland-protocols pkgdatadir)
    execute_process(COMMAND ${WAYLAND_SCANNER} client-header ${protocol_dir}/stable/xdg-shell/xdg-shell.xml ${CMAKE_BINARY_DIR}/xdg-shell-client-protocol.h
        COMMAND ${WAYLAND_SCANNER} private-code ${protocol_dir}/stable/xdg-shell/xdg-shell.xml ${CMAKE_BINARY_DIR}/xdg-shell-protocol.c)
    include_directories(${CMAKE_BINARY_DIR})
  ELSEIF(USE_HEADLESS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_USE_PLATFORM_HEADLESS_EXT")
  ELSE(USE_D2D_WSI)
    find_package(XCB REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_USE_PLATFORM_XCB_KHR")
  ENDIF(USE_D2D_WSI)
ELSEIF(APPLE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_USE_PLATFORM_MACOS_MVK -DVK_EXAMPLE_XCODE_GENERATED")
  # Todo : android?
ENDIF(WIN32)

IF (NOT Vulkan_FOUND)
  message(FATAL_ERROR "Could not find Vulkan library!")
ELSE()
  message(STATUS ${Vulkan_LIBRARY})
ENDIF()

# Find STB Image
add_subdirectory(stb_image)

# Find GLFW
if(GPRT_BUILD_SAMPLES)
  find_package(glfw3 QUIET)
  set(OpenGL_GL_PREFERENCE "LEGACY")
  #find_package(OpenGL QUIET)
  find_package(OpenGL REQUIRED OPTIONAL_COMPONENTS OpenGL)
  if (NOT TARGET glfw)
    if (OpenGL_FOUND)
      message(STATUS "found opengl, building glfw")
      add_subdirectory(glfw)
    endif()
  endif()
endif()

# SPIRV tools
set(SPIRV_TOOLS_BUILD_STATIC ON CACHE BOOL
  "Build ${SPIRV_TOOLS}-static target. ${SPIRV_TOOLS} will alias to ${SPIRV_TOOLS}-static or ${SPIRV_TOOLS}-shared based on BUILD_SHARED_LIBS" 
  FORCE)
set(SPIRV_SKIP_EXECUTABLES ON CACHE BOOL
  "Skip building the executable and tests along with the library"
  FORCE)
set(SPIRV_SKIP_TESTS ON CACHE BOOL
  "Skip building tests along with the library" 
  FORCE)
add_subdirectory(SPIRV-Tools EXCLUDE_FROM_ALL)

