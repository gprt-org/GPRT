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
message(STATUS "Using module to find Vulkan")
find_package(Vulkan REQUIRED)
if (Vulkan_LIBRARY)
message(STATUS "Found Vulkan version: ${Vulkan_VERSION}")

if(${Vulkan_VERSION} VERSION_LESS 1.4.309)
  message(FATAL_ERROR "Vulkan 1.4.309 or higher is required!")
endif()
endif()

IF(WIN32)
  # IF (NOT Vulkan_FOUND)
  #   find_library(Vulkan_LIBRARY NAMES vulkan-1 vulkan PATHS ${CMAKE_SOURCE_DIR}/libs/vulkan)
  #   IF (Vulkan_LIBRARY)
  #     set(Vulkan_FOUND ON)
  #     MESSAGE("Using bundled Vulkan library version")
  #   ENDIF()
  # ENDIF()
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_USE_PLATFORM_WIN32_KHR")

ELSEIF(LINUX)
  # IF (NOT Vulkan_FOUND)
  #   find_library(Vulkan_LIBRARY NAMES vulkan HINTS "$ENV{VULKAN_SDK}/lib" "${CMAKE_SOURCE_DIR}/libs/vulkan" REQUIRED)
  #   IF (Vulkan_LIBRARY)
  #     set(Vulkan_FOUND ON)
  #     MESSAGE("Using bundled Vulkan library version")
  #   ENDIF()
  # ENDIF()
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

# IF (NOT Vulkan_FOUND)
#   message(FATAL_ERROR "Could not find Vulkan library!")
# ELSE()
  
  # Look for the Slang compiler within the Vulkan bin directory
  # message(STATUS ${Vulkan_LIBRARY})

  # message(STATUS ${Vulkan_GLSLC_EXECUTABLE})

get_filename_component(VULKAN_COMPILER_PATH ${Vulkan_GLSLC_EXECUTABLE} DIRECTORY)
message(STATUS ${VULKAN_COMPILER_PATH})

find_program(CMAKE_SLANG_COMPILER slangc
  HINTS "${VULKAN_COMPILER_PATH}"
  # PATHS /another/path
  # NO_DEFAULT_PATH
)

# find_program(CMAKE_SLANG_COMPILER slangc)
if (CMAKE_SLANG_COMPILER) 
  message("Found slangc: ${CMAKE_SLANG_COMPILER}")

  # Test to see if compiler is working
  execute_process(COMMAND ${CMAKE_SLANG_COMPILER} "-v" 
    OUTPUT_FILE slang_version.txt
    RESULT_VARIABLE SLANG_EXIT_CODE
    ERROR_VARIABLE SLANG_VERSION # Slang seems to output version to stderr, so for now we output that.
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )

  if(NOT SLANG_EXIT_CODE EQUAL 0)
    message(ERROR "Envoking Slang compiler failed with error code: ${SLANG_EXIT_CODE}")
  else()
    message(STATUS "Slang version reported by envoking compiler: ${SLANG_VERSION}" )
  endif()
  
else ()
  message(ERROR " slangc appears to be missing from ${VULKAN_COMPILER_PATH}. Please ensure your Vulkan SDK is updated to 1.3.296 or newer.")
endif()

# ENDIF()

# Find Vulkan Memory Allocator
add_subdirectory(vma)

# Find STB Image
add_subdirectory(stb_image)

# Find Tiny Obj
add_subdirectory(tinyobj)

# Find GLFW
# if(GPRT_BUILD_SAMPLES)
  if(NOT GPRT_USE_INCLUDED_GLFW)
    find_package(glfw3 QUIET)
  endif()
  set(OpenGL_GL_PREFERENCE "LEGACY")
  #find_package(OpenGL QUIET)
  find_package(OpenGL REQUIRED OPTIONAL_COMPONENTS OpenGL)
  if (NOT TARGET glfw)
    if (OpenGL_FOUND)
      message(STATUS "found opengl, building glfw")
      add_subdirectory(glfw)
    endif()
  endif()
# endif()

# Find Imgui
add_library(imgui
  ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui.h
  ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui.cpp

  ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_demo.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_draw.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_widgets.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_tables.cpp

  ${CMAKE_CURRENT_SOURCE_DIR}/imguizmo/GraphEditor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imguizmo/GraphEditor.h

  ${CMAKE_CURRENT_SOURCE_DIR}/imguizmo/ImCurveEdit.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imguizmo/ImCurveEdit.h

  ${CMAKE_CURRENT_SOURCE_DIR}/imguizmo/ImGradient.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imguizmo/ImGradient.h

  ${CMAKE_CURRENT_SOURCE_DIR}/imguizmo/ImGuizmo.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imguizmo/ImGuizmo.h

  ${CMAKE_CURRENT_SOURCE_DIR}/imguizmo/ImSequencer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imguizmo/ImSequencer.h

  ${CMAKE_CURRENT_SOURCE_DIR}/imguizmo/ImZoomSlider.h

  ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends/imgui_impl_vulkan.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends/imgui_impl_glfw.cpp

  ${CMAKE_CURRENT_SOURCE_DIR}/imgui/misc/cpp/imgui_stdlib.h
  ${CMAKE_CURRENT_SOURCE_DIR}/imgui/misc/cpp/imgui_stdlib.cpp

  ${CMAKE_CURRENT_SOURCE_DIR}/implot/implot_items.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/implot/implot.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/implot/implot.h

  ${CMAKE_CURRENT_SOURCE_DIR}/GPRT-SampleAssets/OpenFontIcons.h
)

target_include_directories(imgui PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/imgui
  ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
  ${CMAKE_CURRENT_SOURCE_DIR}/imguizmo
  ${CMAKE_CURRENT_SOURCE_DIR}/implot
  ${CMAKE_CURRENT_SOURCE_DIR}/glfw/include
  ${CMAKE_CURRENT_SOURCE_DIR}/GPRT-SampleAssets/
  ${Vulkan_INCLUDE_DIR}
)

target_link_libraries(${VULKAN_LIBRARY} glfw)

# Find mesh generator
if (GPRT_BUILD_SAMPLES)
add_subdirectory(generator)
endif()

# Find DLSS interface
add_subdirectory(dlss)
