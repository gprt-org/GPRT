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

add_library(gprt)

include(cmake/hlsl_embed_spirv.cmake)

embed_spirv(
  OUTPUT_TARGET
    gprtDeviceCode
  ENTRY_POINTS
    gprtFillInstanceData
  SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/gprt_device_code.hlsl
)

target_sources(gprt PRIVATE
    gprt.cpp
    gprt.h
)

set_target_properties(gprt
    PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# IF(WIN32)
# 	# Nothing here (yet)
# ELSEIF(APPLE)
# 	target_link_libraries(gprt ${Vulkan_LIBRARY} "-framework AppKit" "-framework QuartzCore")
# ELSE(WIN32)
# 	target_link_libraries(gprt
#         ${XCB_LIBRARIES}
#         ${Vulkan_LIBRARY}
#         ${Vulkan_LIBRARY}
#         ${DIRECTFB_LIBRARIES}
#         ${WAYLAND_CLIENT_LIBRARIES}
#         ${CMAKE_THREAD_LIBS_INIT}
#     )
# ENDIF(WIN32)

target_link_libraries(gprt 
    PUBLIC 
    ${Vulkan_LIBRARY} 
    gprtDeviceCode
)

set_target_properties(gprt PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(gprt
  PUBLIC
    ${Vulkan_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}
)

add_library(gprt::gprt ALIAS gprt)