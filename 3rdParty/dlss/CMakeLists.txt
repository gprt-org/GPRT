# MIT License

# Copyright (c) 2025 Nathan V. Morrical

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

# Create an imported target for the .lib file
add_library(nvsdk_ngx_d STATIC IMPORTED)
set_target_properties(nvsdk_ngx_d PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/nvsdk_ngx_d.lib"
)

add_library(nvsdk_ngx_d_dbg STATIC IMPORTED)
set_target_properties(nvsdk_ngx_d_dbg PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/nvsdk_ngx_d_dbg.lib"
)

add_library(dlss INTERFACE)

target_include_directories(dlss INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

IF(WIN32)
target_link_libraries(dlss INTERFACE
  $<$<CONFIG:RelWithDebInfo,Release>:${CMAKE_CURRENT_LIST_DIR}/nvsdk_ngx_d.lib>
  $<$<CONFIG:Debug>:${CMAKE_CURRENT_LIST_DIR}/nvsdk_ngx_d_dbg.lib>
)
ELSEIF(LINUX)
target_link_libraries(dlss INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/libnvsdk_ngx.a
)
ELSE()
message(ERROR "Unsupported platform")
ENDIF(WIN32)
