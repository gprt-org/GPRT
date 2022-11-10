cmake_minimum_required(VERSION 2.6)
project(generator)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(TARGET_NAME generator)
add_subdirectory(src)
target_include_directories(generator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/generator)
set_property(TARGET generator PROPERTY POSITION_INDEPENDENT_CODE TRUE)
