cmake_minimum_required(VERSION 2.8.12)

message("Executing command to generate documentation")
execute_process(
    COMMAND make html
    WORKING_DIRECTORY ${DOC_SOURCE_DIR}
    RESULT_VARIABLE RESULT
    OUTPUT_VARIABLE OUTPUT
    ERROR_VARIABLE ERROR
)

if (${RESULT} EQUAL 2)
message("Error: Please verify that python, sphinx, sphinx_rtd_theme, and breathe packages are installed to build documentation")
# else()
endif()

if (RESULT)
message(${RESULT})
endif()

if (OUTPUT)
message(${OUTPUT})
endif()

if (ERROR)
message(${ERROR})
endif()

