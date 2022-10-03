cmake_minimum_required (VERSION 2.8.12)

# Create header for C file
file(WRITE ${OUTPUT_C} "/* Autogenerated by bin2c */\n\n")
file(APPEND ${OUTPUT_C} "#include <stdint.h>\n")
file(APPEND ${OUTPUT_C} "#include <map>\n")
file(APPEND ${OUTPUT_C} "#include <vector>\n")
file(APPEND ${OUTPUT_C} "#include <string>\n")

message(STATUS "BIN2C: Add the following to your program:")
message(STATUS "\nextern std::map<std::string, std::vector<uint8_t>> ${OUTPUT_VAR};")
file(APPEND ${OUTPUT_C} "std::map<std::string, std::vector<uint8_t>> ${OUTPUT_VAR} = {\n")

# Create header of H file
# file(WRITE ${OUTPUT_H} "/* Autogenerated by bin2c */\n\n")
# file(APPEND ${OUTPUT_H} "#pragma once\n")
# file(APPEND ${OUTPUT_H} "#include <stdint.h>\n\n")

# string(REPLACE "," ";" INPUT_FILES ${INPUT_FILES})

# TODO: Get name of each bin

list(LENGTH INPUT_FILES NUM_INPUTS)
math(EXPR NUM_INPUTS_MINUS_ONE "${NUM_INPUTS}-1")


list(LENGTH INPUT_FILES NUM_INPUT_FILES)
math(EXPR NUM_INPUT_FILES_MINUS_ONE "${NUM_INPUT_FILES}-1")

foreach(idx RANGE ${NUM_INPUT_FILES_MINUS_ONE})
  list(GET INPUT_FILES ${idx} bin)
  list(GET INPUT_NAMES ${idx} entryName)

  # Get short filename
  string(REGEX MATCH "([^/]+)$" filename ${bin})
  # Replace filename spaces & extension separator for C compatibility
  # string(REGEX REPLACE "\\.| |-" "_" filename ${filename})
  string(REGEX REPLACE "\\.| |-" "_" filename ${filename})
  # Convert to lower case
  # string(TOLOWER ${filename} filename)
  # Read hex data from file
  file(READ ${bin} filedata HEX)

  # Convert hex data for C compatibility
  string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," filedata ${filedata})

  # Append data to c file
  # file(APPEND ${OUTPUT_C} "uint8_t ${filename}[] = {${filedata}0x00};\n\n")
  file(APPEND ${OUTPUT_C} "\t{\"${entryName}\", {${filedata}0x00}},\n")
  
  # Append extern definitions to h file
  # file(APPEND ${OUTPUT_H} "extern uint8_t ${filename}[];\n\nextern uint32_t ${filename}_size;\n\n")
  # message(STATUS "\nextern uint8_t ${filename}[];")
endforeach()

# Iterate through binary files files
foreach(bin ${INPUT_FILES})
endforeach()

file(APPEND ${OUTPUT_C} "};\n\n")