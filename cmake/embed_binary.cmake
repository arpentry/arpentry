# embed_binary.cmake — Convert a binary file into a C header.
#
# Uses xxd -i for fast hex conversion, then wraps with proper variable names.
#
# Invoked as a CMake script:
#   cmake -DINPUT=path/to/file.ttf -DOUTPUT=path/to/file.ttf.h
#         -DVAR_NAME=inter_regular_ttf -P embed_binary.cmake

file(SIZE "${INPUT}" FILE_SIZE)

execute_process(
    COMMAND xxd -i "${INPUT}"
    OUTPUT_VARIABLE XXD_OUTPUT
    RESULT_VARIABLE XXD_RESULT
)

if(NOT XXD_RESULT EQUAL 0)
    message(FATAL_ERROR "xxd failed on ${INPUT}")
endif()

# xxd -i produces: unsigned char path_to_file[] = { ... };
# Extract just the hex data between the braces
string(REGEX MATCH "\\{([^}]*)\\}" _ "${XXD_OUTPUT}")
set(HEX_DATA "${CMAKE_MATCH_1}")

file(WRITE "${OUTPUT}"
    "/* Auto-generated from ${INPUT_NAME} — do not edit. */\n"
    "#include <stdint.h>\n"
    "static const uint8_t ${VAR_NAME}[] = {${HEX_DATA}};\n"
    "static const unsigned int ${VAR_NAME}_len = ${FILE_SIZE};\n"
)
