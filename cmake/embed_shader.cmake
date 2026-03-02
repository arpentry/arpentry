# embed_shader.cmake — Convert a shader file into a C header with a const char[].
#
# Invoked as a CMake script:
#   cmake -DINPUT=path/to/shader.wgsl -DOUTPUT=path/to/shader.wgsl.h
#         -DVAR_NAME=terrain_wgsl -P embed_shader.cmake

file(READ "${INPUT}" SHADER_SRC)
string(REPLACE "\\" "\\\\" SHADER_SRC "${SHADER_SRC}")
string(REPLACE "\"" "\\\"" SHADER_SRC "${SHADER_SRC}")
string(REPLACE "\n" "\\n\"\n    \"" SHADER_SRC "${SHADER_SRC}")

# Strip trailing empty literal from final newline
string(REGEX REPLACE "\\\\n\"\n    \"$" "\\\\n\"" SHADER_SRC "${SHADER_SRC}")

file(WRITE "${OUTPUT}"
    "/* Auto-generated from ${INPUT_NAME} — do not edit. */\n"
    "static const char *${VAR_NAME} =\n"
    "    \"${SHADER_SRC};\n"
)
