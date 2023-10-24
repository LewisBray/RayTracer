#ifndef INPUT_PARSING_H
#define INPUT_PARSING_H

#include <cstdint>

#include "ray_tracing.h"

struct FileInfo {
    Scene scene;
    Image image;
    Camera camera;
    int max_recursion_depth;
};

struct ParseInputFileResult {
    FileInfo file_info;
    const char* error;
};

static ParseInputFileResult parse_input_file(const char* const filename);

#endif
