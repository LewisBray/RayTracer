#ifndef INPUT_PARSING_H
#define INPUT_PARSING_H

#include <variant>
#include <cstdint>

#include "ray_tracing.h"

struct FileInfo {
    Scene scene;
    Image image;
    Camera camera;
    int max_recursion_depth;
};

static std::variant<FileInfo, const char*> parse_input_file(const char* const filename);

#endif
