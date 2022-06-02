#ifndef INPUT_PARSING_H
#define INPUT_PARSING_H

#include "ray_tracing.h"

#include <variant>
#include <cstdint>

struct FileInfo {
    Scene scene;
    Image image;
    Camera camera;
    int max_recursion_depth;
};

std::variant<FileInfo, const char*> parse_input_file(const char* const filename);

#endif
