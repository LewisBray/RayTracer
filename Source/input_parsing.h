#ifndef INPUT_PARSING_H
#define INPUT_PARSING_H

#include "ray_tracing.h"

#include <variant>
#include <cstdint>

struct FileInfo
{
    Image image;
    Camera camera;
    Scene scene;
    int maxRecursionDepth;
};

std::variant<FileInfo, const char*> parseInputFile(const char* const filename);

#endif
