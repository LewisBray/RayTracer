#ifndef INPUT_PARSING_H
#define INPUT_PARSING_H

#include "ray_tracing.h"

#include <optional>
#include <cstdint>
#include <string>

struct FileInfo
{
    Image image;
    Camera camera;
    Scene scene;
    int maxRecursionDepth;
};

std::optional<FileInfo> parseInputFile(const char* const filename);

#endif
