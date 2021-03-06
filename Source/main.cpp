#include "./FreeImage/Source/FreeImage.h"

#include "input_parsing.h"
#include "ray_tracing.h"
#include "maths.h"

#include <iostream>
#include <sstream>
#include <variant>

int main(const int argc, const char* const argv[])
{
    if (argc < 2)
    {
        std::cout << "Specify an input file." << std::endl;
        return -1;
    }
    
    const std::variant<FileInfo, const char*> fileParsingResult = parseInputFile(argv[1]);
    if (std::holds_alternative<const char*>(fileParsingResult))
    {
        const char* const errorMessage = std::get<const char*>(fileParsingResult);
        std::cout << "Failed to parse input file.  " << errorMessage << std::endl;
        return -1;
    }

    const FileInfo& fileInfo = std::get<FileInfo>(fileParsingResult);
    std::cout << "File info:" << std::endl;
    const Image& image = fileInfo.image;
    std::cout
        << "Image: \n\tfilename - " << image.filename
        << "\n\twidth - " << image.width
        << "\n\theight - " << image.height
        << std::endl;
    
    const Camera& camera = fileInfo.camera;
    std::cout
        << "Camera: \n\teye - " << camera.eye
        << "\n\tlook at - " << camera.lookAt
        << "\n\tup - " << camera.up
        << "\n\tfield of view - (" << camera.fieldOfView.x << ", " << camera.fieldOfView.y << ')'
        << std::endl;
    
    constexpr auto toString = [](const Colour& colour) {
        std::stringstream ss;
        ss << '(' << colour.red << ", " << colour.green << ", " << colour.blue << ')';
        return ss.str();
    };
    
    const Scene& scene = fileInfo.scene;
    std::cout << "Triangles:\n";
    for (std::size_t i = 0; i < scene.triangles.size(); ++i)
    {
        const Triangle& triangle = scene.triangles[i];
        const Colour& ambient = scene.triangleAmbients[i];
        std::cout << '\t' << triangle.a << ", " << triangle.b << ", " << triangle.c << ": " << toString(ambient) << std::endl;
    }
    std::cout << "Ellipsoids:\n";
    for (std::size_t i = 0; i < scene.ellipsoids.size(); ++i)
    {
        const Sphere& sphere = scene.ellipsoids[i].sphere;
        const Colour& ambient = scene.ellipsoidAmbients[i];
        std::cout << '\t' << sphere.centre << ", " << sphere.radius << ": " << toString(ambient) << std::endl;
    }
    
    const int maxRecursionDepth = fileInfo.maxRecursionDepth;
    std::cout << "Max recursion depth:\n\t" << maxRecursionDepth << std::endl;


    // const Triangle triangle{{0.5, -0.5, 0.0, 1.0}, {0.0, 0.5, 0.0, 1.0}, {-0.5, -0.5, 0.0, 1.0}};
    // const Ray ray{{0.0, 0.0, 5.0, 1.0}, {0.0, 0.05, 3.0, 1.0}};
    // const auto intersection = intersect(ray, triangle);
    // if (intersection.has_value())
    //     std::cout << intersection.value() << std::endl;
    // else
    //     std::cout << "No intersection" << std::endl;

    for (int y = 0; y < image.height; ++y)
    {
        for (int x = 0; x < image.width; ++x)
        {
            const Ray ray = rayThroughPixel(camera, x, y, image);
            const Colour colour = intersect(ray, scene, camera.eye, maxRecursionDepth);
            unsigned char* const pixel = image.pixels + 3 * x + 3 * y * image.width;
            pixel[0] = static_cast<unsigned char>(colour.blue * 255);
            pixel[1] = static_cast<unsigned char>(colour.green * 255);
            pixel[2] = static_cast<unsigned char>(colour.red * 255);
        }
    }

    FreeImage_Initialise();
    FIBITMAP* bitmap = FreeImage_ConvertFromRawBits(image.pixels, image.width,
        image.height, image.width * 3, 8 * 3, 0xFF0000, 0x00FF00, 0x0000FF, true);
    FreeImage_Save(FIF_PNG, bitmap, image.filename.c_str(), 0);
    FreeImage_DeInitialise();
    
    return 0;
}
