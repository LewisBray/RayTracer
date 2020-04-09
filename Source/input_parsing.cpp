#include "input_parsing.h"
#include "ray_tracing.h"
#include "maths.h"

#include <algorithm>
#include <optional>
#include <sstream>
#include <fstream>
#include <cassert>
#include <vector>
#include <string>
#include <stack>
#include <regex>

static bool isInt(const std::string& str)
{
    static const std::regex intRegex("[0-9]+");
    return std::regex_match(str.begin(), str.end(), intRegex);
}

static bool isFloatingPoint(const std::string& str)
{
    static const std::regex floatingPointRegex("[-+]?[0-9]*\\.?[0-9]+");
    return std::regex_match(str.begin(), str.end(), floatingPointRegex);
}

struct Command
{
    std::string name;
    std::vector<std::string> params;
};

static Command parseCommand(const std::string& str)
{
    Command command;

    bool firstRead = true;
    std::string token;
    std::stringstream tokenStream{str};
    while (std::getline(tokenStream, token, ' '))
    {
        if (firstRead)
            command.name = token;
        else if (!token.empty())
            command.params.push_back(token);
        
        firstRead = false;
    }

    return command;
}

std::optional<FileInfo> parseInputFile(const char* const filename)
{
    std::ifstream inputFile(filename);
    if (!inputFile.is_open())
        return std::nullopt;
    
    Scene scene;
    int maxRecursionDepth = 5;
    Image image{0, 0, nullptr, "raytrace.png"};
    Camera camera{
        Vector{0.0, 0.0, 0.0, 0.0},
        Vector{0.0, 0.0, 0.0, 0.0},
        Vector{0.0, 0.0, 0.0, 0.0},
        FieldOfView{0.0, 0.0}
    };

    // Shapes
    std::vector<Vector> vertices;
    Matrix currentTransform = identityMatrix();
    Matrix inverseCurrentTransform = identityMatrix();
    std::stack<Matrix, std::vector<Matrix>> transformStack;
    std::stack<Matrix, std::vector<Matrix>> inverseTransformStack;

    // Lighting
    Colour currentAmbient{0.2, 0.2, 0.2};
    scene.directionalLightSource = DirectionalLightSource{
        Vector{0.0, 0.0, 0.0, 1.0},
        Colour{0.0, 0.0, 0.0},
        AttenuationParameters{1.0, 0.0, 0.0}
    };

    // Materials
    Material currentMaterial{
        Colour{0.0, 0.0, 0.0},
        Colour{0.0, 0.0, 0.0},
        Colour{0.0, 0.0, 0.0},
        0.0
    };
    
    std::string line;
    bool firstCommand = true;
    while (std::getline(inputFile, line, '\n'))
    {
        if (line.empty() || line[0] == '#')
            continue;
        
        const Command command = parseCommand(line);
        if (firstCommand && command.name != "size")
            return std::nullopt;
                
        if (command.name == "size")
        {
            if (command.params.size() != 2)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isInt))
                return std::nullopt;
            
            image.width = std::stoi(command.params[0]);
            image.height = std::stoi(command.params[1]);
            image.pixels = new unsigned char[image.width * image.height * 3];
        }
        else if (command.name == "output")
        {
            if (command.params.size() != 1)
                return std::nullopt;
            
            image.filename = command.params[0]; // Do I need to add .png to the end if it isn't there?
        }
        else if (command.name == "maxdepth")
        {
            if (command.params.size() != 1 || !isInt(command.params[0]))
                return std::nullopt;
            
            maxRecursionDepth = std::stoi(command.params.front());
        }
        else if (command.name == "maxverts")
        {
            if (!vertices.empty() || command.params.size() != 1 || !isInt(command.params[0]))
                return std::nullopt;
            
            vertices.reserve(std::stoi(command.params[0]));
        }
        else if (command.name == "camera")
        {
            if (command.params.size() != 10)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isFloatingPoint))
                return std::nullopt;
            
            camera.eye.x = std::stor(command.params[0]);
            camera.eye.y = std::stor(command.params[1]);
            camera.eye.z = std::stor(command.params[2]);
            camera.eye.w = 1.0;

            camera.lookAt.x = std::stor(command.params[3]);
            camera.lookAt.y = std::stor(command.params[4]);
            camera.lookAt.z = std::stor(command.params[5]);
            camera.lookAt.w = 1.0;

            camera.up.x = std::stor(command.params[6]);
            camera.up.y = std::stor(command.params[7]);
            camera.up.z = std::stor(command.params[8]);
            camera.up.w = 1.0;

            assert(!firstCommand);
            camera.fieldOfView.y = std::stor(command.params[9]);
            camera.fieldOfView.x = image.width * camera.fieldOfView.y / image.height;
        }
        else if (command.name == "vertex")
        {
            if (command.params.size() != 3)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isFloatingPoint))
                return std::nullopt;
            
            assert(!firstCommand);
            const real x = std::stor(command.params[0]);
            const real y = std::stor(command.params[1]);
            const real z = std::stor(command.params[2]);
            vertices.emplace_back(Vector{x, y, z, 1.0});
        }
        else if (command.name == "tri")
        {
            if (command.params.size() != 3)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isInt))
                return std::nullopt;
            
            const int indexA = std::stoi(command.params[0]);
            const int indexB = std::stoi(command.params[1]);
            const int indexC = std::stoi(command.params[2]);
            if (std::max({indexA, indexB, indexC}) >= vertices.size())
                return std::nullopt;

            const Vector a = currentTransform * vertices[indexA];
            const Vector b = currentTransform * vertices[indexB];
            const Vector c = currentTransform * vertices[indexC];
            scene.triangles.emplace_back(Triangle{a, b, c});
            scene.triangleAmbients.emplace_back(currentAmbient);
            scene.triangleMaterials.emplace_back(currentMaterial);
        }
        else if (command.name == "sphere")
        {
            if (command.params.size() != 4)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isFloatingPoint))
                return std::nullopt;
            
            const real centreX = std::stor(command.params[0]);
            const real centreY = std::stor(command.params[1]);
            const real centreZ = std::stor(command.params[2]);
            const real radius = std::stor(command.params[3]);
            const Vector centre{centreX, centreY, centreZ, 1.0};

            scene.ellipsoids.emplace_back(Ellipsoid{centre, radius, inverseCurrentTransform});
            scene.ellipsoidTransforms.emplace_back(currentTransform);
            scene.ellipsoidAmbients.emplace_back(currentAmbient);
            scene.ellipsoidMaterials.emplace_back(currentMaterial);
        }
        else if (command.name == "pushTransform")
        {
            if (!command.params.empty())
                return std::nullopt;
            
            transformStack.push(currentTransform);
            inverseTransformStack.push(inverseCurrentTransform);
            currentTransform = identityMatrix();
            inverseCurrentTransform = identityMatrix();
        }
        else if (command.name == "popTransform")
        {
            if (!command.params.empty() || transformStack.empty())
                return std::nullopt;
            
            assert(!inverseTransformStack.empty());
            currentTransform = transformStack.top();
            inverseCurrentTransform = inverseTransformStack.top();
            transformStack.pop();
            inverseTransformStack.pop();
        }
        else if (command.name == "translate")
        {
            if (command.params.size() != 3)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isFloatingPoint))
                return std::nullopt;
            
            const real xOffset = std::stor(command.params[0]);
            const real yOffset = std::stor(command.params[1]);
            const real zOffset = std::stor(command.params[2]);
            const Matrix translation = translationMatrix(xOffset, yOffset, zOffset);
            const Matrix inverseTranslation = translationMatrix(-xOffset, -yOffset, -zOffset);
            currentTransform = currentTransform * translation;
            inverseCurrentTransform = inverseTranslation * inverseCurrentTransform;
        }
        else if (command.name == "scale")
        {
            if (command.params.size() != 3)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isFloatingPoint))
                return std::nullopt;
            
            const real xScale = std::stor(command.params[0]);
            const real yScale = std::stor(command.params[1]);
            const real zScale = std::stor(command.params[2]);
            const Matrix scaling = scalingMatrix(xScale, yScale, zScale);
            const Matrix inverseScaling = scalingMatrix(1.0 / xScale, 1.0 / yScale, 1.0 / zScale);
            currentTransform = currentTransform * scaling;
            inverseCurrentTransform = inverseScaling * inverseCurrentTransform;
        }
        else if (command.name == "rotate")
        {
            if (command.params.size() != 4)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isFloatingPoint))
                return std::nullopt;
            
            const real axisX = std::stor(command.params[0]);
            const real axisY = std::stor(command.params[1]);
            const real axisZ = std::stor(command.params[2]);
            const real angle = std::stor(command.params[3]);
            const Matrix rotation = rotationMatrix(angle, axisX, axisY, axisZ);
            const Matrix inverseRotation = rotationMatrix(-angle, axisX, axisY, axisZ);
            currentTransform = currentTransform * rotation;
            inverseCurrentTransform = inverseRotation * inverseCurrentTransform;
        }
        else if (command.name == "directional")
        {
            if (command.params.size() != 6)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isFloatingPoint))
                return std::nullopt;
            
            scene.directionalLightSource.direction.x = std::stor(command.params[0]);
            scene.directionalLightSource.direction.y = std::stor(command.params[1]);
            scene.directionalLightSource.direction.z = std::stor(command.params[2]);
            scene.directionalLightSource.direction.w = 1.0;

            scene.directionalLightSource.colour.red = std::stor(command.params[3]);
            scene.directionalLightSource.colour.green = std::stor(command.params[4]);
            scene.directionalLightSource.colour.blue = std::stor(command.params[5]);
        }
        else if (command.name == "point")
        {
            if (command.params.size() != 6)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isFloatingPoint))
                return std::nullopt;
            
            const real x = std::stor(command.params[0]);
            const real y = std::stor(command.params[1]);
            const real z = std::stor(command.params[2]);

            const real r = std::stor(command.params[3]);
            const real g = std::stor(command.params[4]);
            const real b = std::stor(command.params[5]);

            const PointLightSource pointLight{
                Vector{x, y, z, 1.0},
                Colour{r, g, b},
                AttenuationParameters{0.0, 0.0, 1.0}
            };

            scene.pointLightSources.emplace_back(pointLight);
        }
        else if (command.name == "attenuation")
        {
            if (command.params.size() != 3)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isFloatingPoint))
                return std::nullopt;
            
            scene.directionalLightSource.attenuationParameters.constant = std::stor(command.params[0]);
            scene.directionalLightSource.attenuationParameters.linear = std::stor(command.params[1]);
            scene.directionalLightSource.attenuationParameters.quadratic = std::stor(command.params[2]);
        }
        else if (command.name == "ambient")
        {
            if (command.params.size() != 3)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isFloatingPoint))
                return std::nullopt;
            
            currentAmbient.red = std::stor(command.params[0]);
            currentAmbient.green = std::stor(command.params[1]);
            currentAmbient.blue = std::stor(command.params[2]);
        }
        else if (command.name == "diffuse")
        {
            if (command.params.size() != 3)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isFloatingPoint))
                return std::nullopt;
            
            currentMaterial.diffuse.red = std::stor(command.params[0]);
            currentMaterial.diffuse.green = std::stor(command.params[1]);
            currentMaterial.diffuse.blue = std::stor(command.params[2]);
        }
        else if (command.name == "specular")
        {
            if (command.params.size() != 3)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isFloatingPoint))
                return std::nullopt;
            
            currentMaterial.specular.red = std::stor(command.params[0]);
            currentMaterial.specular.green = std::stor(command.params[1]);
            currentMaterial.specular.blue = std::stor(command.params[2]);
        }
        else if (command.name == "emission")
        {
            if (command.params.size() != 3)
                return std::nullopt;
            
            if (!std::all_of(command.params.begin(), command.params.end(), isFloatingPoint))
                return std::nullopt;
            
            currentMaterial.emission.red = std::stor(command.params[0]);
            currentMaterial.emission.green = std::stor(command.params[1]);
            currentMaterial.emission.blue = std::stor(command.params[2]);
        }
        else if (command.name == "shininess")
        {
            if (command.params.size() != 1)
                return std::nullopt;
            
            if (!isFloatingPoint(command.params.front()))
                return std::nullopt;
            
            currentMaterial.shininess = std::stor(command.params.front());
        }

        firstCommand = false;
    }

    return FileInfo{image, camera, scene, maxRecursionDepth};
}
