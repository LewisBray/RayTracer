#include "input_parsing.h"
#include "ray_tracing.h"
#include "maths.h"

#include <algorithm>
#include <variant>
#include <sstream>
#include <fstream>
#include <cassert>
#include <vector>
#include <string>
#include <stack>
#include <regex>

static bool isPositiveInt(const std::string& str)
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

    std::string token;
    std::stringstream tokenStream{str};
    while (std::getline(tokenStream, token, ' '))
    {
        if (token.empty())
            continue;

        if (command.name.empty())
            command.name = token;
        else
            command.params.push_back(token);
    }

    return command;
}

std::variant<FileInfo, const char*> parseInputFile(const char* const filename)
{
    std::ifstream inputFile(filename);
    if (!inputFile.is_open())
        return "Failed to open input file.";
    
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
    scene.directionalLightSource = std::nullopt;

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
            return "First command should be 'size'.";
                
        if (command.name == "size")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 2 || !std::all_of(params.begin(), params.end(), isPositiveInt))
                return "'size' command should have 2 positive integer parameters.";
                        
            image.width = std::stoi(params[0]);
            image.height = std::stoi(params[1]);
            image.pixels = new unsigned char[image.width * image.height * 3];
        }
        else if (command.name == "output")
        {
            if (command.params.size() != 1)
                return "'output' command should have 1 parameter.";
            
            image.filename = command.params[0]; // Do I need to add .png to the end if it isn't there?
        }
        else if (command.name == "maxdepth")
        {
            if (command.params.size() != 1 || !isPositiveInt(command.params[0]))
                return "'maxdepth' command should have 1 positive integer parameter.";
            
            maxRecursionDepth = std::stoi(command.params.front());
        }
        else if (command.name == "maxverts")
        {
            if (!vertices.empty())
                return "'maxverts' should be specified before vertices are specified.";

            if (command.params.size() != 1 || !isPositiveInt(command.params[0]))
                return "'maxverts' command should have 1 positive integer parameter.";
            
            vertices.reserve(std::stoi(command.params[0]));
        }
        else if (command.name == "camera")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 10 || !std::all_of(params.begin(), params.end(), isFloatingPoint))
                return "'camera' command should have 10 floating point parameters.";
            
            camera.eye.x = std::stor(params[0]);
            camera.eye.y = std::stor(params[1]);
            camera.eye.z = std::stor(params[2]);
            camera.eye.w = 1.0;

            camera.lookAt.x = std::stor(params[3]);
            camera.lookAt.y = std::stor(params[4]);
            camera.lookAt.z = std::stor(params[5]);
            camera.lookAt.w = 1.0;

            camera.up.x = std::stor(params[6]);
            camera.up.y = std::stor(params[7]);
            camera.up.z = std::stor(params[8]);
            camera.up.w = 1.0;

            assert(!firstCommand);
            camera.fieldOfView.y = std::stor(command.params[9]);
            camera.fieldOfView.x = image.width * camera.fieldOfView.y / image.height;
        }
        else if (command.name == "vertex")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), isFloatingPoint))
                return "'vertex' command should have 3 floating point parameters.";
            
            assert(!firstCommand);
            const real x = std::stor(params[0]);
            const real y = std::stor(params[1]);
            const real z = std::stor(params[2]);
            vertices.emplace_back(Vector{x, y, z, 1.0});
        }
        else if (command.name == "tri")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), isPositiveInt))
                return "'tri' command should have 3 positive integer parameters.";
            
            const int indexA = std::stoi(params[0]);
            const int indexB = std::stoi(params[1]);
            const int indexC = std::stoi(params[2]);
            if (std::max({indexA, indexB, indexC}) >= vertices.size())
                return "Vertex index specified in 'tri' command is beyond the number of specified vertices.";

            const Vector a = currentTransform * vertices[indexA];
            const Vector b = currentTransform * vertices[indexB];
            const Vector c = currentTransform * vertices[indexC];
            scene.triangles.emplace_back(Triangle{a, b, c});
            scene.triangleAmbients.emplace_back(currentAmbient);
            scene.triangleMaterials.emplace_back(currentMaterial);
        }
        else if (command.name == "sphere")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 4 || !std::all_of(params.begin(), params.end(), isFloatingPoint))
                return "'sphere' command should have 3 floating point parameters.";
            
            const real centreX = std::stor(params[0]);
            const real centreY = std::stor(params[1]);
            const real centreZ = std::stor(params[2]);
            const real radius = std::stor(params[3]);
            const Vector centre{centreX, centreY, centreZ, 1.0};

            scene.ellipsoids.emplace_back(Ellipsoid{centre, radius, inverseCurrentTransform});
            scene.ellipsoidTransforms.emplace_back(currentTransform);
            scene.ellipsoidAmbients.emplace_back(currentAmbient);
            scene.ellipsoidMaterials.emplace_back(currentMaterial);
        }
        else if (command.name == "pushTransform")
        {
            if (!command.params.empty())
                return "'pushTransform' command does not take any parameters.";
            
            transformStack.push(currentTransform);
            inverseTransformStack.push(inverseCurrentTransform);
            currentTransform = identityMatrix();
            inverseCurrentTransform = identityMatrix();
        }
        else if (command.name == "popTransform")
        {
            if (!command.params.empty())
                return "'popTransform' command does not take any parameters";
            
            if (transformStack.empty())
                return "Cannot perform 'popTransform' cas there are no transforms on the stack.";

            assert(!inverseTransformStack.empty());
            currentTransform = transformStack.top();
            inverseCurrentTransform = inverseTransformStack.top();
            transformStack.pop();
            inverseTransformStack.pop();
        }
        else if (command.name == "translate")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), isFloatingPoint))
                return "'translate' command should have 3 floating point parameters.";
            
            const real xOffset = std::stor(params[0]);
            const real yOffset = std::stor(params[1]);
            const real zOffset = std::stor(params[2]);
            const Matrix translation = translationMatrix(xOffset, yOffset, zOffset);
            const Matrix inverseTranslation = translationMatrix(-xOffset, -yOffset, -zOffset);
            currentTransform = currentTransform * translation;
            inverseCurrentTransform = inverseTranslation * inverseCurrentTransform;
        }
        else if (command.name == "scale")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), isFloatingPoint))
                return "'scale' command should have 3 floating point arguments.";
            
            const real xScale = std::stor(params[0]);
            const real yScale = std::stor(params[1]);
            const real zScale = std::stor(params[2]);
            const Matrix scaling = scalingMatrix(xScale, yScale, zScale);
            const Matrix inverseScaling = scalingMatrix(1.0 / xScale, 1.0 / yScale, 1.0 / zScale);
            currentTransform = currentTransform * scaling;
            inverseCurrentTransform = inverseScaling * inverseCurrentTransform;
        }
        else if (command.name == "rotate")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 4 || !std::all_of(params.begin(), params.end(), isFloatingPoint))
                return "'rotate' command should have 4 floating point arguments.";
            
            const real axisX = std::stor(params[0]);
            const real axisY = std::stor(params[1]);
            const real axisZ = std::stor(params[2]);
            const real angle = std::stor(params[3]);
            const Matrix rotation = rotationMatrix(angle, axisX, axisY, axisZ);
            const Matrix inverseRotation = rotationMatrix(-angle, axisX, axisY, axisZ);
            currentTransform = currentTransform * rotation;
            inverseCurrentTransform = inverseRotation * inverseCurrentTransform;
        }
        else if (command.name == "directional")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 6 || !std::all_of(params.begin(), params.end(), isFloatingPoint))
                return "'directional' command should have 6 floating point parameters.";
            
            if (!scene.directionalLightSource.has_value())
            {
                scene.directionalLightSource = DirectionalLightSource {
                    Vector{0.0, 0.0, 0.0, 1.0},
                    Colour{0.0, 0.0, 0.0},
                    AttenuationParameters{1.0, 0.0, 0.0}
                };
            }

            scene.directionalLightSource.value().direction.x = std::stor(params[0]);
            scene.directionalLightSource.value().direction.y = std::stor(params[1]);
            scene.directionalLightSource.value().direction.z = std::stor(params[2]);
            scene.directionalLightSource.value().direction.w = 1.0;

            scene.directionalLightSource.value().colour.red = std::stor(params[3]);
            scene.directionalLightSource.value().colour.green = std::stor(params[4]);
            scene.directionalLightSource.value().colour.blue = std::stor(params[5]);
        }
        else if (command.name == "point")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 6 || !std::all_of(params.begin(), params.end(), isFloatingPoint))
                return "'point' command should have 6 floating point parameters.";
            
            const real x = std::stor(params[0]);
            const real y = std::stor(params[1]);
            const real z = std::stor(params[2]);

            const real r = std::stor(params[3]);
            const real g = std::stor(params[4]);
            const real b = std::stor(params[5]);

            const PointLightSource pointLight{
                Vector{x, y, z, 1.0},
                Colour{r, g, b},
                AttenuationParameters{0.0, 0.0, 1.0}
            };

            scene.pointLightSources.emplace_back(pointLight);
        }
        else if (command.name == "attenuation")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), isFloatingPoint))
                return "'attenuation' command should have 3 floating point arguments.";
            
            if (!scene.directionalLightSource.has_value())
            {
                scene.directionalLightSource = DirectionalLightSource {
                    Vector{0.0, 0.0, 0.0, 1.0},
                    Colour{0.0, 0.0, 0.0},
                    AttenuationParameters{1.0, 0.0, 0.0}
                };
            }

            scene.directionalLightSource.value().attenuationParameters.constant = std::stor(params[0]);
            scene.directionalLightSource.value().attenuationParameters.linear = std::stor(params[1]);
            scene.directionalLightSource.value().attenuationParameters.quadratic = std::stor(params[2]);
        }
        else if (command.name == "ambient")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), isFloatingPoint))
                return "'ambient' command should have 3 floating point arguments.";
            
            currentAmbient.red = std::stor(params[0]);
            currentAmbient.green = std::stor(params[1]);
            currentAmbient.blue = std::stor(params[2]);
        }
        else if (command.name == "diffuse")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), isFloatingPoint))
                return "'diffuse' command should have 3 floating point arguments.";
            
            currentMaterial.diffuse.red = std::stor(params[0]);
            currentMaterial.diffuse.green = std::stor(params[1]);
            currentMaterial.diffuse.blue = std::stor(params[2]);
        }
        else if (command.name == "specular")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), isFloatingPoint))
                return "'specular' command should have 3 floating point arguments.";
            
            currentMaterial.specular.red = std::stor(params[0]);
            currentMaterial.specular.green = std::stor(params[1]);
            currentMaterial.specular.blue = std::stor(params[2]);
        }
        else if (command.name == "emission")
        {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), isFloatingPoint))
                return "'emission' command should have 3 floating point arguments.";
            
            currentMaterial.emission.red = std::stor(params[0]);
            currentMaterial.emission.green = std::stor(params[1]);
            currentMaterial.emission.blue = std::stor(params[2]);
        }
        else if (command.name == "shininess")
        {
            if (command.params.size() != 1 || !isFloatingPoint(command.params.front()))
                return "'shininess' command should have 1 floating point argument.";
            
            currentMaterial.shininess = std::stor(command.params.front());
        }
        else
        {
            return "Unknown command entered.";
        }

        firstCommand = false;
    }

    return FileInfo{image, camera, scene, maxRecursionDepth};
}
