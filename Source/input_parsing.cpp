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

static bool is_positive_int(const std::string& str) {
    static const std::regex int_regex("[0-9]+");
    return std::regex_match(str.begin(), str.end(), int_regex);
}

static bool is_floating_point(const std::string& str) {
    static const std::regex floating_point_regex("[-+]?[0-9]*\\.?[0-9]+");
    return std::regex_match(str.begin(), str.end(), floating_point_regex);
}

struct Command {
    std::string name;
    std::vector<std::string> params;
};

static Command parse_command(const std::string& str) {
    Command command;

    std::string token;
    std::stringstream token_stream{str};
    while (std::getline(token_stream, token, ' '))
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

std::variant<FileInfo, const char*> parse_input_file(const char* const filename) {
    std::ifstream input_file(filename);
    if (!input_file.is_open()) {
        return "Failed to open input file.";
    }
    
    Scene scene;
    int max_recursion_depth = 5;
    Image image{0, 0, nullptr, "raytrace.png"};
    Camera camera{
        Vector{0.0, 0.0, 0.0, 0.0},
        Vector{0.0, 0.0, 0.0, 0.0},
        Vector{0.0, 0.0, 0.0, 0.0},
        FieldOfView{0.0, 0.0}
    };

    // Shapes
    std::vector<Vector> vertices;
    Matrix current_transform = identity_matrix();
    Matrix inverse_current_transform = identity_matrix();
    std::stack<Matrix, std::vector<Matrix>> transform_stack;
    std::stack<Matrix, std::vector<Matrix>> inverse_transform_stack;

    // Lighting
    Colour current_ambient{0.2, 0.2, 0.2};
    scene.directional_light_source = std::nullopt;
    scene.attenuation_parameters = AttenuationParameters{1.0, 0.0, 0.0};

    // Materials
    Material current_material{
        Colour{0.0, 0.0, 0.0},
        Colour{0.0, 0.0, 0.0},
        Colour{0.0, 0.0, 0.0},
        0.0
    };
    
    std::string line;
    bool first_command = true;
    while (std::getline(input_file, line, '\n')) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        const Command command = parse_command(line);
        if (first_command && command.name != "size") {
            return "First command should be 'size'.";
        }
                
        if (command.name == "size") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 2 || !std::all_of(params.begin(), params.end(), is_positive_int)) {
                return "'size' command should have 2 positive integer parameters.";
            }
                        
            image.width = std::stoi(params[0]);
            image.height = std::stoi(params[1]);
            image.pixels = new unsigned char[image.width * image.height * 3];
        } else if (command.name == "output") {
            if (command.params.size() != 1) {
                return "'output' command should have 1 parameter.";
            }
            
            image.filename = command.params[0]; // Do I need to add .png to the end if it isn't there?
        } else if (command.name == "maxdepth") {
            if (command.params.size() != 1 || !is_positive_int(command.params[0])) {
                return "'maxdepth' command should have 1 positive integer parameter.";
            }
            
            max_recursion_depth = std::stoi(command.params.front());
        } else if (command.name == "maxverts") {
            if (!vertices.empty()) {
                return "'maxverts' should be specified before vertices are specified.";
            }

            if (command.params.size() != 1 || !is_positive_int(command.params[0])) {
                return "'maxverts' command should have 1 positive integer parameter.";
            }
            
            vertices.reserve(std::stoi(command.params[0]));
        } else if (command.name == "camera") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 10 || !std::all_of(params.begin(), params.end(), is_floating_point)) {
                return "'camera' command should have 10 floating point parameters.";
            }
            
            camera.eye.x = std::stor(params[0]);
            camera.eye.y = std::stor(params[1]);
            camera.eye.z = std::stor(params[2]);
            camera.eye.w = 1.0;

            camera.look_at.x = std::stor(params[3]);
            camera.look_at.y = std::stor(params[4]);
            camera.look_at.z = std::stor(params[5]);
            camera.look_at.w = 1.0;

            camera.up.x = std::stor(params[6]);
            camera.up.y = std::stor(params[7]);
            camera.up.z = std::stor(params[8]);
            camera.up.w = 1.0;

            assert(!first_command);
            camera.field_of_view.y = std::stor(command.params[9]);
            camera.field_of_view.x = image.width * camera.field_of_view.y / image.height;
        } else if (command.name == "vertex") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), is_floating_point)) {
                return "'vertex' command should have 3 floating point parameters.";
            }
            
            assert(!first_command);
            const real x = std::stor(params[0]);
            const real y = std::stor(params[1]);
            const real z = std::stor(params[2]);
            vertices.emplace_back(Vector{x, y, z, 1.0});
        } else if (command.name == "tri") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), is_positive_int)) {
                return "'tri' command should have 3 positive integer parameters.";
            }
            
            const int a_index = std::stoi(params[0]);
            const int b_index = std::stoi(params[1]);
            const int c_index = std::stoi(params[2]);
            if (std::max({a_index, b_index, c_index}) >= vertices.size()) {
                return "Vertex index specified in 'tri' command is beyond the number of specified vertices.";
            }

            const Vector a = current_transform * vertices[a_index];
            const Vector b = current_transform * vertices[b_index];
            const Vector c = current_transform * vertices[c_index];
            scene.triangles.emplace_back(Triangle{a, b, c});
            scene.triangle_ambients.emplace_back(current_ambient);
            scene.triangle_materials.emplace_back(current_material);
        } else if (command.name == "sphere") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 4 || !std::all_of(params.begin(), params.end(), is_floating_point)) {
                return "'sphere' command should have 3 floating point parameters.";
            }
            
            const real x_centre = std::stor(params[0]);
            const real y_centre = std::stor(params[1]);
            const real z_centre = std::stor(params[2]);
            const real radius = std::stor(params[3]);
            const Vector centre{x_centre, y_centre, z_centre, 1.0};

            scene.ellipsoids.emplace_back(Ellipsoid{centre, radius, inverse_current_transform});
            scene.ellipsoid_transforms.emplace_back(current_transform);
            scene.ellipsoid_ambients.emplace_back(current_ambient);
            scene.ellipsoid_materials.emplace_back(current_material);
        } else if (command.name == "pushTransform") {
            if (!command.params.empty()) {
                return "'pushTransform' command does not take any parameters.";
            }
            
            transform_stack.push(current_transform);
            inverse_transform_stack.push(inverse_current_transform);
            current_transform = identity_matrix();
            inverse_current_transform = identity_matrix();
        } else if (command.name == "popTransform") {
            if (!command.params.empty()) {
                return "'popTransform' command does not take any parameters";
            }
            
            if (transform_stack.empty()) {
                return "Cannot perform 'popTransform' as there are no transforms on the stack.";
            }

            assert(!inverse_transform_stack.empty());
            current_transform = transform_stack.top();
            inverse_current_transform = inverse_transform_stack.top();
            transform_stack.pop();
            inverse_transform_stack.pop();
        } else if (command.name == "translate") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), is_floating_point)) {
                return "'translate' command should have 3 floating point parameters.";
            }
            
            const real x_offset = std::stor(params[0]);
            const real y_offset = std::stor(params[1]);
            const real z_offset = std::stor(params[2]);
            const Matrix translation = translation_matrix(x_offset, y_offset, z_offset);
            const Matrix inverse_translation = translation_matrix(-x_offset, -y_offset, -z_offset);
            current_transform = current_transform * translation;
            inverse_current_transform = inverse_translation * inverse_current_transform;
        } else if (command.name == "scale") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), is_floating_point)) {
                return "'scale' command should have 3 floating point parameters.";
            }
            
            const real x_scale = std::stor(params[0]);
            const real y_scale = std::stor(params[1]);
            const real z_scale = std::stor(params[2]);
            const Matrix scaling = scaling_matrix(x_scale, y_scale, z_scale);
            const Matrix inverse_scaling = scaling_matrix(1.0 / x_scale, 1.0 / y_scale, 1.0 / z_scale);
            current_transform = current_transform * scaling;
            inverse_current_transform = inverse_scaling * inverse_current_transform;
        } else if (command.name == "rotate") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 4 || !std::all_of(params.begin(), params.end(), is_floating_point)) {
                return "'rotate' command should have 4 floating point parameters.";
            }
            
            const real axis_x = std::stor(params[0]);
            const real axis_y = std::stor(params[1]);
            const real axis_z = std::stor(params[2]);
            const real angle = std::stor(params[3]);
            const real angle_in_radians = to_radians(angle);
            const Matrix rotation = rotation_matrix(angle_in_radians, axis_x, axis_y, axis_z);
            const Matrix inverse_rotation = rotation_matrix(-angle_in_radians, axis_x, axis_y, axis_z);
            current_transform = current_transform * rotation;
            inverse_current_transform = inverse_rotation * inverse_current_transform;
        } else if (command.name == "directional") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 6 || !std::all_of(params.begin(), params.end(), is_floating_point)) {
                return "'directional' command should have 6 floating point parameters.";
            }
            
            if (!scene.directional_light_source.has_value()) {
                scene.directional_light_source = DirectionalLightSource{
                    Vector{0.0, 0.0, 0.0, 1.0},
                    Colour{0.0, 0.0, 0.0},
                };
            }

            scene.directional_light_source.value().direction.x = std::stor(params[0]);
            scene.directional_light_source.value().direction.y = std::stor(params[1]);
            scene.directional_light_source.value().direction.z = std::stor(params[2]);
            scene.directional_light_source.value().direction.w = 1.0;

            scene.directional_light_source.value().colour.red = std::stor(params[3]);
            scene.directional_light_source.value().colour.green = std::stor(params[4]);
            scene.directional_light_source.value().colour.blue = std::stor(params[5]);
        } else if (command.name == "point") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 6 || !std::all_of(params.begin(), params.end(), is_floating_point)) {
                return "'point' command should have 6 floating point parameters.";
            }
            
            const real x = std::stor(params[0]);
            const real y = std::stor(params[1]);
            const real z = std::stor(params[2]);

            const real r = std::stor(params[3]);
            const real g = std::stor(params[4]);
            const real b = std::stor(params[5]);

            const PointLightSource point_light{
                Vector{x, y, z, 1.0},
                Colour{r, g, b},
            };

            scene.point_light_sources.emplace_back(point_light);
        } else if (command.name == "attenuation") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), is_floating_point)) {
                return "'attenuation' command should have 3 floating point parameters.";
            }
            
            scene.attenuation_parameters.constant = std::stor(params[0]);
            scene.attenuation_parameters.linear = std::stor(params[1]);
            scene.attenuation_parameters.quadratic = std::stor(params[2]);
        } else if (command.name == "ambient") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), is_floating_point)) {
                return "'ambient' command should have 3 floating point parameters.";
            }
            
            current_ambient.red = std::stor(params[0]);
            current_ambient.green = std::stor(params[1]);
            current_ambient.blue = std::stor(params[2]);
        } else if (command.name == "diffuse") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), is_floating_point)) {
                return "'diffuse' command should have 3 floating point parameters.";
            }
            
            current_material.diffuse.red = std::stor(params[0]);
            current_material.diffuse.green = std::stor(params[1]);
            current_material.diffuse.blue = std::stor(params[2]);
        } else if (command.name == "specular") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), is_floating_point)) {
                return "'specular' command should have 3 floating point parameters.";
            }
            
            current_material.specular.red = std::stor(params[0]);
            current_material.specular.green = std::stor(params[1]);
            current_material.specular.blue = std::stor(params[2]);
        } else if (command.name == "emission") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !std::all_of(params.begin(), params.end(), is_floating_point)) {
                return "'emission' command should have 3 floating point parameters.";
            }
            
            current_material.emission.red = std::stor(params[0]);
            current_material.emission.green = std::stor(params[1]);
            current_material.emission.blue = std::stor(params[2]);
        } else if (command.name == "shininess") {
            if (command.params.size() != 1 || !is_floating_point(command.params.front())) {
                return "'shininess' command should have 1 floating point parameter.";
            }
            
            current_material.shininess = std::stor(command.params.front());
        } else {
            return "Unknown command entered.";
        }

        first_command = false;
    }

    return FileInfo{image, camera, scene, max_recursion_depth};
}
