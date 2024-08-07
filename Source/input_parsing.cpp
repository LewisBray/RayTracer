#include <sstream>
#include <fstream>
#include <cassert>
#include <vector>
#include <string>

#include "input_parsing.h"
#include "ray_tracing.h"
#include "maths.h"

static bool is_digit(const char c) {
    return ('0' <= c && c <= '9');
}

static bool is_positive_int(const std::string& str) {
    for (const char c : str) {
        if (!is_digit(c)) {
            return false;
        }
    }

    return true;
}

static bool is_floating_point(const std::string& str) {
    assert(!str.empty());   // string should be coming from command params which are checked to be non-empty

    const char first_char = str[0];
    const bool first_char_valid = (first_char == '-' || first_char == '+' || first_char == '.' || is_digit(first_char));
    if (!first_char_valid) {
        return false;
    }

    bool previously_encountered_dot = (first_char == '.');

    const std::size_t char_count = str.size();
    for (std::size_t i = std::size_t{1}; i < char_count; ++i) {
        const char c = str[i];
        if (c == '+' || c == '-') {
            return false;
        } else if (c == '.') {
            if (previously_encountered_dot) {
                return false;
            }
            previously_encountered_dot = true;
        } else if (!is_digit(c)) {
            return false;
        }
    }

    return true;
}

static bool all_of(const std::vector<std::string>& strings, bool(*f)(const std::string&)) {
    for (const std::string& str : strings) {
        if (!f(str)) {
            return false;
        }
    }

    return true;
}

struct Command {
    std::string name;
    std::vector<std::string> params;
};

static Command parse_command(const std::string& str) {
    Command command;

    std::string token;
    std::stringstream token_stream{str};
    while (std::getline(token_stream, token, ' ')) {
        if (token.empty()) {
            continue;
        }

        if (command.name.empty()) {
            command.name = token;
        } else {
            command.params.push_back(token);
        }
    }

    return command;
}

static ParseInputFileResult parse_error(const char* const error) {
    ParseInputFileResult result = {};
    result.error = error;

    return result;
}

static ParseInputFileResult parse_input_file(const char* const filename) {
    TIME_BLOCK("parse input file");
    
    std::ifstream input_file(filename);
    if (!input_file.is_open()) {
        return parse_error("Failed to open input file.");
    }
    
    Scene scene;
    int max_recursion_depth = 5;
    Image image{0, 0, nullptr, "raytrace.png"};
    Camera camera{
        Vector{0.0f, 0.0f, 0.0f},
        Vector{0.0f, 0.0f, 0.0f},
        Vector{0.0f, 0.0f, 0.0f},
        FieldOfView{0.0f, 0.0f}
    };

    // Shapes
    std::vector<Vector> vertices;
    Matrix current_transform = identity_matrix();
    Matrix inverse_current_transform = identity_matrix();
    std::vector<Matrix> transform_stack;
    std::vector<Matrix> inverse_transform_stack;

    // bounding boxes
    scene.bounding_box = AxisAlignedBoundingBox{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // Lighting
    scene.ambient = Colour{0.2f, 0.2f, 0.2f};
    scene.directional_light_source = DirectionalLightSource{Vector{0.0f, 0.0f, 0.0f}, Colour{0.0f, 0.0f, 0.0f}};
    scene.has_directional_light_source = false;
    scene.attenuation_parameters = AttenuationParameters{1.0f, 0.0f, 0.0f};

    // Materials
    Material current_material{
        Colour{0.0f, 0.0f, 0.0f},
        Colour{0.0f, 0.0f, 0.0f},
        Colour{0.0f, 0.0f, 0.0f},
        0.0f
    };
    
    std::size_t triangle_count = 0;
    std::size_t sphere_count = 0;
    std::size_t ellipsoid_count = 0;
    
    std::string line;
    bool first_command = true;
    while (std::getline(input_file, line, '\n')) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        const Command command = parse_command(line);
        if (first_command && command.name != "size") {
            return parse_error("First command should be 'size'.");
        }
                
        if (command.name == "size") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 2 || !all_of(params, is_positive_int)) {
                return parse_error("'size' command should have 2 positive integer parameters.");
            }
                        
            image.width = static_cast<unsigned>(std::stoi(params[0]));
            image.height = static_cast<unsigned>(std::stoi(params[1]));
            image.pixels = new unsigned char[static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height) * 3ul];
        } else if (command.name == "output") {
            if (command.params.size() != 1) {
                return parse_error("'output' command should have 1 parameter.");
            }

            const std::string& output_name = command.params[0];
            if (output_name.length() + 1 > sizeof(image.filename)) {
                return parse_error("'output' name is too long.");
            }

            for (std::size_t i = 0; i < output_name.length(); ++i) {
                image.filename[i] = output_name[i]; // TODO: do I need to add .png to the end if it isn't there?
            }

            for (std::size_t i = output_name.length(); i < sizeof(image.filename); ++i) {
                image.filename[i] = '\0';
            }
        } else if (command.name == "maxdepth") {
            if (command.params.size() != 1 || !is_positive_int(command.params[0])) {
                return parse_error("'maxdepth' command should have 1 positive integer parameter.");
            }
            
            max_recursion_depth = std::stoi(command.params.front());
        } else if (command.name == "maxverts") {
            if (!vertices.empty()) {
                return parse_error("'maxverts' should be specified before vertices are specified.");
            }

            if (command.params.size() != 1 || !is_positive_int(command.params[0])) {
                return parse_error("'maxverts' command should have 1 positive integer parameter.");
            }
            
            vertices.reserve(std::stoul(command.params[0]));
        } else if (command.name == "camera") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 10 || !all_of(params, is_floating_point)) {
                return parse_error("'camera' command should have 10 floating point parameters.");
            }
            
            camera.eye.x = std::stof(params[0]);
            camera.eye.y = std::stof(params[1]);
            camera.eye.z = std::stof(params[2]);

            camera.look_at.x = std::stof(params[3]);
            camera.look_at.y = std::stof(params[4]);
            camera.look_at.z = std::stof(params[5]);

            camera.up.x = std::stof(params[6]);
            camera.up.y = std::stof(params[7]);
            camera.up.z = std::stof(params[8]);

            assert(!first_command);
            camera.field_of_view.y = std::stof(command.params[9]);
            camera.field_of_view.x = static_cast<float>(image.width) * camera.field_of_view.y / static_cast<float>(image.height);
        } else if (command.name == "vertex") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !all_of(params, is_floating_point)) {
                return parse_error("'vertex' command should have 3 floating point parameters.");
            }
            
            assert(!first_command);
            const float x = std::stof(params[0]);
            const float y = std::stof(params[1]);
            const float z = std::stof(params[2]);
            vertices.emplace_back(Vector{x, y, z});
        } else if (command.name == "tri") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !all_of(params, is_positive_int)) {
                return parse_error("'tri' command should have 3 positive integer parameters.");
            }
            
            const std::size_t a_index = std::stoul(params[0]);   // TODO: this throws if cannot be parsed as int
            const std::size_t b_index = std::stoul(params[1]);
            const std::size_t c_index = std::stoul(params[2]);

            const std::size_t vertex_count = vertices.size();
            if (a_index >= vertex_count || b_index >= vertex_count || c_index >= vertex_count) {
                return parse_error("Vertex index specified in 'tri' command is beyond the number of specified vertices.");
            }

            Matrix transform = identity_matrix();
            for (const Matrix& m : transform_stack) {
                transform = transform * m;
            }

            transform = transform * current_transform;

            const Vector a = transform * vertices[a_index];
            const Vector b = transform * vertices[b_index];
            const Vector c = transform * vertices[c_index];
            scene.triangle_materials.emplace_back(current_material);

            const Vector a_to_b = b - a;
            const Vector a_to_c = c - a;
            
            const std::size_t triangle_batch_index = triangle_count / 8;
            const std::size_t triangle_index = triangle_count % 8;
            if (triangle_index == 0) {
                scene.triangle8s.push_back(Triangle8{});
            }
            
            Triangle8& triangle8 = scene.triangle8s[triangle_batch_index];
            triangle8.a.x[triangle_index] = a.x;
            triangle8.a.y[triangle_index] = a.y;
            triangle8.a.z[triangle_index] = a.z;
            
            triangle8.a_to_b.x[triangle_index] = a_to_b.x;
            triangle8.a_to_b.y[triangle_index] = a_to_b.y;
            triangle8.a_to_b.z[triangle_index] = a_to_b.z;

            triangle8.a_to_c.x[triangle_index] = a_to_c.x;
            triangle8.a_to_c.y[triangle_index] = a_to_c.y;
            triangle8.a_to_c.z[triangle_index] = a_to_c.z;

            // update scene bounding box
            const float triangle_min_x = fp_min(a.x, fp_min(b.x, c.x));
            const float triangle_max_x = fp_max(a.x, fp_max(b.x, c.x));
            const float triangle_min_y = fp_min(a.y, fp_min(b.y, c.y));
            const float triangle_max_y = fp_max(a.y, fp_max(b.y, c.y));
            const float triangle_min_z = fp_min(a.z, fp_min(b.z, c.z));
            const float triangle_max_z = fp_max(a.z, fp_max(b.z, c.z));

            scene.bounding_box.min_x = fp_min(scene.bounding_box.min_x, triangle_min_x);
            scene.bounding_box.max_x = fp_max(scene.bounding_box.max_x, triangle_max_x);
            scene.bounding_box.min_y = fp_min(scene.bounding_box.min_y, triangle_min_y);
            scene.bounding_box.max_y = fp_max(scene.bounding_box.max_y, triangle_max_y);
            scene.bounding_box.min_z = fp_min(scene.bounding_box.min_z, triangle_min_z);
            scene.bounding_box.max_z = fp_max(scene.bounding_box.max_z, triangle_max_z);
            
            ++triangle_count;
        } else if (command.name == "sphere") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 4 || !all_of(params, is_floating_point)) {
                return parse_error("'sphere' command should have 4 floating point parameters.");
            }
            
            const float x_centre = std::stof(params[0]);
            const float y_centre = std::stof(params[1]);
            const float z_centre = std::stof(params[2]);
            const float radius = std::stof(params[3]);

            Matrix transform = identity_matrix();
            for (const Matrix& m : transform_stack) {
                transform = transform * m;
            }

            transform = transform * current_transform;

            Matrix inverse_transform = identity_matrix();
            for (const Matrix&m : inverse_transform_stack) {
                inverse_transform = m * inverse_transform;
            }

            inverse_transform = inverse_current_transform * inverse_transform;

            const float x0 = transform.rows[0][0];
            const float x1 = transform.rows[1][0];
            const float x2 = transform.rows[2][0];
            const float y0 = transform.rows[0][1];
            const float y1 = transform.rows[1][1];
            const float y2 = transform.rows[2][1];
            const float z0 = transform.rows[0][2];
            const float z1 = transform.rows[1][2];
            const float z2 = transform.rows[2][2];
            const float x_scale_squared = x0 * x0 + x1 * x1 + x2 * x2;
            const float y_scale_squared = y0 * y0 + y1 * y1 + y2 * y2;
            const float z_scale_squared = z0 * z0 + z1 * z1 + z2 * z2;
            if (!are_equal(x_scale_squared, y_scale_squared) || !are_equal(y_scale_squared, z_scale_squared)) { // i.e. is ellipsoid, not sphere
                // ensure ellipsoid sphere has centre at origin
                if (x_centre != 0.0f || y_centre != 0.0f || z_centre != 0.0f) {
                    const Matrix centre_translation = translation_matrix(x_centre, y_centre, z_centre);
                    transform = transform * centre_translation;

                    const Matrix inverse_centre_translation = translation_matrix(-x_centre, -y_centre, -z_centre);
                    inverse_transform = inverse_centre_translation * inverse_transform;
                }

                // ensure ellipsoid sphere radius is 1
                if (radius != 1.0f) {
                    const Matrix radius_scaling = scaling_matrix(radius, radius, radius);
                    transform = transform * radius_scaling;

                    const float inverse_radius = 1.0f / radius;
                    const Matrix inverse_radius_scaling = scaling_matrix(inverse_radius, inverse_radius, inverse_radius);
                    inverse_transform = inverse_radius_scaling * inverse_transform;
                }

                scene.ellipsoid_materials.emplace_back(current_material);

                const std::size_t ellipsoid_batch_index = ellipsoid_count / 8;
                const std::size_t ellipsoid_index = ellipsoid_count % 8;
                if (ellipsoid_index == 0) {
                    scene.ellipsoid8_inverse_transforms.push_back(Mat3x4x8{});
                    scene.ellipsoid8_transforms.push_back(Mat3x4x8{});
                }
                
                Mat3x4x8& ellipsoid8_inverse_transform = scene.ellipsoid8_inverse_transforms[ellipsoid_batch_index];
                for (int row = 0; row < 3; ++row) {
                    for (int column = 0; column < 4; ++column) {
                        ellipsoid8_inverse_transform.rows[row][column][ellipsoid_index] = inverse_transform.rows[row][column];
                    }
                }

                Mat3x4x8& ellipsoid8_transform = scene.ellipsoid8_transforms[ellipsoid_batch_index];
                for (int row = 0; row < 3; ++row) {
                    for (int column = 0; column < 4; ++column) {
                        ellipsoid8_transform.rows[row][column][ellipsoid_index] = transform.rows[row][column];
                    }
                }

                // update scene bounding box
                const float x_component_magnitude_squared =
                    transform.rows[0][0] * transform.rows[0][0] +
                    transform.rows[0][1] * transform.rows[0][1] +
                    transform.rows[0][2] * transform.rows[0][2];
                const float x_component_magnitude = fp_sqrt(x_component_magnitude_squared);

                const float y_component_magnitude_squared =
                    transform.rows[1][0] * transform.rows[1][0] +
                    transform.rows[1][1] * transform.rows[1][1] +
                    transform.rows[1][2] * transform.rows[1][2];
                const float y_component_magnitude = fp_sqrt(y_component_magnitude_squared);

                const float z_component_magnitude_squared =
                    transform.rows[2][0] * transform.rows[2][0] +
                    transform.rows[2][1] * transform.rows[2][1] +
                    transform.rows[2][2] * transform.rows[2][2];
                const float z_component_magnitude = fp_sqrt(z_component_magnitude_squared);

                const float ellipsoid_min_x = transform.rows[0][3] - x_component_magnitude;
                const float ellipsoid_max_x = transform.rows[0][3] + x_component_magnitude;
                const float ellipsoid_min_y = transform.rows[1][3] - y_component_magnitude;
                const float ellipsoid_max_y = transform.rows[1][3] + y_component_magnitude;
                const float ellipsoid_min_z = transform.rows[2][3] - z_component_magnitude;
                const float ellipsoid_max_z = transform.rows[2][3] + z_component_magnitude;

                scene.bounding_box.min_x = fp_min(scene.bounding_box.min_x, ellipsoid_min_x);
                scene.bounding_box.max_x = fp_max(scene.bounding_box.max_x, ellipsoid_max_x);
                scene.bounding_box.min_y = fp_min(scene.bounding_box.min_y, ellipsoid_min_y);
                scene.bounding_box.max_y = fp_max(scene.bounding_box.max_y, ellipsoid_max_y);
                scene.bounding_box.min_z = fp_min(scene.bounding_box.min_z, ellipsoid_min_z);
                scene.bounding_box.max_z = fp_max(scene.bounding_box.max_z, ellipsoid_max_z);
                
                ++ellipsoid_count;
            } else {
                const Vector centre{x_centre, y_centre, z_centre};
                const Vector transformed_centre = transform * centre;

                const float axes_scaling = fp_sqrt(x_scale_squared);
                const float scaled_radius = axes_scaling * radius;

                scene.sphere_materials.emplace_back(current_material);

                const std::size_t sphere_batch_index = sphere_count / 8;
                const std::size_t sphere_index = sphere_count % 8;
                if (sphere_index == 0) {
                    scene.sphere8s.push_back(Sphere8{});
                }
                
                Sphere8& sphere8 = scene.sphere8s[sphere_batch_index];
                sphere8.centre.x[sphere_index] = transformed_centre.x;
                sphere8.centre.y[sphere_index] = transformed_centre.y;
                sphere8.centre.z[sphere_index] = transformed_centre.z;

                sphere8.radius[sphere_index] = scaled_radius;

                // update scene bounding box
                const float sphere_min_x = transformed_centre.x - scaled_radius;
                const float sphere_max_x = transformed_centre.x + scaled_radius;
                const float sphere_min_y = transformed_centre.y - scaled_radius;
                const float sphere_max_y = transformed_centre.y + scaled_radius;
                const float sphere_min_z = transformed_centre.z - scaled_radius;
                const float sphere_max_z = transformed_centre.z + scaled_radius;

                scene.bounding_box.min_x = fp_min(scene.bounding_box.min_x, sphere_min_x);
                scene.bounding_box.max_x = fp_max(scene.bounding_box.max_x, sphere_max_x);
                scene.bounding_box.min_y = fp_min(scene.bounding_box.min_y, sphere_min_y);
                scene.bounding_box.max_y = fp_max(scene.bounding_box.max_y, sphere_max_y);
                scene.bounding_box.min_z = fp_min(scene.bounding_box.min_z, sphere_min_z);
                scene.bounding_box.max_z = fp_max(scene.bounding_box.max_z, sphere_max_z);
                
                ++sphere_count;
            }
        } else if (command.name == "pushTransform") {
            if (!command.params.empty()) {
                return parse_error("'pushTransform' command does not take any parameters.");
            }
            
            transform_stack.push_back(current_transform);
            inverse_transform_stack.push_back(inverse_current_transform);
            current_transform = identity_matrix();
            inverse_current_transform = identity_matrix();
        } else if (command.name == "popTransform") {
            if (!command.params.empty()) {
                return parse_error("'popTransform' command does not take any parameters");
            }
            
            if (transform_stack.empty()) {
                return parse_error("Cannot perform 'popTransform' as there are no transforms on the stack.");
            }

            assert(!inverse_transform_stack.empty());
            current_transform = transform_stack.back();
            inverse_current_transform = inverse_transform_stack.back();
            transform_stack.pop_back();
            inverse_transform_stack.pop_back();
        } else if (command.name == "translate") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !all_of(params, is_floating_point)) {
                return parse_error("'translate' command should have 3 floating point parameters.");
            }
            
            const float x_offset = std::stof(params[0]);
            const float y_offset = std::stof(params[1]);
            const float z_offset = std::stof(params[2]);
            const Matrix translation = translation_matrix(x_offset, y_offset, z_offset);
            const Matrix inverse_translation = translation_matrix(-x_offset, -y_offset, -z_offset);
            current_transform = current_transform * translation;
            inverse_current_transform = inverse_translation * inverse_current_transform;
        } else if (command.name == "scale") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !all_of(params, is_floating_point)) {
                return parse_error("'scale' command should have 3 floating point parameters.");
            }
            
            const float x_scale = std::stof(params[0]);
            const float y_scale = std::stof(params[1]);
            const float z_scale = std::stof(params[2]);
            const Matrix scaling = scaling_matrix(x_scale, y_scale, z_scale);
            const Matrix inverse_scaling = scaling_matrix(1.0f / x_scale, 1.0f / y_scale, 1.0f / z_scale);
            current_transform = current_transform * scaling;
            inverse_current_transform = inverse_scaling * inverse_current_transform;
        } else if (command.name == "rotate") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 4 || !all_of(params, is_floating_point)) {
                return parse_error("'rotate' command should have 4 floating point parameters.");
            }
            
            const float axis_x = std::stof(params[0]);
            const float axis_y = std::stof(params[1]);
            const float axis_z = std::stof(params[2]);
            const float angle = std::stof(params[3]);
            const float angle_in_radians = to_radians(angle);
            const Matrix rotation = rotation_matrix(angle_in_radians, axis_x, axis_y, axis_z);
            const Matrix inverse_rotation = rotation_matrix(-angle_in_radians, axis_x, axis_y, axis_z);
            current_transform = current_transform * rotation;
            inverse_current_transform = inverse_rotation * inverse_current_transform;
        } else if (command.name == "directional") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 6 || !all_of(params, is_floating_point)) {
                return parse_error("'directional' command should have 6 floating point parameters.");
            }

            scene.has_directional_light_source = true;
            
            const Vector non_unit_direction = Vector{std::stof(params[0]), std::stof(params[1]), std::stof(params[2])};
            scene.directional_light_source.direction = normalise(non_unit_direction);

            scene.directional_light_source.colour.red = std::stof(params[3]);
            scene.directional_light_source.colour.green = std::stof(params[4]);
            scene.directional_light_source.colour.blue = std::stof(params[5]);
        } else if (command.name == "point") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 6 || !all_of(params, is_floating_point)) {
                return parse_error("'point' command should have 6 floating point parameters.");
            }
            
            const float x = std::stof(params[0]);
            const float y = std::stof(params[1]);
            const float z = std::stof(params[2]);

            const float r = std::stof(params[3]);
            const float g = std::stof(params[4]);
            const float b = std::stof(params[5]);

            const PointLightSource point_light{
                Vector{x, y, z},
                Colour{r, g, b}
            };

            scene.point_light_sources.emplace_back(point_light);
        } else if (command.name == "attenuation") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !all_of(params, is_floating_point)) {
                return parse_error("'attenuation' command should have 3 floating point parameters.");
            }
            
            scene.attenuation_parameters.constant = std::stof(params[0]);
            scene.attenuation_parameters.linear = std::stof(params[1]);
            scene.attenuation_parameters.quadratic = std::stof(params[2]);
        } else if (command.name == "ambient") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !all_of(params, is_floating_point)) {
                return parse_error("'ambient' command should have 3 floating point parameters.");
            }
            
            scene.ambient.red = std::stof(params[0]);
            scene.ambient.green = std::stof(params[1]);
            scene.ambient.blue = std::stof(params[2]);
        } else if (command.name == "diffuse") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !all_of(params, is_floating_point)) {
                return parse_error("'diffuse' command should have 3 floating point parameters.");
            }
            
            current_material.diffuse.red = std::stof(params[0]);
            current_material.diffuse.green = std::stof(params[1]);
            current_material.diffuse.blue = std::stof(params[2]);
        } else if (command.name == "specular") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !all_of(params, is_floating_point)) {
                return parse_error("'specular' command should have 3 floating point parameters.");
            }
            
            current_material.specular.red = std::stof(params[0]);
            current_material.specular.green = std::stof(params[1]);
            current_material.specular.blue = std::stof(params[2]);
        } else if (command.name == "emission") {
            const std::vector<std::string>& params = command.params;
            if (params.size() != 3 || !all_of(params, is_floating_point)) {
                return parse_error("'emission' command should have 3 floating point parameters.");
            }
            
            current_material.emission.red = std::stof(params[0]);
            current_material.emission.green = std::stof(params[1]);
            current_material.emission.blue = std::stof(params[2]);
        } else if (command.name == "shininess") {
            if (command.params.size() != 1 || !is_floating_point(command.params.front())) {
                return parse_error("'shininess' command should have 1 floating point parameter.");
            }
            
            current_material.shininess = std::stof(command.params.front());
        } else {
            return parse_error("Unknown command entered.");
        }

        first_command = false;
    }

    return ParseInputFileResult{FileInfo{scene, image, camera, max_recursion_depth}, nullptr};
}
