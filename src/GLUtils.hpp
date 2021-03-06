#pragma once

GLFWwindow *init_glfw(const std::string& window_name);
void init_glew();
GLuint compile_shader(std::string filename, GLenum shader_type);
GLuint compiler_render_program(std::string vertex_shader_filename, std::string fragment_shader_filename, std::string out_color_name);
glm::vec3 random_saturated_color();