#version 330

uniform mat4 view;
uniform mat4 proj;

in vec3 position;
in vec3 color;
in float radius;

out vec3 vertex_color;

void main() {
    // vertex_color = color;
    vertex_color = vec3(0.5,0.5,0.5);


    vec4 pos = proj * view * vec4(position, 1.0);

    gl_PointSize = radius/pos.w;
    gl_Position = pos;
}
