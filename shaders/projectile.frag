#version 330

const float RIM_WIDTH = 0.05;

in vec3 vertex_color;

out vec4 out_color;

void main() {
    // out_color = vec4(gl_PointCoord.x,0.0,gl_PointCoord.y,1.0);
    float l = length(gl_PointCoord - vec2(0.5));
    if (l > 0.5) {
        discard;
    }
    if (l > 0.5 - RIM_WIDTH) {
        out_color = vec4(0.0, 0.0, 0.0, 1.0);
    } else {
        out_color = vec4(vertex_color, 1.0);
    }
}
