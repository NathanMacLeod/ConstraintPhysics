#shader vertex
#version 330 core

layout(location = 0) in vec4 position;
layout(location = 1) in vec3 in_color;
layout(location = 2) in float in_ambient_k;
layout(location = 3) in float in_diffuse_k;
layout(location = 4) in float in_specular_k;
layout(location = 5) in float in_specular_p;
layout(location = 6) in float tex_id;
layout(location = 7) in vec2 tex_coords;

out vec3 v_Color;
uniform mat4 u_P;

void main() {
	gl_Position = u_P * position;
	v_Color = in_color;
}

#shader fragment
#version 330 core

in vec3 v_Color;
out vec4 color;

void main() {
	color = vec4(v_Color, 1.0);
}
