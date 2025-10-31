#shader vertex
#version 330 core

layout(location = 0) in vec4 position;
layout(location = 1) in vec3 in_color;

uniform mat4 u_MVP;

void main() {
	gl_Position = u_MVP * position;
}

#shader fragment
#version 330 core

out vec4 color;
uniform int u_Color;

void main() {
	if (u_Color == 0) {
		color = vec4(1.0, 0, 0, 1.0);
	}
	else if (u_Color == 1) {
		color = vec4(0, 1.0, 0, 1.0);
	}
	else {
		color = vec4(0, 0, 1.0, 1.0);
	}
}
