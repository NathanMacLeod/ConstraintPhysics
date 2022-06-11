#shader vertex
#version 330 core

layout(location = 0) in vec4 position;
layout(location = 1) in vec4 in_color;

out vec4 v_Color;
uniform mat4 u_MVP;

void main() {
	gl_Position = u_MVP * position;
	v_Color = in_color;
}

#shader fragment
#version 330 core

in vec4 v_Color;
out vec4 color;

void main() {
	color = v_Color;
}
