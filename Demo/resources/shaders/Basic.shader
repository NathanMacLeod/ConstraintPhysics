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
out vec4 v_mat_props;
out float v_tex_ids;
out vec2 v_tex_coords;

void main() {
	v_mat_props = vec4(in_ambient_k, in_diffuse_k, in_specular_k, in_specular_p);
	gl_Position = position;
	v_Color = in_color;
	v_tex_ids = tex_id;
}

#shader geometry
#version 330 core

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

in vec3 v_Color[];
in vec4 v_mat_props[];
in float v_tex_ids[];
in vec2 v_tex_coords[];
out vec3 g_Color;
out vec4 g_mat_props;
out vec3 g_pos;
out vec3 g_Norm;
out float g_tex_id;
out vec2 g_tex_coords;

uniform mat4 u_P;

vec3 calcNormal(vec3 p0, vec3 p1, vec3 p2) {
	vec3 v1 = p0 - p1;
	vec3 v2 = p2 - p1;
	return normalize(cross(v2, v1));
}

void main() {
	g_Norm = calcNormal(vec3(gl_in[0].gl_Position), vec3(gl_in[1].gl_Position), vec3(gl_in[2].gl_Position));

	g_Color = v_Color[0];
	g_mat_props = v_mat_props[0];
	gl_Position = u_P * gl_in[0].gl_Position;
	g_pos = vec3(gl_in[0].gl_Position);
	g_tex_id = v_tex_ids[0];
	g_tex_coords = v_tex_coords[0];
	EmitVertex();
	g_Color = v_Color[1];
	g_mat_props = v_mat_props[1];
	gl_Position = u_P * gl_in[1].gl_Position;
	g_pos = vec3(gl_in[1].gl_Position);
	g_tex_id = v_tex_ids[1];
	g_tex_coords = v_tex_coords[1];
	EmitVertex();
	g_Color = v_Color[2];
	g_mat_props = v_mat_props[2];
	gl_Position = u_P * gl_in[2].gl_Position;
	g_pos = vec3(gl_in[2].gl_Position);
	g_tex_id = v_tex_ids[2];
	g_tex_coords = v_tex_coords[2];
	EmitVertex();
}

#shader fragment
#version 330 core

in vec3 g_Norm;
in vec3 g_pos;
in vec3 g_Color;
in vec4 g_mat_props;
in float g_tex_id;
in vec2 g_tex_coords;
out vec4 color;

uniform int u_Asleep;
uniform vec3 u_ambient_light;
uniform vec3 u_pointlight_col;
uniform vec3 u_pointlight_pos;
uniform sampler2D u_textures[32];


void main() {

	float ambient_k = g_mat_props[0];
	float diffuse_k = g_mat_props[1];
	float specular_k = g_mat_props[2];
	float specular_p = g_mat_props[3];

	vec3 surface_color;
	int tex_index = int(g_tex_id);
	//something going wrong here
	if (false && g_tex_id != -1) {
		// vec4 tex_color = texture(u_textures[tex_index], g_tex_coords);
		// surface_color = vec3(tex_color[0], tex_color[1], tex_color[2]);
	}
	else {
		surface_color = (u_Asleep == 1) ? vec3(1.0, 0, 0) : g_Color;
	}
	vec3 observed_ambient = ambient_k * u_ambient_light * surface_color;

	vec3 light_dir = normalize(u_pointlight_pos - vec3(g_pos));

	float diffuse_directional_factor = dot(light_dir, g_Norm);
	vec3 observed_diffuse = diffuse_k * diffuse_directional_factor * u_pointlight_col * surface_color;

	vec3 camera_dir = -normalize(vec3(g_pos));
	vec3 light_reflect_direction = 2 * g_Norm * dot(g_Norm, light_dir) - light_dir;
	
	vec3 observed_specular = vec3(0, 0, 0);
	float specular_directional_factor = dot(light_reflect_direction, camera_dir);
	if (specular_directional_factor > 0) {
		observed_specular = specular_k* pow(specular_directional_factor, specular_p)* u_pointlight_col;
	}

	color = vec4(observed_ambient + observed_diffuse + observed_specular, 1.0);
}
