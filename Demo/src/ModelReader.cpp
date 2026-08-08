#include "ModelReader.h"
#include <fstream>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <cstdlib>

static bool stringStartsWith(std::string s, std::string prefix) {
	if (s.size() < prefix.size()) return false;
	for (int i = 0; i < prefix.size(); i++) {
		if (s[i] != prefix[i]) return false;
	}
	return true;
}

static char* getToken(char* s, const char* delim, char** next_token) {
	assert(*next_token != NULL);
	char* token = strtok_r(s, delim, next_token);
	return token;
}

static void outputData(MeshColliderOutput* out, bool is_current_object_collider, const std::string current_object_name, const std::vector<mthz::Vec3>& current_vertices, const std::vector<std::vector<uint32_t>>& current_face_indices) {
	if (is_current_object_collider) {
		out->colliders[current_object_name] = phyz::ConvexUnionGeometry(phyz::ConvexPrimitive((const phyz::ConvexGeometry&)phyz::Polyhedron::getPolyAfterFindMergedCoplanarFaces(phyz::Polyhedron(current_vertices, current_face_indices)), phyz::Material::default_material()));
	}
	else {
		//this whole code block is super dumb but is also a temporary hack. Will have to redo things anyway when textures are added
		std::vector<std::vector<uint32_t>> unsigned_face_indices;
		for (const std::vector<uint32_t>& face_indices : current_face_indices) {
			std::vector<unsigned int> unsigned_face;
			for (int i : face_indices) unsigned_face.push_back(i);
			unsigned_face_indices.push_back(unsigned_face);
		}
		out->meshes[current_object_name] = fromPhyzMesh(phyz::Mesh{ current_vertices, unsigned_face_indices });
	}
}

MeshColliderOutput readMeshAndColliders(std::string formatted_obj_path, double scale) {
	std::ifstream in(formatted_obj_path);
	assert(in.good());
	
	MeshColliderOutput out;

	uint32_t vertex_offset = 1;

	bool no_current_object = true;
	bool is_current_object_collider;
	std::string current_object_name;
	std::vector<mthz::Vec3> current_vertices;
	std::vector<std::vector<uint32_t>> current_face_indices;

	std::string line;

	static char buff[2048];
	while (std::getline(in, line)) {
		sprintf(buff, "%s", line.c_str());

		
		char* token;
		char* next_token;
		const char* delim = " ";
		if ((token = strtok_r(buff, delim, &next_token)) != NULL) {

			if (strcmp(token, "g") == 0) { //new group
				if (!no_current_object) {
					outputData(&out, is_current_object_collider, current_object_name, current_vertices, current_face_indices);
					vertex_offset += static_cast<uint32_t>(current_vertices.size());
					current_vertices.clear();
					current_face_indices.clear();
				}
				no_current_object = false;
				current_object_name = std::string(getToken(NULL, delim, &next_token));
				current_object_name = current_object_name.substr(0, current_object_name.find_last_of('_')); //blender applies a _<mesh_name> automatically. This is to get rid of that.

				std::string collider_prefix = "col_";
				if (stringStartsWith(current_object_name, collider_prefix)) {
					is_current_object_collider = true;
					current_object_name = current_object_name.substr(collider_prefix.size()); //removes "col_" from object_name
				}
				else {
					is_current_object_collider = false;
				}

			} 
			else if (strcmp(token, "v") == 0) { //vertex
				assert(!no_current_object);

				double x = scale * atof(getToken(NULL, delim, &next_token));
				double y = scale * atof(getToken(NULL, delim, &next_token));
				double z = scale * atof(getToken(NULL, delim, &next_token));
				current_vertices.push_back(mthz::Vec3(x, y, z));
			}
			else if (strcmp(token, "f") == 0) { //face
				assert(!no_current_object);

				std::vector<uint32_t> indices;
				while ((token = strtok_r(NULL, delim, &next_token)) != NULL) {
					int index = atoi(token) - vertex_offset;
					assert(index >= 0 && index < current_vertices.size());
					indices.push_back(index);
				}
				assert(indices.size() >= 3);
				current_face_indices.push_back(indices);
			}
		}
	}

	
	if (!no_current_object) outputData(&out, is_current_object_collider, current_object_name, current_vertices, current_face_indices);
	return out;
}