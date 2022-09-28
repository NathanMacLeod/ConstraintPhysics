#include "HACD.h"

#include <fstream>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <set>
#include <unordered_map>

namespace phyz {

	AABB Mesh::getAABB() {
		const double inf = std::numeric_limits<double>::infinity();
		mthz::Vec3 min(inf, inf, inf);
		mthz::Vec3 max(-inf, -inf, -inf);

		for (const mthz::Vec3& p : vertices) {
			min.x = std::min<double>(min.x, p.x);
			min.y = std::min<double>(min.y, p.y);
			min.z = std::min<double>(min.z, p.z);

			max.x = std::max<double>(max.x, p.x);
			max.y = std::max<double>(max.y, p.y);
			max.z = std::max<double>(max.z, p.z);
		}

		return { min, max };
	}

	static char* get_token(char* s, const char* delim, char** next_token) {
		assert(*next_token != NULL);
		char* token = strtok_s(s, delim, next_token);
		return token;
	}

	Mesh readOBJ(const std::string& file_path, double scale) {
		Mesh out = { std::vector<mthz::Vec3>(), std::vector<std::vector<unsigned int>>() };

		std::ifstream in(file_path);
		assert(in.good());

		std::string line;

		static char buff[2048];
		std::set<int> referenced_index;
		while (std::getline(in, line)) {
			sprintf_s(buff, "%s", line.c_str());

			char* token;
			char* next_token;
			const char* delim = " ";
			if ((token = strtok_s(buff, delim, &next_token)) != NULL) {

				//vertex 
				if (strcmp(token, "v") == 0) {
					double x = scale * atof(get_token(NULL, delim, &next_token));
					double y = scale * atof(get_token(NULL, delim, &next_token));
					double z = scale * atof(get_token(NULL, delim, &next_token));
					out.vertices.push_back(mthz::Vec3(x, y, z));
				}
				else if (strcmp(token, "f") == 0) {
					std::vector<unsigned int> indices;
					while ((token = strtok_s(NULL, delim, &next_token)) != NULL) {
						int index = atoi(token) - 1;
						referenced_index.insert(index);
						assert(index >= 0 && index < out.vertices.size());
						indices.push_back(index);
					}
					assert(indices.size() >= 3);
					out.face_indices.push_back(indices);
				}
			}
		}

		for (int i = 0; i < out.vertices.size(); i++) {
			if (referenced_index.find(i) == referenced_index.end()) {
				mthz::Vec3 v = out.vertices[i];
				printf("%f %f %f\n", v.x, v.y, v.z);
			}
		}

		return out;
	}

}