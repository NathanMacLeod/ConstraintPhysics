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
						int index = atoi(token) - 1; //haha obj your so quirky starting at index 1, not 0 xd xd xd
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

	//static struct Face;

	//static struct Face {
	//	Face(const std::vector<mthz::Vec3>& vertices, std::vector<unsigned int> indices, mthz::Vec3 normalish)
	//		: indices(indices), flagged_for_remove(false)
	//	{
	//		mthz::Vec3 v1 = vertices[indices[1]] - vertices[indices[0]];
	//		int i = 2;
	//		mthz::Vec3 v2;
	//		do {
	//			v2 = vertices[indices[i++]] - vertices[indices[0]];
	//		} while (abs(v2.dot(v1)) > 0.99 && i < indices.size());

	//		normal = v1.cross(v2);
	//		if (normal.dot(normalish) < 0) {
	//			normal = -normal;
	//		}
	//	}

	//	void addNeighbor(Face* f) {

	//	}

	//	std::vector<Edge*> edges;
	//	std::vector<unsigned int> indices;
	//	mthz::Vec3 normal;
	//	bool flagged_for_remove;
	//};

	////naive brute force
	//ConvexPoly getConvexHull(const Mesh& m) {
	//	assert(m.vertices.size() >= 4);
	//	std::vector<Face*> faces;

	//	//setup inital tetrahedron
	//	mthz::Vec3 v0 = m.vertices[0], v1 = m.vertices[1], v2 = m.vertices[2], v3 = m.vertices[3];
	//	mthz::Vec3 center = (v0 + v1 + v2 + v3) / 4.0;

	//	faces.push_back(new Face(m.vertices, { 0, 1, 2 }, v0 - center));
	//	faces.push_back(new Face(m.vertices, { 1, 2, 3 }, v1 - center));
	//	faces.push_back(new Face(m.vertices, { 2, 3, 0 }, v2 - center));
	//	faces.push_back(new Face(m.vertices, { 3, 0, 1 }, v3 - center));
	//	/*faces[0]->neighbors = { faces[1], faces[2], faces[3] };
	//	faces[1]->neighbors = { faces[0], faces[2], faces[3] };
	//	faces[2]->neighbors = { faces[1], faces[0], faces[3] };
	//	faces[3]->neighbors = { faces[1], faces[2], faces[0] };*/

	//	for (int i = 4; i < m.vertices.size(); i++) {
	//		mthz::Vec3 v = m.vertices[i];
	//	}
	//}
}