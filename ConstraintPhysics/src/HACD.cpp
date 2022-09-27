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

	//namespace qkhl {
	//	struct Face;

	//	struct HalfEdge {
	//		int p1, p2;
	//		Face* face;
	//		HalfEdge* twin;
	//	};

	//	class Face {
	//	public:
	//		Face(const std::vector<mthz::Vec3>& global_points, std::vector<, HalfEdge* h1_twin, HalfEdge* h2_twin, HalfEdge* h3_twin)
	//			: p1(p1), p2(p2), p3(p3)
	//		{
	//			n = (points[p2] - points[p1]).cross(points[p3] - points[p2]).normalize();

	//			h1 = new HalfEdge{ p1, p2, this, h1_twin };
	//			h2 = new HalfEdge{ p2, p3, this, h2_twin };
	//			h3 = new HalfEdge{ p3, p1, this, h3_twin };

	//			if (h1_twin != nullptr) h1_twin->twin = h1;
	//			if (h2_twin != nullptr) h2_twin->twin = h2;
	//			if (h3_twin != nullptr) h3_twin->twin = h3;
	//		}

	//		~Face() {
	//			if (h1->twin != nullptr) h1->twin->twin = nullptr;
	//			if (h2->twin != nullptr) h2->twin->twin = nullptr;
	//			if (h3->twin != nullptr) h3->twin->twin = nullptr;

	//			delete h1;
	//			delete h2;
	//			delete h3;
	//		}


	//		int p1, p2, p3;
	//		HalfEdge* h1; HalfEdge* h2; HalfEdge* h3;
	//		mthz::Vec3 n;
	//	};

	//	struct MinMaxIndx {
	//		int min, max;
	//	};

	//	MinMaxIndx minMaxDirectionQuery(const std::vector<mthz::Vec3>& points, mthz::Vec3 direction, std::vector<int> excluded_indxs = {}) {
	//		MinMaxIndx out {-1, -1};
	//		double min_val = std::numeric_limits<double>::infinity();
	//		double max_val = -std::numeric_limits<double>::infinity();

	//		for (int i = 0; i < points.size(); i++) {
	//			bool skip = false;
	//			for (int excluded : excluded_indxs) {
	//				if (excluded == i) {
	//					skip = true;
	//					break;
	//				}
	//			}
	//			if (skip) break;

	//			double value = direction.dot(points[i]);
	//			if (value < min_val) {
	//				min_val = value;
	//				out.min = i;
	//			}
	//			if (value > max_val) {
	//				max_val = value;
	//				out.max = i;
	//			}
	//		}

	//		return out;
	//	}
	//}	

	//ConvexPoly getConvexHull(const Mesh& m) {
	//	//**********************************
	//	//****Create Initial Tetrahedron****
	//	//**********************************
	//	qkhl::MinMaxIndx p1p2segment = qkhl::minMaxDirectionQuery(m.vertices, mthz::Vec3(1, 0, 0));
	//	int p1 = p1p2segment.min;
	//	int p2 = p1p2segment.max;

	//	mthz::Vec3 u, w;
	//	(m.vertices[p2] - m.vertices[p1]).getPerpendicularBasis(&u, &w);
	//	double p1u = m.vertices[p1].dot(u);
	//	double p1w = m.vertices[p1].dot(w);
	//	double max_val = 0;
	//	int p3 = -1;
	//	for (int i = 0; i < m.vertices.size(); i++) {
	//		double u_dist = m.vertices[i].dot(u) - p1u;
	//		double w_dist = m.vertices[i].dot(w) - p1w;

	//		double value = u_dist * u_dist + w_dist * w_dist;
	//		if (value > max_val) {
	//			max_val = value;
	//			p3 = i;
	//		}
	//	}

	//	mthz::Vec3 n = (m.vertices[p2] - m.vertices[p1]).cross(m.vertices[p3] - m.vertices[p2]);
	//	double p1n = m.vertices[p1].dot(n);

	//	qkhl::MinMaxIndx p4Candidates = qkhl::minMaxDirectionQuery(m.vertices, n, { p1, p2, p3 });
	//	int p4 = p4Candidates.min;
	//	if (m.vertices[p4Candidates.max].dot(n) - p1n > p1n - m.vertices[p4Candidates.min].dot(n)) {
	//		p4 = p4Candidates.max;
	//		//keep winding counterclockwise
	//		int tmp = p2;
	//		p2 = p3;
	//		p3 = tmp;
	//	}

	//	qkhl::Face* f1 = new qkhl::Face(m.vertices, p1, p2, p3, nullptr, nullptr, nullptr);
	//	qkhl::Face* f2 = new qkhl::Face(m.vertices, p1, p2, p4, f1->h1, nullptr, nullptr);
	//	qkhl::Face* f3 = new qkhl::Face(m.vertices, p2, p3, p4, f1->h2, nullptr, f2->h2);
	//	qkhl::Face* f4 = new qkhl::Face(m.vertices, p3, p1, p4, f1->h3, f2->h3, f3->h2);

	//	
	//	
	//}

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