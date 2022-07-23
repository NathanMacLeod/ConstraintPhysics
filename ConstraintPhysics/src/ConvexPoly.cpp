#include "ConvexPoly.h"

#include <limits>
#include <algorithm>

namespace phyz {

	ConvexPoly::ConvexPoly(const ConvexPoly& c)
		: points(c.points), interior_point(c.interior_point)
	{
		for (int i = 0; i < c.surfaces.size(); i++) {
			surfaces.push_back(Surface(c.surfaces[i], this));
		}
		for (int i = 0; i < c.edges.size(); i++) {
			edges.push_back(Edge(c.edges[i], this));
		}
	}

	static struct Pair {
		int p1;
		int p2;

		bool operator==(const Pair p) {
			return (p1 == p.p1 && p2 == p.p2) || (p2 == p.p1 && p1 == p.p2);
		}
	};

	void ConvexPoly::compute_edges() {
		std::vector<Pair> seen_pairs;
		for (const Surface& s : surfaces) {
			int n = s.point_indexes.size();
			for (int i = 0; i < n; i++) {
				Pair p = Pair{ s.point_indexes[i], s.point_indexes[(i + 1) % n] };

				bool seen = false;
				for (const Pair pair : seen_pairs) {
					if (p == pair) {
						seen = true;
						break;
					}
				}

				if (!seen) {
					seen_pairs.push_back(p);
				}
			}
		}

		edges = std::vector<Edge>(seen_pairs.size());
		for (int i = 0; i < seen_pairs.size(); i++) {
			Pair pair = seen_pairs[i];
			edges[i] = Edge(pair.p1, pair.p2, this);
		}
	}

	//get point of minimum radius
	void ConvexPoly::gen_interiorP() {
		if (points.size() < 2) {
			return;
		}

		double maxDiam = 0;
		int maxD_i, maxD_j;
		for (int i = 0; i < points.size(); i++) {
			for (int j = i + 1; j < points.size(); j++) {
				double diameter = (points[i] - points[j]).magSqrd();
				if (diameter > maxDiam) {
					maxDiam = diameter;
					maxD_i = i;
					maxD_j = j;
				}
			}
		}
		interior_point = (points[maxD_i] + points[maxD_j]) / 2.0;
	}

	AABB ConvexPoly::gen_AABB() const {
		const double inf = std::numeric_limits<double>::infinity();
		mthz::Vec3 min(inf, inf, inf);
		mthz::Vec3 max(-inf, -inf, -inf);

		for (const mthz::Vec3& p : points) {
			min.x = std::min<double>(min.x, p.x);
			min.y = std::min<double>(min.y, p.y);
			min.z = std::min<double>(min.z, p.z);

			max.x = std::max<double>(max.x, p.x);
			max.y = std::max<double>(max.y, p.y);
			max.z = std::max<double>(max.z, p.z);
		}

		return { min, max };
	}

	void ConvexPoly::rotate(const mthz::Quaternion q, mthz::Vec3 pivot_point) {
		mthz::Mat3 rotMat = q.getRotMatrix();
		for (int i = 0; i < points.size(); i++) {
			points[i] = pivot_point + rotMat * (points[i] - pivot_point);
		}
		interior_point = pivot_point + rotMat * (interior_point - pivot_point);
	}

	void ConvexPoly::translate(mthz::Vec3 t) {
		for (int i = 0; i < points.size(); i++) {
			points[i] += t;
		}
		interior_point += t;
	}

	ConvexPoly ConvexPoly::genRect(double x, double y, double z, double dx, double dy, double dz) {
		ConvexPoly out;

		out.points = std::vector<mthz::Vec3>(8);
		out.surfaces = std::vector<Surface>(6);

		out.points[0] = (mthz::Vec3(x, y, z)); //0
		out.points[1] = (mthz::Vec3(x + dx, y, z)); //1
		out.points[2] = (mthz::Vec3(x + dx, y + dy, z)); //2
		out.points[3] = (mthz::Vec3(x, y + dy, z)); //3

		out.points[4] = (mthz::Vec3(x, y, z + dz)); //4
		out.points[5] = (mthz::Vec3(x + dx, y, z + dz)); //5
		out.points[6] = (mthz::Vec3(x + dx, y + dy, z + dz)); //6
		out.points[7] = (mthz::Vec3(x, y + dy, z + dz)); //7

		mthz::Vec3 mid = mthz::Vec3(x + dx / 2.0, y + dy / 2.0, z + dz / 2.0);
		std::vector<int> indexes = std::vector<int>(4);

		indexes[0] = 0; indexes[1] = 3; indexes[2] = 2; indexes[3] = 1;
		out.surfaces[0] = (Surface(indexes, &out, mid));

		indexes[0] = 0; indexes[1] = 1; indexes[2] = 5; indexes[3] = 4;
		out.surfaces[1] = (Surface(indexes, &out, mid));

		indexes[0] = 1; indexes[1] = 2; indexes[2] = 6; indexes[3] = 5;
		out.surfaces[2] = (Surface(indexes, &out, mid));

		indexes[0] = 2; indexes[1] = 3; indexes[2] = 7; indexes[3] = 6;
		out.surfaces[3] = (Surface(indexes, &out, mid));

		indexes[0] = 3; indexes[1] = 0; indexes[2] = 4; indexes[3] = 7;
		out.surfaces[4] = (Surface(indexes, &out, mid));

		indexes[0] = 4; indexes[1] = 5; indexes[2] = 6; indexes[3] = 7;
		out.surfaces[5] = (Surface(indexes, &out, mid));

		out.interior_point = mid;
		out.compute_edges();

		return out;
	}

	ConvexPoly ConvexPoly::genTetra(double x, double y, double z, double r) {
		ConvexPoly out;

		const double rt3 = sqrt(3.0);

		out.points = std::vector<mthz::Vec3>(4);
		out.surfaces = std::vector<Surface>(4);

		out.points[0] = (mthz::Vec3(x + 3*r/4, y - r/3, z - rt3*r/4)); //0
		out.points[1] = (mthz::Vec3(x, y - r/3, z + rt3*r/2)); //1
		out.points[2] = (mthz::Vec3(x - 3*r/4, y - r/3, z - rt3*r/4)); //2
		out.points[3] = (mthz::Vec3(x, y + r, z)); //3

		mthz::Vec3 mid = mthz::Vec3(x, y, z);
		std::vector<int> indexes = std::vector<int>(3);

		indexes[0] = 0; indexes[1] = 1; indexes[2] = 2;
		out.surfaces[0] = (Surface(indexes, &out, mid));

		indexes[0] = 0; indexes[1] = 3; indexes[2] = 1;
		out.surfaces[1] = (Surface(indexes, &out, mid));

		indexes[0] = 0; indexes[1] = 2; indexes[2] = 3;
		out.surfaces[2] = (Surface(indexes, &out, mid));

		indexes[0] = 2; indexes[1] = 1; indexes[2] = 3;
		out.surfaces[3] = (Surface(indexes, &out, mid));

		out.interior_point = mid;
		out.compute_edges();

		return out;
	}

	Edge::Edge(int p1_indx, int p2_indx, ConvexPoly* poly) {
		this->poly = poly;
		this->p1_indx = p1_indx;
		this->p2_indx = p2_indx;
	}

	Edge::Edge(const Edge& e, ConvexPoly* poly) {
		this->poly = poly;
		p1_indx = e.p1_indx;
		p2_indx = e.p2_indx;
	}
	Edge::Edge() {
		p1_indx = 0;
		p2_indx = 0;
		poly = nullptr;
	}

	mthz::Vec3 Edge::p1() const {
		return poly->points[p1_indx];
	}

	mthz::Vec3 Edge::p2() const {
		return poly->points[p2_indx];
	}

	Surface::Surface(const std::vector<int>& point_indexes, ConvexPoly* poly, mthz::Vec3 interior_point)
		: point_indexes(point_indexes), poly(poly)
	{
		if (point_indexes.size() < 3) {
			printf("ERROR: Trying to initialize surface with fewer than 3 points\n");
		}

		normalDirection = 1; //initial guess
		mthz::Vec3 norm = normal();
		mthz::Vec3 outward = poly->points[point_indexes[0]] - interior_point;
		if (outward.dot(norm) < 0) {
			normalDirection = -1;
		}
	}

	Surface::Surface(const Surface& s, ConvexPoly* poly) 
		:point_indexes(s.point_indexes), normalDirection(s.normalDirection), poly(poly)
	{}

	Surface::Surface() {
		poly = nullptr;
	}

	mthz::Vec3 Surface::normal() const {
		mthz::Vec3 p1 = poly->points[point_indexes[0]];
		mthz::Vec3 p2 = poly->points[point_indexes[1]];
		mthz::Vec3 p3 = poly->points[point_indexes[2]];

		return (p2 - p1).cross(p3 - p2).normalize() * normalDirection;
	}

	int Surface::n_points() const {
		return point_indexes.size();
	}


	mthz::Vec3 Surface::getPointI(int i) const {
		if (i < 0 || i >= point_indexes.size()) {
			printf("ERROR: Trying to access invalid index of surface: %d, valid range [0, %d]\n", i, point_indexes.size());
		}

		return poly->points[point_indexes[i]];
	}

	GaussMap computeGaussMap(const ConvexPoly& c) {
		GaussMap g;
		for (int i = 0; i < c.surfaces.size(); i++) {
			const Surface& s1 = c.surfaces[i];
			g.face_verts.push_back(s1.normal());

			for (int j = i + 1; j < c.surfaces.size(); j++) {
				const Surface& s2 = c.surfaces[j];

				for (int k = 0; k < s1.n_points(); k++) {
					for (int w = 0; w < s2.n_points(); w++) {
						mthz::Vec3 x1 = s1.getPointI(k);
						mthz::Vec3 x2 = s1.getPointI((k + 1) % s1.n_points());
						mthz::Vec3 y1 = s2.getPointI(w);
						mthz::Vec3 y2 = s2.getPointI((w + 1) % s2.n_points());

						//if si and sj share an edge, there exists an arc between v1 and v2 on the gauss map
						if ((x1 == y1 && x2 == y2) || (x1 == y2 && x2 == y1)) {
							g.arcs.push_back({ (unsigned int)i, (unsigned int)j });
						}
					}
				}
			}
		}
		return g;
	}

}