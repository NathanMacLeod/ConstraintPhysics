#include "ConvexPoly.h"

#include <limits>
#include <algorithm>

namespace phyz {

	ConvexPoly::ConvexPoly(const ConvexPoly& c)
		: points(c.points)
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

	void ConvexPoly::gen_interiorP() {
		mthz::Vec3 avg;
		for (const mthz::Vec3 p : points) {
			avg += p;
		}
		interior_point = avg / points.size();
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
		:point_indexes(s.point_indexes), normalDirection(s.normalDirection)
	{
		this->poly = poly;
	}

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

}