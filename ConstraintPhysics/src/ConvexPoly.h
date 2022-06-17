#pragma once

#include "../../Math/src/Vec3.h"
#include "../../Math/src/Quaternion.h"
#include "../../Math/src/Mat3.h"
#include <vector>

namespace phyz {
	class Surface;
	class Edge;

	class ConvexPoly {
	public:
		ConvexPoly(const ConvexPoly& c);
		ConvexPoly() {}

		static ConvexPoly genRect(double x, double y, double z, double dx, double dy, double dz);
		static ConvexPoly genTetra(double x, double y, double z, double r);

		void rotate(const mthz::Quaternion q, mthz::Vec3 pivot_point);
		void translate(mthz::Vec3 t);
		void gen_interiorP();
		void compute_edges();

		std::vector<mthz::Vec3> points;
		std::vector<Surface> surfaces;
		std::vector<Edge> edges;
		mthz::Vec3 interior_point;

	};

	class Edge {
	public:
		Edge(int p1_indx, int p2_indx, ConvexPoly* poly);
		Edge(const Edge& e, ConvexPoly* poly);
		Edge();

		mthz::Vec3 p1() const;
		mthz::Vec3 p2() const;

		friend class ConvexPoly;
	private:
		ConvexPoly* poly;
		int p1_indx;
		int p2_indx;
	};

	class Surface {
	public:
		Surface(const std::vector<int>& point_indexes, ConvexPoly* poly, mthz::Vec3 interior_point);
		Surface(const Surface& s, ConvexPoly* poly);
		Surface();

		int n_points() const;
		mthz::Vec3 normal() const;
		mthz::Vec3 getPointI(int i) const;

		friend class ConvexPoly;
	private:
		ConvexPoly* poly;
		std::vector<int> point_indexes;
		int normalDirection;
	};
}