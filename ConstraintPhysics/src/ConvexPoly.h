#pragma once

#include "../../Math/src/Vec3.h"
#include "../../Math/src/Quaternion.h"
#include "../../Math/src/Mat3.h"
#include "AABB.h"
#include <vector>

namespace phyz {
	class Surface;
	class Edge;

	//this class needs a proper constructor this is really dumb
	class ConvexPoly {
	public:
		ConvexPoly(const ConvexPoly& c);
		ConvexPoly();

		static ConvexPoly genRect(double x, double y, double z, double dx, double dy, double dz);
		static ConvexPoly genTetra(double x, double y, double z, double r);

		void rotate(const mthz::Quaternion q, mthz::Vec3 pivot_point);
		void translate(mthz::Vec3 t);
		void gen_interiorP();
		AABB gen_AABB() const;
		void compute_edges();
		void assign_IDs();

		std::vector<mthz::Vec3> points;
		std::vector<Surface> surfaces;
		std::vector<Edge> edges;
		mthz::Vec3 interior_point;
		int id;

	};

	class Edge {
	public:
		Edge(int p1_indx, int p2_indx, ConvexPoly* poly);
		Edge(const Edge& e, ConvexPoly* poly);
		Edge();

		mthz::Vec3 p1() const;
		mthz::Vec3 p2() const;

		int p1_indx;
		int p2_indx;
		friend class ConvexPoly;
	private:
		ConvexPoly* poly;
		
	};

	class Surface {
	public:
		Surface(const std::vector<int>& point_indexes, ConvexPoly* poly, mthz::Vec3 interior_point);
		Surface(const Surface& s, ConvexPoly* poly);
		Surface();

		int n_points() const;
		mthz::Vec3 normal() const;
		mthz::Vec3 getPointI(int i) const;

		std::vector<int> point_indexes;
		friend class ConvexPoly;
	private:
		ConvexPoly* poly;
		int surfaceID;
		int normalDirection;
	};

	struct GaussArc {
		unsigned int v1_indx;
		unsigned int v2_indx;
	};

	struct GaussMap {
		std::vector<mthz::Vec3> face_verts;
		std::vector<GaussArc> arcs;
	};

	GaussMap computeGaussMap(const ConvexPoly& c);
}