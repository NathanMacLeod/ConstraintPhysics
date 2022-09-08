#pragma once

#include "../../Math/src/Vec3.h"
#include "../../Math/src/Quaternion.h"
#include "../../Math/src/Mat3.h"
#include "AABB.h"
#include "CollisionDetect.h"
#include <vector>

namespace phyz {
	class Surface;
	class Edge;
	struct GaussMap;
	class RigidBody;

	struct Material {
		const static Material default_material() { return {1.0, 0.3, 0.6, 1.1}; }
		static Material ice() { return { 0.6, 0.3, 0.1, 0.2 }; }
		static Material rubber() { return { 0.8, 0.6, 1.0, 1.4 }; }
		const static Material modified_density(double d) {
			Material m = default_material();
			m.density = d;
			return m;
		}

		double density;
		double restitution;
		double kinetic_friction_coeff;
		double static_friction_coeff;
	};

	class ConvexPoly {
	public:
		ConvexPoly() {}
		ConvexPoly(const ConvexPoly& c);
		ConvexPoly(const std::vector<mthz::Vec3>& points, const std::vector<std::vector<int>>& surface_vertex_indices, Material material=Material::default_material());

		GaussMap computeGaussMap() const;
		ConvexPoly getRotated(const mthz::Quaternion q, mthz::Vec3 pivot_point=mthz::Vec3(0,0,0)) const;
		ConvexPoly getTranslated(mthz::Vec3 t) const;
		AABB gen_AABB() const;
		inline const std::vector<mthz::Vec3>& getPoints() const { return points; }
		inline const std::vector<Surface>& getSurfaces() const { return surfaces; }
		inline const std::vector<Edge>& getEdges() const { return edges; }
		inline const std::vector<int>& getFaceIndicesAdjacentToPointI(int i) const { return adjacent_faces_to_vertex[i]; }
		inline const std::vector<int>& getEdgeIndicesAdjacentToPointI(int i) const { return adjacent_edges_to_vertex[i]; }
		inline int getID() const { return id; }

		Material material;

		friend class Surface;
		friend class Edge;
		friend class RigidBody;
	private:
		std::vector<std::vector<int>> adjacent_faces_to_vertex;
		std::vector<std::vector<int>> adjacent_edges_to_vertex;
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

		inline mthz::Vec3 p1() const { return poly->points[p1_indx]; }
		inline mthz::Vec3 p2() const { return poly->points[p2_indx]; }

		int p1_indx;
		int p2_indx;
		friend class ConvexPoly;
	private:
		ConvexPoly* poly;
		
	};


	class Surface {
	public:
		Surface(const std::vector<int>& point_indexes, ConvexPoly* poly, mthz::Vec3 interior_point, int surfaceID = -1);
		Surface(const Surface& s, ConvexPoly* poly);
		Surface();

		int n_points() const;
		mthz::Vec3 normal() const;
		mthz::Vec3 getPointI(int i) const;
		inline int getSurfaceID() const { return surfaceID; }

		std::vector<int> point_indexes;
		friend class ConvexPoly;
	private:
		void findNormalCalcInfo(const mthz::Vec3& normalish);
		void setWindingAntiClockwise(const mthz::Vec3& normal);

		ConvexPoly* poly;
		int normal_calc_index;
		int surfaceID;
		int normalDirection;
	};

	struct GaussArc {
		unsigned int v1_indx;
		unsigned int v2_indx;
	};

	struct GaussVert {
		mthz::Vec3 v;
		bool SAT_redundant;
		ExtremaInfo cached_SAT_query;
		int SAT_reference_point_index;
		double SAT_reference_point_value;
	};

	struct GaussMap {
		std::vector<GaussVert> face_verts;
		std::vector<GaussArc> arcs;
	};
}