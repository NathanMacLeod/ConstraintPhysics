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

	enum ConvexGeometryType { POLYHEDRON, SPHERE };
	class ConvexGeometry {
	public:
		virtual void recomputeFromReference(const ConvexGeometry& reference, const mthz::Mat3& rot, mthz::Vec3 trans) = 0;
		virtual AABB gen_AABB() const = 0;
		virtual ConvexGeometryType getType() const = 0;
	};

	class ConvexPrimitive {
	public:
		ConvexPrimitive() {}
		ConvexPrimitive(const ConvexPrimitive& c);
		ConvexPrimitive(const ConvexGeometry& geometry_primitive, Material material=Material::default_material());
		~ConvexPrimitive() { if (geometry) delete geometry; }

		inline ConvexGeometryType getType() const { return type; }
		const ConvexGeometry* getGeometry() const { return geometry; }

		ConvexPrimitive getRotated(const mthz::Quaternion q, mthz::Vec3 pivot_point=mthz::Vec3(0,0,0)) const;
		ConvexPrimitive getTranslated(mthz::Vec3 t) const;
		inline AABB gen_AABB() const { return geometry->gen_AABB(); }
		inline void recomputeFromReference(const ConvexGeometry& reference, const mthz::Mat3& rot, mthz::Vec3 trans) { geometry->recomputeFromReference(reference, rot, trans); }
		inline int getID() const { return id; }

		Material material;

	private:
		
		ConvexGeometryType type;
		ConvexGeometry* geometry = nullptr;
		int id;
	};

	class Sphere : ConvexGeometry {
	public:
		Sphere() {}
		Sphere(const Sphere& c);
		Sphere(mthz::Vec3 center, double radius);

		Sphere getRotated(const mthz::Quaternion q, mthz::Vec3 pivot_point = mthz::Vec3(0, 0, 0)) const;
		Sphere getTranslated(mthz::Vec3 t) const;
		void recomputeFromReference(const ConvexGeometry& reference, const mthz::Mat3& rot, mthz::Vec3 trans) override;
		AABB gen_AABB() const override;
		ConvexGeometryType getType() const override { return SPHERE; };

		inline double getRadius() const { return radius; }
		inline mthz::Vec3 getCenter() const { return center; }

		friend class Surface;
		friend class Edge;
		friend class RigidBody;
	private:

		mthz::Vec3 center;
		double radius;
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

	class Polyhedron : ConvexGeometry {
	public:
		Polyhedron() {}
		Polyhedron(const Polyhedron& c);
		Polyhedron(const std::vector<mthz::Vec3>& points, const std::vector<std::vector<int>>& surface_vertex_indices);

		Polyhedron getRotated(const mthz::Quaternion q, mthz::Vec3 pivot_point = mthz::Vec3(0, 0, 0)) const;
		Polyhedron getTranslated(mthz::Vec3 t) const;
		void recomputeFromReference(const ConvexGeometry& reference, const mthz::Mat3& rot, mthz::Vec3 trans) override;
		AABB gen_AABB() const override;
		ConvexGeometryType getType() const override { return POLYHEDRON; };

		inline const std::vector<mthz::Vec3>& getPoints() const { return points; }
		inline const std::vector<Surface>& getSurfaces() const { return surfaces; }
		inline const std::vector<Edge>& getEdges() const { return edges; }
		inline const std::vector<int>& getFaceIndicesAdjacentToPointI(int i) const { return adjacent_faces_to_vertex[i]; }
		inline const std::vector<int>& getEdgeIndicesAdjacentToPointI(int i) const { return adjacent_edges_to_vertex[i]; }
		inline const GaussMap& getGaussMap() const { return gauss_map; }

		friend class Surface;
		friend class Edge;
		friend class RigidBody;
	private:

		GaussMap computeGaussMap() const;
		GaussMap gauss_map;

		std::vector<std::vector<int>> adjacent_faces_to_vertex;
		std::vector<std::vector<int>> adjacent_edges_to_vertex;
		std::vector<mthz::Vec3> points;
		std::vector<Surface> surfaces;
		std::vector<Edge> edges;
		mthz::Vec3 interior_point;
	};

	class Edge {
	public:
		Edge(int p1_indx, int p2_indx, Polyhedron* poly);
		Edge(const Edge& e, Polyhedron* poly);
		Edge();

		inline mthz::Vec3 p1() const { return poly->points[p1_indx]; }
		inline mthz::Vec3 p2() const { return poly->points[p2_indx]; }

		int p1_indx;
		int p2_indx;
		friend class Polyhedron;
	private:
		Polyhedron* poly;
		
	};

	class Surface {
	public:
		Surface(const std::vector<int>& point_indexes, Polyhedron* poly, mthz::Vec3 interior_point, int surfaceID = -1);
		Surface(const Surface& s, Polyhedron* poly);
		Surface();

		int n_points() const;
		mthz::Vec3 normal() const;
		mthz::Vec3 getPointI(int i) const;
		inline int getSurfaceID() const { return surfaceID; }

		std::vector<int> point_indexes;
		friend class Polyhedron;
	private:
		void findNormalCalcInfo(const mthz::Vec3& normalish);
		void setWindingAntiClockwise(const mthz::Vec3& normal);

		Polyhedron* poly;
		int normal_calc_index;
		int surfaceID;
		int normalDirection;
	};

}