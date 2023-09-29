#pragma once

#include "../../Math/src/Vec3.h"
#include "../../Math/src/Quaternion.h"
#include "../../Math/src/Mat3.h"
#include "AABB.h"
#include "CollisionDetect.h"
#include "AABB_Tree.h"
#include <vector>

namespace phyz {
	class Surface;
	class Edge;
	class RigidBody;

	struct Material {
		const static Material default_material() { return {1.0, 0.3, 0.6, 1.1}; }
		static Material ice() { return { 0.6, 0.3, 0.01, 0.2 }; }
		static Material rubber() { return { 0.8, 0.6, 1.0, 1.4 }; }
		static Material high_friction()  { return {0.7, 0.3, 1.0, 2.0}; }
		static Material super_friction() { return { 0.8, 0.6, 40.0, 100.0 }; }
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

	enum ConvexGeometryType { POLYHEDRON, SPHERE, CYLINDER };
	class ConvexGeometry {
	public:
		virtual void recomputeFromReference(const ConvexGeometry& reference, const mthz::Mat3& rot, mthz::Vec3 trans) = 0;
		virtual AABB gen_AABB() const = 0;
		virtual ConvexGeometryType getType() const = 0;
	};

	struct RayQueryReturn {
		bool did_hit;
		mthz::Vec3 intersection_point;
		double intersection_dist;
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
		ConvexPrimitive getScaled(double d, mthz::Vec3 center_of_dialtion) const;
		inline AABB gen_AABB() const { return geometry->gen_AABB(); }
		inline void recomputeFromReference(const ConvexGeometry& reference, const mthz::Mat3& rot, mthz::Vec3 trans) { geometry->recomputeFromReference(reference, rot, trans); }
		inline int getID() const { return id; }

		RayQueryReturn testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) const;

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
		Sphere getScaled(double d, mthz::Vec3 center_of_dialtion) const;
		void recomputeFromReference(const ConvexGeometry& reference, const mthz::Mat3& rot, mthz::Vec3 trans) override;
		AABB gen_AABB() const override;
		ConvexGeometryType getType() const override { return SPHERE; };

		inline double getRadius() const { return radius; }
		inline mthz::Vec3 getCenter() const { return center; }

		RayQueryReturn testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir);

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

	class Cylinder : ConvexGeometry {
	public:
		Cylinder() {}
		Cylinder(const Cylinder& c);
		Cylinder(mthz::Vec3 center, double radius, double height, mthz::Vec3 height_axis, int edge_approximation_detail = 20);

		Cylinder getRotated(const mthz::Quaternion q, mthz::Vec3 pivot_point = mthz::Vec3(0, 0, 0)) const;
		Cylinder getTranslated(mthz::Vec3 t) const;
		Cylinder getScaled(double d, mthz::Vec3 center_of_dialtion) const;
		void recomputeFromReference(const ConvexGeometry& reference, const mthz::Mat3& rot, mthz::Vec3 trans) override;
		AABB gen_AABB() const override;
		ConvexGeometryType getType() const override { return CYLINDER; };

		inline double getRadius() const { return radius; }
		inline double getHeight() const { return height; }
		inline mthz::Vec3 getHeightAxis() const { return height_axis; }
		inline mthz::Vec3 getCenter() const { return center; }

		RayQueryReturn testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir);

		static mthz::Vec3 getExtremaOfDisk(mthz::Vec3 disk_center, mthz::Vec3 disk_normal, double radius, mthz::Vec3 target_direction);

		friend class Surface;
		friend class Edge;
		friend class RigidBody;
	private:

		std::vector<mthz::Vec3> gauss_verts;
		std::vector<GaussArc> gauss_arcs;

		mthz::Vec3 center;
		mthz::Vec3 height_axis;
		double radius;
		double height;
	};

	class Polyhedron : ConvexGeometry {
	public:
		Polyhedron() {}
		Polyhedron(const Polyhedron& c);
		Polyhedron(const std::vector<mthz::Vec3>& points, const std::vector<std::vector<int>>& surface_vertex_indices);

		//polyhedrons don't work well as colliders if they have coplanar faces. This method
		//is to get a cleaned version of a polyhedron if it was fed data that might contain coplanar faces
		//note will not work if polyhedron in concave (but Polyhedron shouldn't be concave in the first place)
		static Polyhedron getPolyAfterFindMergedCoplanarFaces(const Polyhedron& p);

		Polyhedron getRotated(const mthz::Quaternion q, mthz::Vec3 pivot_point = mthz::Vec3(0, 0, 0)) const;
		Polyhedron getTranslated(mthz::Vec3 t) const;
		Polyhedron getScaled(double d, mthz::Vec3 center_of_dialtion) const;
		void recomputeFromReference(const ConvexGeometry& reference, const mthz::Mat3& rot, mthz::Vec3 trans) override;
		AABB gen_AABB() const override;
		ConvexGeometryType getType() const override { return POLYHEDRON; };

		inline const std::vector<mthz::Vec3>& getPoints() const { return points; }
		inline const std::vector<Surface>& getSurfaces() const { return surfaces; }
		inline const std::vector<Edge>& getEdges() const { return edges; }
		inline const std::vector<int>& getFaceIndicesAdjacentToPointI(int i) const { return adjacent_faces_to_vertex[i]; }
		inline const std::vector<int>& getEdgeIndicesAdjacentToPointI(int i) const { return adjacent_edges_to_vertex[i]; }
		inline const GaussMap& getGaussMap() const { return gauss_map; }

		RayQueryReturn testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir);

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
	public: mthz::Vec3 interior_point;
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