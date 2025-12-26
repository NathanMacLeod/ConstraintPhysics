#pragma once

#include "../../Math/src/Vec3.h"
#include "../../Math/src/Quaternion.h"
#include "../../Math/src/Mat3.h"
#include "CFM.h"
#include "AABB.h"
#include "CollisionDetect.h"
#include "AABB_Tree.h"
#include <vector>

namespace phyz {
	class Surface;
	class Edge;
	class RigidBody;

	struct Material {
		static Material default_material() { return { CFM{USE_GLOBAL}, 1.0, 0.3, 0.6, 1.1 }; }
		static Material ice() { return { CFM{USE_GLOBAL}, 0.6, 0.3, 0.01, 0.2 }; }
		static Material rubber() { return { CFM{USE_GLOBAL}, 0.8, 0.6, 1.0, 1.4 }; }
		static Material high_friction()  { return { CFM{USE_GLOBAL}, 0.7, 0.3, 1.0, 2.0}; }
		static Material super_friction() { return { CFM{USE_GLOBAL}, 0.8, 0.6, 40.0, 100.0 }; }
		static Material modified_density(double d) {
			Material m = default_material();
			m.density = d;
			return m;
		}

		CFM cfm;
		double density;
		double restitution;
		double kinetic_friction_coeff;
		double static_friction_coeff;
	};

	enum ConvexGeometryType { POLYHEDRON, SPHERE, CYLINDER };
	class ConvexGeometry {
	public:
		virtual ~ConvexGeometry() {}
		virtual void recomputeFromReference(const ConvexGeometry& reference, const mthz::Mat3& rot, mthz::Vec3 trans) = 0;
		virtual AABB gen_AABB() const = 0;
		virtual ConvexGeometryType getType() const = 0;
	};

	struct RayQueryReturn {
		bool did_hit;
		mthz::Vec3 intersection_point;
		mthz::Vec3 surface_norm;
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
		uint32_t SAT_reference_point_index;
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
		Cylinder(mthz::Vec3 center, double radius, double height, mthz::Vec3 height_axis = mthz::Vec3(0, 1, 0), uint32_t edge_approximation_detail = 12, uint32_t face_approximation_detail = 8);

		Cylinder getRotated(const mthz::Quaternion q, mthz::Vec3 pivot_point = mthz::Vec3(0, 0, 0)) const;
		Cylinder getTranslated(mthz::Vec3 t) const;
		Cylinder getScaled(double d, mthz::Vec3 center_of_dialtion) const;
		void recomputeFromReference(const ConvexGeometry& reference, const mthz::Mat3& rot, mthz::Vec3 trans) override;
		AABB gen_AABB() const override;
		ConvexGeometryType getType() const override { return CYLINDER; };

		inline double getRadius() const { return radius; }
		inline double getHeight() const { return height; }
		inline int getTopApproxPointIDOffset() const { return 0; }
		inline int getBotApproxPointIDOffset() const { return static_cast<int>(top_face_approximation.size()); }
		inline int getTopSurfaceID() const { return 2 * static_cast<int>(top_face_approximation.size()); }
		inline int getBotSurfaceID() const { return 2 * static_cast<int>(top_face_approximation.size()) + 1; }
		inline int getTopEdgeID() const { return 2 * static_cast<int>(top_face_approximation.size()) + 2; }
		inline int getBotEdgeID() const { return 2 * static_cast<int>(top_face_approximation.size()) + 3; }
		inline mthz::Vec3 getHeightAxis() const { return height_axis; }
		inline mthz::Vec3 getCenter() const { return center; }
		inline mthz::Vec3 getTopDiskCenter() const { return center + 0.5 * height * height_axis; }
		inline mthz::Vec3 getBotDiskCenter() const { return center - 0.5 * height * height_axis; }
		inline const std::vector<mthz::Vec3>& getTopFaceApprox() const { return top_face_approximation; }
		inline const std::vector<mthz::Vec3>& getBotFaceApprox() const { return bot_face_approximation; }
		inline const std::vector<mthz::Vec3>& getGuassVerts() const { return gauss_verts; }
		inline const std::vector<GaussArc>& getGuassArcs() const { return gauss_arcs; }

		RayQueryReturn testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir);

		static mthz::Vec3 getExtremaOfDisk(mthz::Vec3 disk_center, mthz::Vec3 disk_normal, double radius, mthz::Vec3 target_direction);

		friend class Surface;
		friend class Edge;
		friend class RigidBody;
	private:
		std::vector<mthz::Vec3> top_face_approximation;
		std::vector<mthz::Vec3> bot_face_approximation;
		std::vector<mthz::Vec3> gauss_verts;
		std::vector<GaussArc> gauss_arcs;

		mthz::Vec3 center;
		mthz::Vec3 height_axis;
		double radius;
		double height;
	};

	class Surface {
	public:
		Surface(const std::vector<uint32_t>& point_indexes, Polyhedron* poly, mthz::Vec3 interior_point, int32_t surfaceID = -1);
		Surface(const Surface& s, Polyhedron* poly);
		Surface();

		uint32_t n_points() const;
		mthz::Vec3 normal() const;
		mthz::Vec3 getPointI(int i) const;
		inline int32_t getSurfaceID() const { return surfaceID; }

		std::vector<uint32_t> point_indexes;
		friend class Polyhedron;
	private:
		void findNormalCalcInfo(const mthz::Vec3& normalish);
		void setWindingAntiClockwise(const mthz::Vec3& normal);

		Polyhedron* poly;
		int normal_calc_index;
		int surfaceID;
		int normalDirection;
	};

	class Edge {
	public:
		Edge(int p1_indx, int p2_indx, Polyhedron* poly);
		Edge(const Edge& e, Polyhedron* poly);
		Edge();

		mthz::Vec3 p1() const;
		mthz::Vec3 p2() const;

		int p1_indx;
		int p2_indx;
		friend class Polyhedron;
	private:
		Polyhedron* poly;

	};

	class Polyhedron : ConvexGeometry {
	public:
		Polyhedron() {}
		Polyhedron(const Polyhedron& c);
		Polyhedron(const std::vector<mthz::Vec3>& points, const std::vector<std::vector<uint32_t>>& surface_vertex_indices);

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

}