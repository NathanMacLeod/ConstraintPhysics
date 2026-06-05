#pragma once
#include "ConvexPrimitive.h"
#include "AABB_Tree.h"
#include "HACD.h"
#include <array>

class DebugDemo;

namespace phyz {
	class RigidBody;

	class ConvexUnionGeometry {
	public:
		ConvexUnionGeometry() {}
		ConvexUnionGeometry(const ConvexPrimitive& c) : polyhedra({ c }) {}
		ConvexUnionGeometry(const std::initializer_list<ConvexUnionGeometry>& g);

		static ConvexUnionGeometry box(mthz::Vec3 pos, double dx, double dy, double dz, Material material = Material::default_material());
		static ConvexUnionGeometry sphere(mthz::Vec3 center, double radius, Material material = Material::default_material());
		static ConvexUnionGeometry capsule(mthz::Vec3 bot_sphere_center, double radius, double drum_height, Material material = Material::default_material());
		static ConvexUnionGeometry cylinder(mthz::Vec3 pos, double radius, double height, Material material = Material::default_material());
		static ConvexUnionGeometry psuedoSphere(mthz::Vec3 center, double radius, int n_rows = 15, int n_cols = 20, Material material = Material::default_material());
		static ConvexUnionGeometry tetra(mthz::Vec3 p1, mthz::Vec3 p2, mthz::Vec3 p3, mthz::Vec3 p4, Material material = Material::default_material());
		static ConvexUnionGeometry octahedron(mthz::Vec3 pos, double radius, Material material = Material::default_material());
		static ConvexUnionGeometry regDodecahedron(mthz::Vec3 pos, double size, Material material = Material::default_material());
		static ConvexUnionGeometry stellatedDodecahedron(mthz::Vec3 pos, double size, double spike_length_ratio, Material = Material::default_material());
		static ConvexUnionGeometry triPrism(double x1, double z1, double x2, double z2, double x3, double z3, double y, double height, Material material = Material::default_material());
		static ConvexUnionGeometry polyCylinder(mthz::Vec3 pos, double radius, double height, uint32_t detail = 10, Material material = Material::default_material());
		static ConvexUnionGeometry polyCapsule(mthz::Vec3 pos, double radius, double drum_height, uint32_t detail = 10, Material material = Material::default_material());
		static ConvexUnionGeometry ring(mthz::Vec3 pos, double inner_radius, double outter_radius, double height, int detail = 4, Material material = Material::default_material());
		static ConvexUnionGeometry gear(mthz::Vec3 pos, double radius, double tooth_length, double height, int n_teeth, bool parity = false, Material material = Material::default_material(), double tooth_width = -1);
		static ConvexUnionGeometry bevelGear(mthz::Vec3 pos, double radius, double tooth_radius, double tooth_width, double tooth_height, double height, int n_teeth, bool parity = false, Material material = Material::default_material(), double hole_radius = 0.0, int circle_detail = 7);
		static ConvexUnionGeometry pinion(mthz::Vec3 pos, double height, double width, double tooth_height, double tooth_width, double gap_width, int n_teeth, Material material = Material::default_material());
		static ConvexUnionGeometry uShape(mthz::Vec3 pos, double inner_radius, double outer_radius, double height, int n_segments = 8, Material material = Material::default_material());
		static ConvexUnionGeometry funnel(mthz::Vec3 pos, double tube_radius, double tube_height, double bowl_radius, double bowl_angle, double thickness, int n_segments = 15, Material material = Material::default_material());

		static ConvexUnionGeometry merge(const ConvexUnionGeometry& g1, const ConvexUnionGeometry& g2);
		ConvexUnionGeometry getNewMaterial(Material material);
		ConvexUnionGeometry getTranslated(const mthz::Vec3 v) const;
		ConvexUnionGeometry getRotated(const mthz::Quaternion q, const mthz::Vec3& rot_point=mthz::Vec3(0, 0, 0)) const;
		ConvexUnionGeometry getScaled(double d, mthz::Vec3 center_of_dialation=mthz::Vec3(0, 0, 0)) const;

		inline const std::vector<ConvexPrimitive>& getPolyhedra() const { return polyhedra; }
	private:
		std::vector<ConvexPrimitive> polyhedra;
	};

	//used for defining tri mesh geometry
	//note winding (counter-clockwise) is significant, determines the normal direction 
	struct TriIndices {
		uint32_t i1, i2, i3;
		Material material;
	};

	struct MeshInput {
		std::vector<TriIndices> triangle_indices;
		std::vector<mthz::Vec3> points;
	};

	MeshInput generateGridMeshInput(uint32_t grid_length, uint32_t grid_width, double grid_size, mthz::Vec3 positon=mthz::Vec3(), Material=Material::default_material());
	MeshInput generateRadialMeshInput(uint32_t n_rot_segments, uint32_t n_radial_segments, double radius_size, mthz::Vec3 positon = mthz::Vec3());
	MeshInput generateMeshInputFromMesh(const Mesh& m, mthz::Vec3 positon=mthz::Vec3(), double scaling=1.0);

	struct StaticMeshVertex {
		mthz::Vec3 p;
		uint32_t self_index;

		std::vector<mthz::Vec3> valid_normal_gauss_map;
	};

	struct StaticMeshHalfEdge {
		uint32_t p1_index, p2_index;
		int32_t twin_index; // the opposite edge of the neighboring triangle. if there is no neighbor on this edge, the value is -1;
		uint32_t next_index; // the half_edge on this triangle starting from p2_index
		uint32_t triangle_index;
		uint32_t self_index;
		mthz::Vec3 out_direction;
		uint32_t id;

		bool has_gauss_arc;
		mthz::Vec3 gauss_arc_g1;
		mthz::Vec3 gauss_arc_g2;
	};

	struct StaticMeshFace {
		mthz::Vec3 normal;
		uint32_t vertex_indices[3]; // verts
		uint32_t half_edge_indices[3]; //edges
		uint32_t self_index;

		Material material;
	};

	struct TriMeshRayQueryReturn {
		RayQueryReturn hit_info;
		uint32_t hit_triangle_inedex;
	};

	class StaticMeshGeometry {
	public:
		StaticMeshGeometry() : aabb_tree(0) {}
		StaticMeshGeometry(const StaticMeshGeometry& c);
		StaticMeshGeometry(const MeshInput& input);

		// exists for debugging only really
		StaticMeshGeometry(const std::array<StaticMeshVertex, 3>& vertices, const std::array<StaticMeshHalfEdge, 3>& half_edges);

		void recomputeFromReference(const StaticMeshGeometry& reference, const mthz::Mat3& rot, mthz::Vec3 trans, mthz::Vec3 center_of_rotation=mthz::Vec3(0, 0, 0));
		AABB genAABB() const;

		inline const std::vector<StaticMeshFace>& getTriangles() const { return triangles; }
		inline const std::vector<StaticMeshVertex>& getVertices() const { return vertices; }
		inline const AABBTree<unsigned int>& getAABBTree() const { return aabb_tree; }

		TriMeshRayQueryReturn testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) const;

		inline StaticMeshVertex get_transformed_vertex(uint32_t index, mthz::Mat3 rot, mthz::Vec3 trans, mthz::Vec3 center_of_rotation) const;
		inline StaticMeshHalfEdge get_transformed_half_edge(uint32_t index, mthz::Mat3 rot, mthz::Vec3 trans) const;
		inline StaticMeshFace get_transformed_face(uint32_t index, mthz::Mat3 rot, mthz::Vec3 trans) const;
		inline StaticMeshVertex get_vertex(uint32_t index) const { assert(index < vertices.size()); return vertices[index]; }
		inline StaticMeshHalfEdge get_half_edge(uint32_t index) const { assert(index < half_edges.size()); return half_edges[index]; }
		inline StaticMeshFace get_triangle(uint32_t index) const { assert(index < triangles.size()); return triangles[index]; }

		inline uint32_t getVertexId(uint32_t vertex_index) const { return vertex_index; }
		//inline uint32_t getEdgeId(uint32_t halfedge_index) { return hal; }
		inline uint32_t getTriangleId(uint32_t triangle_index) const { return triangle_index + vertices.size() + half_edges.size(); }

		friend class Surface;
		friend class Edge;
		friend class RigidBody;
		friend class StaticMeshHalfEdge;
		friend class StaticMeshFace;
	private:
		AABB getAABBOfTriangle(const StaticMeshFace& triangle) const;

		std::vector<StaticMeshVertex> vertices;
		std::vector<StaticMeshHalfEdge> half_edges;
		std::vector<StaticMeshFace> triangles;
		AABBTree<unsigned int> aabb_tree;
	};
}