#pragma once
#include "ConvexPrimitive.h"
#include "AABB_Tree.h"
#include "HACD.h"

namespace phyz {
	class RigidBody;

	class ConvexUnionGeometry {
	public:
		ConvexUnionGeometry() {}
		ConvexUnionGeometry(const ConvexPrimitive& c) : polyhedra({ c }) {}
		ConvexUnionGeometry(const std::initializer_list<ConvexUnionGeometry>& g);

		static ConvexUnionGeometry box(mthz::Vec3 pos, double dx, double dy, double dz, Material material = Material::default_material());
		static ConvexUnionGeometry sphere(mthz::Vec3 center, double radius, Material material = Material::default_material());
		static ConvexUnionGeometry cylinder(mthz::Vec3 pos, double radius, double height, Material material = Material::default_material());
		static ConvexUnionGeometry psuedoSphere(mthz::Vec3 center, double radius, int n_rows = 15, int n_cols = 20, Material material = Material::default_material());
		static ConvexUnionGeometry tetra(mthz::Vec3 p1, mthz::Vec3 p2, mthz::Vec3 p3, mthz::Vec3 p4, Material material = Material::default_material());
		static ConvexUnionGeometry octahedron(mthz::Vec3 pos, double radius, Material material = Material::default_material());
		static ConvexUnionGeometry regDodecahedron(mthz::Vec3 pos, double size, Material material = Material::default_material());
		static ConvexUnionGeometry stellatedDodecahedron(mthz::Vec3 pos, double size, double spike_length_ratio, Material = Material::default_material());
		static ConvexUnionGeometry triPrism(double x1, double z1, double x2, double z2, double x3, double z3, double y, double height, Material material = Material::default_material());
		static ConvexUnionGeometry polyCylinder(mthz::Vec3 pos, double radius, double height, int detail = 10, Material material = Material::default_material());
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
		unsigned int i1, i2, i3;
		Material material;
	};

	struct MeshInput {
		std::vector<TriIndices> triangle_indices;
		std::vector<mthz::Vec3> points;
	};

	MeshInput generateGridMeshInput(int grid_length, int grid_width, double grid_size, mthz::Vec3 positon=mthz::Vec3(), Material=Material::default_material());
	MeshInput generateRadialMeshInput(int n_rot_segments, int n_radial_segments, double radius_size, mthz::Vec3 positon = mthz::Vec3());
	MeshInput generateMeshInputFromMesh(const Mesh& m, mthz::Vec3 positon=mthz::Vec3(), double scaling=1.0);

	struct StaticMeshVertex {
		mthz::Vec3 p;
		//Material material;
		int id;
	};

	struct StaticMeshEdge {
		mthz::Vec3 p1, p2;
		mthz::Vec3 out_direction;
		//Material material;
		int id;
	};

	struct StaticMeshFace {
		mthz::Vec3 normal;
		StaticMeshVertex vertices[3];
		StaticMeshEdge edges[3];
		std::vector<mthz::Vec3> gauss_region;
		int concave_neighbor_count;
		Material material;
		AABB aabb;
		int id;

		inline AABB computeAABB() const { return AABB::encapsulatePointCloud({ vertices[0].p, vertices[1].p, vertices[2].p }); }
		StaticMeshFace getTransformed(const mthz::Mat3& rot, mthz::Vec3 translation, mthz::Vec3 center_of_rotation) const;
	};

	class StaticMeshGeometry {
	public:
		StaticMeshGeometry() : aabb_tree(0) {}
		StaticMeshGeometry(const StaticMeshGeometry& c);
		StaticMeshGeometry(const MeshInput& input);

		void recomputeFromReference(const StaticMeshGeometry& reference, const mthz::Mat3& rot, mthz::Vec3 trans, mthz::Vec3 center_of_rotation=mthz::Vec3(0, 0, 0));
		AABB genAABB() const;

		inline const std::vector<StaticMeshFace>& getTriangles() const { return triangles; }
		inline const AABBTree<unsigned int>& getAABBTree() const { return aabb_tree; }

		RayQueryReturn testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir);

		friend class Surface;
		friend class Edge;
		friend class RigidBody;
	private:
		std::vector<StaticMeshFace> triangles;
		AABBTree<unsigned int> aabb_tree;
	};
}