#pragma once
#include "ConvexPrimitive.h"

namespace phyz {
	class RigidBody;

	class Geometry {
	public:
		Geometry() {}
		Geometry(const ConvexPrimitive& c) : polyhedra({ c }) {}
		Geometry(const std::initializer_list<Geometry>& g);

		static Geometry box(mthz::Vec3 pos, double dx, double dy, double dz, Material material = Material::default_material());
		static Geometry sphere(mthz::Vec3 center, double radius, Material material = Material::default_material());
		static Geometry tetra(mthz::Vec3 p1, mthz::Vec3 p2, mthz::Vec3 p3, mthz::Vec3 p4, Material material = Material::default_material());
		static Geometry regDodecahedron(mthz::Vec3 pos, double size, Material material = Material::default_material());
		static Geometry stellatedDodecahedron(mthz::Vec3 pos, double size, double spike_length_ratio, Material = Material::default_material());
		static Geometry triPrism(double x1, double z1, double x2, double z2, double x3, double z3, double y, double height, Material material = Material::default_material());
		static Geometry cylinder(mthz::Vec3 pos, double radius, double height, int detail = 10, Material material = Material::default_material());
		static Geometry ring(mthz::Vec3 pos, double inner_radius, double outter_radius, double height, int detail = 4, Material material = Material::default_material());
		static Geometry gear(mthz::Vec3 pos, double radius, double tooth_length, double height, int n_teeth, bool parity = false, Material material = Material::default_material(), double tooth_width = -1);
		static Geometry bevelGear(mthz::Vec3 pos, double radius, double tooth_radius, double tooth_width, double tooth_height, double height, int n_teeth, bool parity = false, Material material = Material::default_material(), double hole_radius = 0.0, int circle_detail = 7);
		static Geometry pinion(mthz::Vec3 pos, double height, double width, double tooth_height, double tooth_width, double gap_width, int n_teeth, Material material = Material::default_material());

		static Geometry merge(const Geometry& g1, const Geometry& g2);
		Geometry getTranslated(const mthz::Vec3 v) const;
		Geometry getRotated(const mthz::Quaternion q, const mthz::Vec3& rot_point=mthz::Vec3(0, 0, 0)) const;
		Geometry getScaled(double d, mthz::Vec3 center_of_dialation=mthz::Vec3(0, 0, 0)) const;

		inline const std::vector<ConvexPrimitive>& getPolyhedra() const { return polyhedra; }
	private:
		std::vector<ConvexPrimitive> polyhedra;
	};
}