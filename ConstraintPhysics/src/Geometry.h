#pragma once
#include "ConvexPoly.h"

namespace phyz {
	class RigidBody;

	class Geometry {
	public:
		Geometry() {}
		Geometry(const ConvexPoly& c) : polyhedra({ c }) {}
		Geometry(const std::initializer_list<Geometry>& g);

		static Geometry box(mthz::Vec3 pos, double dx, double dy, double dz, double density = 1.0);
		static Geometry tetra(mthz::Vec3 p1, mthz::Vec3 p2, mthz::Vec3 p3, mthz::Vec3 p4, double density = 1.0);
		static Geometry triPrism(double x1, double z1, double x2, double z2, double x3, double z3, double y, double height, double density = 1.0);
		static Geometry cylinder(mthz::Vec3 pos, double radius, double height, int detail = 10, double density = 1.0);
		static Geometry ring(mthz::Vec3 pos, double inner_radius, double outter_radius, double height, int detail = 4, double density = 1.0);
		static Geometry gear(mthz::Vec3 pos, double radius, double tooth_length, double height, int n_teeth, bool parity = false, double density = 1.0);
		static Geometry bevelGear(mthz::Vec3 pos, double radius, double tooth_radius, double tooth_width, double tooth_height, double height, int n_teeth, double density = 1.0);

		static Geometry merge(const Geometry& g1, const Geometry& g2);
		Geometry getTranslated(const mthz::Vec3 v) const;
		Geometry getRotated(const mthz::Quaternion q, const mthz::Vec3& rot_point=mthz::Vec3(0, 0, 0)) const;

		inline const std::vector<ConvexPoly>& getPolyhedra() const { return polyhedra; }
	private:
		std::vector<ConvexPoly> polyhedra;
	};
}