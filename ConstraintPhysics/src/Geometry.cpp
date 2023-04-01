#include "Geometry.h"

namespace phyz {

	Geometry::Geometry(const std::initializer_list<Geometry>& in) {
		for (const Geometry& g : in) {
			polyhedra.reserve(g.polyhedra.size());
			for (const ConvexPrimitive& c : g.polyhedra) {
				polyhedra.push_back(c);
			}
		}
	}

	Geometry Geometry::box(mthz::Vec3 pos, double dx, double dy, double dz, Material material) {

		std::vector<mthz::Vec3> points(8);
		std::vector<std::vector<int>> surface_indices(6);

		points[0] = (mthz::Vec3(pos.x, pos.y, pos.z)); //0
		points[1] = (mthz::Vec3(pos.x + dx, pos.y, pos.z)); //1
		points[2] = (mthz::Vec3(pos.x + dx, pos.y + dy, pos.z)); //2
		points[3] = (mthz::Vec3(pos.x, pos.y + dy, pos.z)); //3

		points[4] = (mthz::Vec3(pos.x, pos.y, pos.z + dz)); //4
		points[5] = (mthz::Vec3(pos.x + dx, pos.y, pos.z + dz)); //5
		points[6] = (mthz::Vec3(pos.x + dx, pos.y + dy, pos.z + dz)); //6
		points[7] = (mthz::Vec3(pos.x, pos.y + dy, pos.z + dz)); //7

		surface_indices[0] = { 0, 3, 2, 1 };
		surface_indices[1] = { 0, 1, 5, 4 };
		surface_indices[2] = { 1, 2, 6, 5 };
		surface_indices[3] = { 2, 3, 7, 6 };
		surface_indices[4] = { 3, 0, 4, 7 };
		surface_indices[5] = { 4, 5, 6, 7 };

		return Geometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material));
	}

	Geometry Geometry::sphere(mthz::Vec3 center, double radius, Material material) {
		return Geometry(ConvexPrimitive((const ConvexGeometry&)Sphere(center, radius), material));
	}

	Geometry Geometry::tetra(mthz::Vec3 p1, mthz::Vec3 p2, mthz::Vec3 p3, mthz::Vec3 p4, Material material) {
		ConvexPrimitive out;

		const double rt3 = sqrt(3.0);

		std::vector<mthz::Vec3> points = { p1, p2, p3, p4 };
		std::vector<std::vector<int>> surface_indices(4);

		surface_indices[0] = { 0, 1, 2 };
		surface_indices[1] = { 1, 2, 3 };
		surface_indices[2] = { 2, 3, 0 };
		surface_indices[3] = { 3, 0, 1 };

		return Geometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material));
	}

	Geometry Geometry::triPrism(double x1, double z1, double x2, double z2, double x3, double z3, double y, double height, Material material) {
		std::vector<mthz::Vec3> points(6);
		std::vector<std::vector<int>> surface_indices(5);

		points[0] = (mthz::Vec3(x1, y, z1)); //0
		points[1] = (mthz::Vec3(x2, y, z2)); //1
		points[2] = (mthz::Vec3(x3, y, z3)); //2

		points[3] = (mthz::Vec3(x1, y + height, z1)); //3
		points[4] = (mthz::Vec3(x2, y + height, z2)); //4
		points[5] = (mthz::Vec3(x3, y + height, z3)); //5

		surface_indices[0] = { 0, 1, 2 };
		surface_indices[1] = { 0, 1, 4, 3 };
		surface_indices[2] = { 1, 2, 5, 4 };
		surface_indices[3] = { 2, 0, 3, 5 };
		surface_indices[4] = { 3, 4, 5 };

		return Geometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material));
	}

	Geometry Geometry::cylinder(mthz::Vec3 pos, double radius, double height, int detail, Material material) {
		int n_faces = 3 + detail;
		std::vector<mthz::Vec3> points(n_faces * 2);
		std::vector<std::vector<int>> surface_indices(2 + n_faces);

		for (int i = 0; i < n_faces; i++) {
			double theta = 2 * PI * i / n_faces;
			points[i] = mthz::Vec3(pos.x + radius * cos(theta), pos.y, pos.z + radius * sin(theta));
			points[i + n_faces] = mthz::Vec3(pos.x + radius * cos(theta), pos.y + height, pos.z + radius * sin(theta));
		}

		for (int i = 0; i < n_faces; i++) {
			surface_indices[0].push_back(n_faces - 1 - i);
			surface_indices[1].push_back(n_faces + i);
		}

		for (int i = 0; i < n_faces; i++) {
			int j = (i + 1 == n_faces) ? 0 : i + 1;
			surface_indices[i + 2] = { i, j, n_faces + j, n_faces + i };
		}

		return ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material);
	}

	Geometry Geometry::ring(mthz::Vec3 pos, double inner_radius, double outer_radius, double height, int detail, Material material) {
		int n_faces = 3 + detail;
		std::vector<mthz::Vec3> points(4 * n_faces);
		//std::vector<std::vector<int>> surface_indices(4 * n_faces);

		Geometry out;
		for (int i = 0; i < n_faces; i++) {
			double theta = 2 * PI * i / n_faces;
			points[i + 0 * n_faces] = mthz::Vec3(pos.x + inner_radius * cos(theta), pos.y, pos.z + inner_radius * sin(theta)); //lower inner radius
			points[i + 1 * n_faces] = mthz::Vec3(pos.x + outer_radius * cos(theta), pos.y, pos.z + outer_radius * sin(theta)); //lower outer radius
			points[i + 2 * n_faces] = mthz::Vec3(pos.x + inner_radius * cos(theta), pos.y + height, pos.z + inner_radius * sin(theta)); //upper inner radius
			points[i + 3 * n_faces] = mthz::Vec3(pos.x + outer_radius * cos(theta), pos.y + height, pos.z + outer_radius * sin(theta)); //upper outer radius
 		}

		for (int i = 0; i < n_faces; i++) {
			int j = (i + 1 == n_faces) ? 0 : i + 1;
			std::vector<mthz::Vec3> segment_points = {
				points[i], points[i + n_faces], points[i + 2 * n_faces], points[i + 3 * n_faces],
				points[j], points[j + n_faces], points[j + 2 * n_faces], points[j + 3 * n_faces]
			};

			std::vector<std::vector<int>> surface_indices = {
				{ { 0, 1, 5, 4 } }, //underside
				{ { 1, 5, 7, 3 } }, //outside
				{ { 2, 3, 7, 6 } }, //upperside
				{ { 0, 2, 6, 4 } }, //inner side
				{ { 0, 1, 3, 2 } }, //inside edge 2
				{ { 4, 5, 7, 6 } }  //inside edge 1
			};

			out = out.merge(out, Geometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(segment_points, surface_indices), material)));
		}

		return out;
	}

	static Geometry tooth(mthz::Vec3 pos, double width, double length, double height, Material material) {
		std::vector<mthz::Vec3> points(12);
		std::vector<std::vector<int>> surface_indices(8);
		double f = 0.4;
		double p = 0.3;

		points[0] = (mthz::Vec3(pos.x, pos.y, pos.z)); //0
		points[1] = (mthz::Vec3(pos.x + length * f, pos.y, pos.z)); //1
		points[2] = (mthz::Vec3(pos.x + length, pos.y, pos.z + width * p)); //2
		points[3] = (mthz::Vec3(pos.x + length, pos.y, pos.z + width * (1-p))); //3
		points[4] = (mthz::Vec3(pos.x + length * f, pos.y, pos.z + width)); //1
		points[5] = (mthz::Vec3(pos.x, pos.y, pos.z + width)); //1

		points[6] = (mthz::Vec3(pos.x, pos.y + height, pos.z)); //0
		points[7] = (mthz::Vec3(pos.x + length * f, pos.y + height, pos.z)); //1
		points[8] = (mthz::Vec3(pos.x + length, pos.y + height, pos.z + width * p)); //2
		points[9] = (mthz::Vec3(pos.x + length, pos.y + height, pos.z + width * (1 - p))); //3
		points[10] = (mthz::Vec3(pos.x + length * f, pos.y + height, pos.z + width)); //1
		points[11] = (mthz::Vec3(pos.x, pos.y + height, pos.z + width)); //1

		surface_indices[0] = { 0, 1, 2, 3, 4, 5 };
		surface_indices[1] = { 6, 7, 8, 9, 10, 11 };
		surface_indices[2] = { 0, 1, 7, 6 };
		surface_indices[3] = { 1, 2, 8, 7 };
		surface_indices[4] = { 2, 3, 9, 8 };
		surface_indices[5] = { 3, 4, 10, 9 };
		surface_indices[6] = { 4, 5, 11, 10 };
		surface_indices[7] = { 5, 0, 6, 11 };
	
		return Geometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material));
	}

	Geometry Geometry::gear(mthz::Vec3 pos, double radius, double tooth_length, double height, int n_teeth, bool parity, Material material, double tooth_width) {

		Geometry wheel = cylinder(pos, radius, height, 2 * n_teeth - 3, material);
		double default_tooth_width = radius * PI / n_teeth;
		tooth_width = (tooth_width == -1)?  default_tooth_width : tooth_width;
		Geometry toothG= tooth(pos + mthz::Vec3(radius, 0, -0.5 * (tooth_width - default_tooth_width)), tooth_width, tooth_length, height, material);

		double d_theta = 2 * PI / (n_teeth);
		toothG = toothG.getRotated(mthz::Quaternion(-0.25 * d_theta, mthz::Vec3(0, 1, 0)), pos + mthz::Vec3(radius, 0, 0));
		for (int i = 0; i < n_teeth; i++) {
			toothG= toothG.getRotated(mthz::Quaternion(d_theta, mthz::Vec3(0, 1, 0)), pos);
			wheel = merge(wheel, toothG);
		}

		return (parity)? wheel.getRotated(mthz::Quaternion(d_theta/2.0, mthz::Vec3(0, 1, 0)), pos) :  wheel;
	}

	Geometry Geometry::bevelGear(mthz::Vec3 pos, double radius, double tooth_radius, double tooth_width, double tooth_height, double height, int n_teeth, bool parity, Material material, double hole_radius, int circle_detail) {
		Geometry wheel = (hole_radius == 0) ? cylinder(pos, radius, height, circle_detail, material) : ring(pos, hole_radius, radius, height, circle_detail, material);
		mthz::Vec3 tooth_pos = pos + mthz::Vec3(radius - tooth_radius, height, 0);
		Geometry toothG = tooth(tooth_pos, tooth_width, tooth_height, -tooth_radius, material).getRotated(mthz::Quaternion(PI/2.0, mthz::Vec3(0, 0, 1)), tooth_pos);

		double d_theta = 2 * PI / (n_teeth);
		toothG = toothG.getRotated(mthz::Quaternion(-0.25 * d_theta, mthz::Vec3(0, 1, 0)), pos + mthz::Vec3(radius, 0, 0));
		for (int i = 0; i < n_teeth; i++) {
			toothG = toothG.getRotated(mthz::Quaternion(d_theta, mthz::Vec3(0, 1, 0)), pos);
			wheel = merge(wheel, toothG);
		}

		return (parity) ? wheel.getRotated(mthz::Quaternion(d_theta / 2.0, mthz::Vec3(0, 1, 0)), pos) : wheel;
	}

	Geometry Geometry::pinion(mthz::Vec3 pos, double height, double width, double tooth_height, double tooth_width, double gap_width, int n_teeth, Material material) {
		double length = (1 + n_teeth) * gap_width + n_teeth * tooth_width;
		Geometry pinion = box(pos, width, height, length, material);
		Geometry toothG = tooth(mthz::Vec3(0, 0, 0), tooth_width, tooth_height, -width, material).getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)));
		for (int i = 0; i < n_teeth; i++) {
			mthz::Vec3 tooth_pos = pos + mthz::Vec3(0, height, (1 + i) * gap_width + i * tooth_width);
			pinion = pinion.merge(pinion, toothG.getTranslated(tooth_pos));
		}
		return pinion;
	}

	Geometry Geometry::merge(const Geometry& g1, const Geometry& g2) {
		Geometry out;
		out.polyhedra.reserve(g1.polyhedra.size() + g2.polyhedra.size());
		for (const ConvexPrimitive& c : g1.polyhedra) {
			out.polyhedra.push_back(c);
		}
		for (const ConvexPrimitive& c : g2.polyhedra) {
			out.polyhedra.push_back(c);
		}
		return out;
	}

	Geometry Geometry::getTranslated(const mthz::Vec3 v) const {
		Geometry out;
		out.polyhedra.reserve(polyhedra.size());
		for (const ConvexPrimitive& c : polyhedra) {
			out.polyhedra.push_back(c.getTranslated(v));
		}
		return out;
	}

	Geometry Geometry::getRotated(const mthz::Quaternion q, const mthz::Vec3& rot_point) const {
		Geometry out;
		out.polyhedra.reserve(polyhedra.size());
		for (const ConvexPrimitive& c : polyhedra) {
			out.polyhedra.push_back(c.getRotated(q, rot_point));
		}
		return out;
	}
}