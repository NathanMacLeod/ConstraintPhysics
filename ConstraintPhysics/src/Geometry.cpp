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

	Geometry Geometry::psuedoSphere(mthz::Vec3 center, double radius, int n_rows, int n_cols, Material material) {
		std::vector<mthz::Vec3> points;
		std::vector<std::vector<int>> surface_indices;

		mthz::Vec3 bottom_pole = center - mthz::Vec3(0, radius, 0);
		mthz::Vec3 top_pole = center + mthz::Vec3(0, radius, 0);
		points.push_back(bottom_pole);
		points.push_back(top_pole);

		//create other vertices
		for (int row = 1; row < n_rows; row++) {
			for (int col = 0; col < n_cols; col++) {
				//polar coordinates
				double theta = -2 * PI * col / n_cols;
				double phi = PI - PI * row / n_rows;

				points.push_back(center + radius * mthz::Vec3(cos(theta) * sin(phi), cos(phi), sin(theta) * sin(phi)));
			}
		}

		int bottom_pole_index = 0;
		int top_pole_index = 1;
		int nonpole_offset = 2;
		//create surfaces
		for (int col = 0; col < n_cols; col++) {
			int i1 = col;
			int i2 = (col + 1) % n_cols;

			surface_indices.push_back({ bottom_pole_index, i2 + nonpole_offset, i1 + nonpole_offset });
		}
		for (int row = 1; row < n_rows - 1; row++) {
			for (int col = 0; col < n_cols; col++) {
				int i1 = col;
				int i2 = (col + 1) % n_cols;
				int row_offset = (row - 1) * n_cols;

				surface_indices.push_back({ i1 + row_offset + nonpole_offset, i2 + row_offset + nonpole_offset, i2 + n_cols + row_offset + nonpole_offset, i1 + n_cols + row_offset + nonpole_offset });
			}
		}
		for (int col = 0; col < n_cols; col++) {
			int i1 = col;
			int i2 = (col + 1) % n_cols;
			int row_offset = (n_rows - 2) * n_cols;

			surface_indices.push_back({ i1 + row_offset + nonpole_offset, i2 + row_offset + nonpole_offset, top_pole_index });
		}

		return Geometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material));
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

	Geometry Geometry::octahedron(mthz::Vec3 pos, double radius, Material material) {
		std::vector<mthz::Vec3> points(6);
		std::vector<std::vector<int>> surface_indices(8);

		points[0] = mthz::Vec3(0, radius, 0);
		points[1] = mthz::Vec3(-radius, 0, 0);
		points[2] = mthz::Vec3(0, 0, -radius);
		points[3] = mthz::Vec3(radius, 0, 0);
		points[4] = mthz::Vec3(0, 0, radius);
		points[5] = mthz::Vec3(0, -radius, 0);

		surface_indices[0] = { 1, 2, 0 };
		surface_indices[1] = { 2, 3, 0 };
		surface_indices[2] = { 3, 4, 0 };
		surface_indices[3] = { 4, 1, 0 };

		surface_indices[4] = { 1, 2, 5 };
		surface_indices[5] = { 2, 3, 5 };
		surface_indices[6] = { 3, 4, 5 };
		surface_indices[7] = { 4, 1, 5 };

		return Geometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material));
	}

	//https://en.wikipedia.org/wiki/Regular_dodecahedron#Cartesian_coordinates
	Geometry Geometry::regDodecahedron(mthz::Vec3 pos, double size, Material material) {
		std::vector<mthz::Vec3> points(20);
		std::vector<std::vector<int>> surface_indices(12);

		double r = size / 2.0;
		double phi = (1 + sqrt(5)) / 2.0;
		double s = r / phi; // r / golden-ratio
		double q = r / (phi * phi);

		points[0] = mthz::Vec3(-s, -s, -s);
		points[1] = mthz::Vec3(s, -s, -s);
		points[2] = mthz::Vec3(s, s, -s);
		points[3] = mthz::Vec3(-s, s, -s);
		points[4] = mthz::Vec3(-s, -s, s);
		points[5] = mthz::Vec3(s, -s, s);
		points[6] = mthz::Vec3(s, s, s);
		points[7] = mthz::Vec3(-s, s, s);

		points[8] = mthz::Vec3(0, -r, -q);
		points[9] = mthz::Vec3(0, r, -q);
		points[10] = mthz::Vec3(0, r, q);
		points[11] = mthz::Vec3(0, -r, q);

		points[12] = mthz::Vec3(-r, -q, 0);
		points[13] = mthz::Vec3(r, -q, 0);
		points[14] = mthz::Vec3(r, q, 0);
		points[15] = mthz::Vec3(-r, q, 0);

		points[16] = mthz::Vec3(-q, 0, -r);
		points[17] = mthz::Vec3(q, 0, -r);
		points[18] = mthz::Vec3(q, 0, r);
		points[19] = mthz::Vec3(-q, 0, r);

		surface_indices[0] = { 0, 8, 1, 17, 16 };
		surface_indices[1] = { 1, 13, 14, 2, 17 };
		surface_indices[2] = { 8, 1, 13, 5, 11 };
		surface_indices[3] = { 8, 11, 4, 12, 0 };
		surface_indices[4] = { 11, 5, 18, 19, 4 };
		surface_indices[5] = { 5, 13, 14, 6, 18 };
		surface_indices[6] = { 3, 16, 17, 2, 9 };
		surface_indices[7] = { 12, 0, 16, 3, 15 };
		surface_indices[8] = { 4, 12, 15, 7, 19 };
		surface_indices[9] = { 7, 15, 3, 9, 10 };
		surface_indices[10] = { 7, 10, 6, 18, 19 };
		surface_indices[11] = { 10, 9, 2, 14, 6 };

		return Geometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material)).getTranslated(pos);
	}

	Geometry Geometry::stellatedDodecahedron(mthz::Vec3 pos, double size, double spike_length_ratio, Material material) {
		Geometry dodecahedron = regDodecahedron(pos, size, material);

		std::vector<Geometry> spikes;
		for (const Surface& s : ((Polyhedron*)dodecahedron.getPolyhedra()[0].getGeometry())->getSurfaces()) {
			mthz::Vec3 center = (s.getPointI(0) + s.getPointI(1) + s.getPointI(2) + s.getPointI(3) + s.getPointI(4)) / 5;
			mthz::Vec3 spike_tip = center + s.normal() * spike_length_ratio * size;

			std::vector<mthz::Vec3> points(6);
			std::vector<std::vector<int>> surface_indices(6);

			for (int i = 0; i < 5; i++) points[i] = s.getPointI(i);
			points[5] = spike_tip;

			surface_indices[0] = { 0, 1, 2, 3, 4 };
			surface_indices[1] = { 0, 1, 5 };
			surface_indices[2] = { 1, 2, 5 };
			surface_indices[3] = { 2, 3, 5 };
			surface_indices[4] = { 3, 4, 5 };
			surface_indices[5] = { 4, 0, 5 };

			spikes.push_back(Geometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material)));
		}

		for (const Geometry& g : spikes) dodecahedron = Geometry::merge(dodecahedron, g);

		return dodecahedron;
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

	Geometry Geometry::getScaled(double d, mthz::Vec3 center_of_dialation) const {
		Geometry out;
		out.polyhedra.reserve(polyhedra.size());
		for (const ConvexPrimitive& c : polyhedra) {
			out.polyhedra.push_back(c.getScaled(d, center_of_dialation));
		}
		return out;
	}
}