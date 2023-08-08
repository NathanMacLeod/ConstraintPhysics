#include "Geometry.h"

namespace phyz {

	ConvexUnionGeometry::ConvexUnionGeometry(const std::initializer_list<ConvexUnionGeometry>& in) {
		for (const ConvexUnionGeometry& g : in) {
			polyhedra.reserve(g.polyhedra.size());
			for (const ConvexPrimitive& c : g.polyhedra) {
				polyhedra.push_back(c);
			}
		}
	}

	ConvexUnionGeometry ConvexUnionGeometry::box(mthz::Vec3 pos, double dx, double dy, double dz, Material material) {

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

		return ConvexUnionGeometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material));
	}

	ConvexUnionGeometry ConvexUnionGeometry::sphere(mthz::Vec3 center, double radius, Material material) {
		return ConvexUnionGeometry(ConvexPrimitive((const ConvexGeometry&)Sphere(center, radius), material));
	}

	ConvexUnionGeometry ConvexUnionGeometry::psuedoSphere(mthz::Vec3 center, double radius, int n_rows, int n_cols, Material material) {
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

		return ConvexUnionGeometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material));
	}

	ConvexUnionGeometry ConvexUnionGeometry::tetra(mthz::Vec3 p1, mthz::Vec3 p2, mthz::Vec3 p3, mthz::Vec3 p4, Material material) {
		ConvexPrimitive out;

		const double rt3 = sqrt(3.0);

		std::vector<mthz::Vec3> points = { p1, p2, p3, p4 };
		std::vector<std::vector<int>> surface_indices(4);

		surface_indices[0] = { 0, 1, 2 };
		surface_indices[1] = { 1, 2, 3 };
		surface_indices[2] = { 2, 3, 0 };
		surface_indices[3] = { 3, 0, 1 };

		return ConvexUnionGeometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material));
	}

	ConvexUnionGeometry ConvexUnionGeometry::triPrism(double x1, double z1, double x2, double z2, double x3, double z3, double y, double height, Material material) {
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

		return ConvexUnionGeometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material));
	}

	ConvexUnionGeometry ConvexUnionGeometry::octahedron(mthz::Vec3 pos, double radius, Material material) {
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

		return ConvexUnionGeometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material));
	}

	//https://en.wikipedia.org/wiki/Regular_dodecahedron#Cartesian_coordinates
	ConvexUnionGeometry ConvexUnionGeometry::regDodecahedron(mthz::Vec3 pos, double size, Material material) {
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

		return ConvexUnionGeometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material)).getTranslated(pos);
	}

	ConvexUnionGeometry ConvexUnionGeometry::stellatedDodecahedron(mthz::Vec3 pos, double size, double spike_length_ratio, Material material) {
		ConvexUnionGeometry dodecahedron = regDodecahedron(pos, size, material);

		std::vector<ConvexUnionGeometry> spikes;
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

			spikes.push_back(ConvexUnionGeometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material)));
		}

		for (const ConvexUnionGeometry& g : spikes) dodecahedron = ConvexUnionGeometry::merge(dodecahedron, g);

		return dodecahedron;
	}

	ConvexUnionGeometry ConvexUnionGeometry::cylinder(mthz::Vec3 pos, double radius, double height, int detail, Material material) {
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

	ConvexUnionGeometry ConvexUnionGeometry::ring(mthz::Vec3 pos, double inner_radius, double outer_radius, double height, int detail, Material material) {
		int n_faces = 3 + detail;
		std::vector<mthz::Vec3> points(4 * n_faces);
		//std::vector<std::vector<int>> surface_indices(4 * n_faces);

		ConvexUnionGeometry out;
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

			out = out.merge(out, ConvexUnionGeometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(segment_points, surface_indices), material)));
		}

		return out;
	}

	static ConvexUnionGeometry tooth(mthz::Vec3 pos, double width, double length, double height, Material material) {
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
	
		return ConvexUnionGeometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material));
	}

	ConvexUnionGeometry ConvexUnionGeometry::gear(mthz::Vec3 pos, double radius, double tooth_length, double height, int n_teeth, bool parity, Material material, double tooth_width) {

		ConvexUnionGeometry wheel = cylinder(pos, radius, height, 2 * n_teeth - 3, material);
		double default_tooth_width = radius * PI / n_teeth;
		tooth_width = (tooth_width == -1)?  default_tooth_width : tooth_width;
		ConvexUnionGeometry toothG= tooth(pos + mthz::Vec3(radius, 0, -0.5 * (tooth_width - default_tooth_width)), tooth_width, tooth_length, height, material);

		double d_theta = 2 * PI / (n_teeth);
		toothG = toothG.getRotated(mthz::Quaternion(-0.25 * d_theta, mthz::Vec3(0, 1, 0)), pos + mthz::Vec3(radius, 0, 0));
		for (int i = 0; i < n_teeth; i++) {
			toothG= toothG.getRotated(mthz::Quaternion(d_theta, mthz::Vec3(0, 1, 0)), pos);
			wheel = merge(wheel, toothG);
		}

		return (parity)? wheel.getRotated(mthz::Quaternion(d_theta/2.0, mthz::Vec3(0, 1, 0)), pos) :  wheel;
	}

	ConvexUnionGeometry ConvexUnionGeometry::bevelGear(mthz::Vec3 pos, double radius, double tooth_radius, double tooth_width, double tooth_height, double height, int n_teeth, bool parity, Material material, double hole_radius, int circle_detail) {
		ConvexUnionGeometry wheel = (hole_radius == 0) ? cylinder(pos, radius, height, circle_detail, material) : ring(pos, hole_radius, radius, height, circle_detail, material);
		mthz::Vec3 tooth_pos = pos + mthz::Vec3(radius - tooth_radius, height, 0);
		ConvexUnionGeometry toothG = tooth(tooth_pos, tooth_width, tooth_height, -tooth_radius, material).getRotated(mthz::Quaternion(PI/2.0, mthz::Vec3(0, 0, 1)), tooth_pos);

		double d_theta = 2 * PI / (n_teeth);
		toothG = toothG.getRotated(mthz::Quaternion(-0.25 * d_theta, mthz::Vec3(0, 1, 0)), pos + mthz::Vec3(radius, 0, 0));
		for (int i = 0; i < n_teeth; i++) {
			toothG = toothG.getRotated(mthz::Quaternion(d_theta, mthz::Vec3(0, 1, 0)), pos);
			wheel = merge(wheel, toothG);
		}

		return (parity) ? wheel.getRotated(mthz::Quaternion(d_theta / 2.0, mthz::Vec3(0, 1, 0)), pos) : wheel;
	}

	ConvexUnionGeometry ConvexUnionGeometry::pinion(mthz::Vec3 pos, double height, double width, double tooth_height, double tooth_width, double gap_width, int n_teeth, Material material) {
		double length = (1 + n_teeth) * gap_width + n_teeth * tooth_width;
		ConvexUnionGeometry pinion = box(pos, width, height, length, material);
		ConvexUnionGeometry toothG = tooth(mthz::Vec3(0, 0, 0), tooth_width, tooth_height, -width, material).getRotated(mthz::Quaternion(PI / 2.0, mthz::Vec3(0, 0, 1)));
		for (int i = 0; i < n_teeth; i++) {
			mthz::Vec3 tooth_pos = pos + mthz::Vec3(0, height, (1 + i) * gap_width + i * tooth_width);
			pinion = ConvexUnionGeometry::merge(pinion, toothG.getTranslated(tooth_pos));
		}
		return pinion;
	}

	ConvexUnionGeometry ConvexUnionGeometry::uShape(mthz::Vec3 pos, double inner_radius, double outer_radius, double height, int n_segments, Material material) {
		assert(n_segments >= 1);

		ConvexUnionGeometry out;
		double dTheta = PI / n_segments;
		for (int i = 0; i < n_segments; i++) {
			std::vector<mthz::Vec3> points(8);
			std::vector<std::vector<int>> surface_indices(6);


			double t1 = i * dTheta;
			double t2 = (i + 1) * dTheta;

			points[0] = pos + inner_radius * mthz::Vec3(cos(t1), 0, sin(t1));
			points[1] = pos + outer_radius * mthz::Vec3(cos(t1), 0, sin(t1));
			points[2] = pos + outer_radius * mthz::Vec3(cos(t2), 0, sin(t2));
			points[3] = pos + inner_radius * mthz::Vec3(cos(t2), 0, sin(t2));

			points[4] = points[0] + mthz::Vec3(0, height, 0);
			points[5] = points[1] + mthz::Vec3(0, height, 0);
			points[6] = points[2] + mthz::Vec3(0, height, 0);
			points[7] = points[3] + mthz::Vec3(0, height, 0);

			surface_indices[0] = { 0, 1, 2, 3 };
			surface_indices[1] = { 4, 5, 6, 7 };
			surface_indices[2] = { 0, 1, 5, 4 };
			surface_indices[3] = { 0, 4, 7, 3 };
			surface_indices[4] = { 1, 2, 6, 5 };
			surface_indices[5] = { 3, 2, 6, 7 };

			ConvexUnionGeometry segment = ConvexUnionGeometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(points, surface_indices), material));
			out = ConvexUnionGeometry::merge(out, segment);
		}

		return out;
	}

	ConvexUnionGeometry ConvexUnionGeometry::funnel(mthz::Vec3 pos, double tube_radius, double tube_height, double bowl_radius, double bowl_angle, double thickness, int n_segments, Material material) {
		ConvexUnionGeometry out;

		double dTheta = 2 * PI / n_segments;
		for (int i = 0; i < n_segments; i++) {
			std::vector<mthz::Vec3> tube_points(8);
			std::vector<std::vector<int>> tube_surface_indices(6);

			double t1 = i * dTheta;
			double t2 = (i + 1) * dTheta;

			double bevel_length_diff = thickness * tan(bowl_angle / 2.0);

			tube_points[0] = pos + tube_radius * mthz::Vec3(cos(t1), 0, sin(t1));
			tube_points[1] = pos + (tube_radius + thickness) * mthz::Vec3(cos(t1), 0, sin(t1));
			tube_points[2] = pos + (tube_radius + thickness) * mthz::Vec3(cos(t2), 0, sin(t2));
			tube_points[3] = pos + tube_radius * mthz::Vec3(cos(t2), 0, sin(t2));

			tube_points[4] = tube_points[0] + mthz::Vec3(0, tube_height + bevel_length_diff, 0);
			tube_points[5] = tube_points[1] + mthz::Vec3(0, tube_height, 0);
			tube_points[6] = tube_points[2] + mthz::Vec3(0, tube_height, 0);
			tube_points[7] = tube_points[3] + mthz::Vec3(0, tube_height + bevel_length_diff, 0);

			tube_surface_indices[0] = { 0, 1, 2, 3 };
			tube_surface_indices[1] = { 4, 5, 6, 7 };
			tube_surface_indices[2] = { 0, 1, 5, 4 };
			tube_surface_indices[3] = { 0, 4, 7, 3 };
			tube_surface_indices[4] = { 1, 2, 6, 5 };
			tube_surface_indices[5] = { 3, 2, 6, 7 };

			ConvexUnionGeometry tube_segment = ConvexUnionGeometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(tube_points, tube_surface_indices), material));
			out = ConvexUnionGeometry::merge(out, tube_segment);

			std::vector<mthz::Vec3> bowl_points(8);
			std::vector<std::vector<int>> bowl_surface_indices(6);

			bowl_points[0] = tube_points[4];
			bowl_points[1] = tube_points[5];
			bowl_points[2] = tube_points[6];
			bowl_points[3] = tube_points[7];

			bowl_points[4] = bowl_points[0] + (bowl_radius + bevel_length_diff) * mthz::Vec3(sin(bowl_angle) * cos(t1), cos(bowl_angle), sin(bowl_angle) * sin(t1));
			bowl_points[5] = bowl_points[1] + bowl_radius * mthz::Vec3(sin(bowl_angle) * cos(t1), cos(bowl_angle), sin(bowl_angle) * sin(t1));
			bowl_points[6] = bowl_points[2] + bowl_radius * mthz::Vec3(sin(bowl_angle) * cos(t2), cos(bowl_angle), sin(bowl_angle) * sin(t2));
			bowl_points[7] = bowl_points[3] + (bowl_radius + bevel_length_diff) * mthz::Vec3(sin(bowl_angle) * cos(t2), cos(bowl_angle), sin(bowl_angle) * sin(t2));

			bowl_surface_indices[0] = { 0, 1, 2, 3 };
			bowl_surface_indices[1] = { 4, 5, 6, 7 };
			bowl_surface_indices[2] = { 0, 1, 5, 4 };
			bowl_surface_indices[3] = { 0, 4, 7, 3 };
			bowl_surface_indices[4] = { 1, 2, 6, 5 };
			bowl_surface_indices[5] = { 3, 2, 6, 7 };

			ConvexUnionGeometry bowl_segment = ConvexUnionGeometry(ConvexPrimitive((const ConvexGeometry&)Polyhedron(bowl_points, bowl_surface_indices), material));
			out = ConvexUnionGeometry::merge(out, bowl_segment);
		}

		return out;
	}

	ConvexUnionGeometry ConvexUnionGeometry::merge(const ConvexUnionGeometry& g1, const ConvexUnionGeometry& g2) {
		ConvexUnionGeometry out;
		out.polyhedra.reserve(g1.polyhedra.size() + g2.polyhedra.size());
		for (const ConvexPrimitive& c : g1.polyhedra) {
			out.polyhedra.push_back(c);
		}
		for (const ConvexPrimitive& c : g2.polyhedra) {
			out.polyhedra.push_back(c);
		}
		return out;
	}

	ConvexUnionGeometry ConvexUnionGeometry::getTranslated(const mthz::Vec3 v) const {
		ConvexUnionGeometry out;
		out.polyhedra.reserve(polyhedra.size());
		for (const ConvexPrimitive& c : polyhedra) {
			out.polyhedra.push_back(c.getTranslated(v));
		}
		return out;
	}

	ConvexUnionGeometry ConvexUnionGeometry::getRotated(const mthz::Quaternion q, const mthz::Vec3& rot_point) const {
		ConvexUnionGeometry out;
		out.polyhedra.reserve(polyhedra.size());
		for (const ConvexPrimitive& c : polyhedra) {
			out.polyhedra.push_back(c.getRotated(q, rot_point));
		}
		return out;
	}

	ConvexUnionGeometry ConvexUnionGeometry::getScaled(double d, mthz::Vec3 center_of_dialation) const {
		ConvexUnionGeometry out;
		out.polyhedra.reserve(polyhedra.size());
		for (const ConvexPrimitive& c : polyhedra) {
			out.polyhedra.push_back(c.getScaled(d, center_of_dialation));
		}
		return out;
	}

	MeshInput generateGridMeshInput(int grid_length, int grid_width, double grid_size, mthz::Vec3 position) {
		std::vector<mthz::Vec3> points((grid_length + 1) * (grid_width + 1));
		std::vector<TriIndices> triangle_indices(grid_length * grid_width * 2);

		for (int j = 0; j < grid_width + 1; j++) {
			for (int i = 0; i < grid_length + 1; i++) {
				points[i + (grid_width + 1) * j] = position + mthz::Vec3(grid_size * i, 0, grid_size * j);
				if (i > 0 && j > 0) {
					int tile_indx = i - 1 + grid_width * (j - 1);

					unsigned int indx1 = i - 1 + (grid_width + 1) * (j - 1);
					unsigned int indx2 = i + (grid_width + 1) * (j - 1);
					unsigned int indx3 = i + (grid_width + 1) * j;
					unsigned int indx4 = i - 1 + (grid_width + 1) * j;

					triangle_indices[2 * tile_indx] = TriIndices{ indx3, indx2, indx1, Material::default_material() };
					triangle_indices[2 * tile_indx + 1] = TriIndices{ indx4, indx3, indx1, Material::default_material() };
				}
			}
		}

		return MeshInput{ triangle_indices, points };
	}

	MeshInput generateRadialMeshInput(int n_rot_segments, int n_radial_segments, double radius_size, mthz::Vec3 position) {
		assert(n_radial_segments >= 1);

		std::vector<mthz::Vec3> points(1 + n_rot_segments * n_radial_segments);
		std::vector<TriIndices> triangle_indices(n_rot_segments + 2 * n_rot_segments * (n_radial_segments - 1));

		points[0] = position; //center;
		int offset = 1;

		double dTheta = 2 * PI / n_rot_segments;
		for (int r = 1; r < n_radial_segments+1; r++) {
			for (int t = 0; t < n_rot_segments; t++) {
				double theta = dTheta * t;
				points[offset + t + (r-1) * n_rot_segments] = position + r * radius_size * mthz::Vec3(mthz::Vec3(cos(theta), 0, -sin(theta)));
			}
		}

		
		for (unsigned int t = 0; t < n_rot_segments; t++) {
			triangle_indices[t] = TriIndices{ 0, t + offset, ((t + 1) % n_rot_segments) + offset, Material::default_material() };
		}

		
		for (int r = 0; r < n_radial_segments - 1; r++) {
			for (unsigned int t = 0; t < n_rot_segments; t++) {

				unsigned int indx1 = (r+1) * n_rot_segments + t + offset;
				unsigned int indx2 = (r+1) * n_rot_segments + ((t + 1) % n_rot_segments) + offset;
				unsigned int indx3 = r * n_rot_segments + ((t + 1) % n_rot_segments) + offset;
				unsigned int indx4 = r * n_rot_segments + t + offset;

				triangle_indices[2 * (t + r * n_rot_segments) + n_rot_segments] = TriIndices{ indx1, indx2, indx3, Material::default_material() };
				triangle_indices[2 * (t + r * n_rot_segments) + 1 + n_rot_segments] = TriIndices{ indx1, indx3, indx4, Material::default_material() };
			}
		}

		return MeshInput{ triangle_indices, points };
	}

	MeshInput generateMeshInputFromMesh(const Mesh& m, mthz::Vec3 position, double scaling) {
		std::vector<mthz::Vec3> points;
		std::vector<TriIndices> triangle_indices;

		points.reserve(m.vertices.size());
		triangle_indices.reserve(m.face_indices.size());

		for (mthz::Vec3 v : m.vertices) {
			points.push_back(position + scaling * v);
		}
		for (const std::vector<unsigned int>& face : m.face_indices) {
			assert(face.size() == 3);
			triangle_indices.push_back(TriIndices{face[0], face[1], face[2], Material::default_material()});
		}

		return MeshInput{ triangle_indices, points };
	}

	StaticMeshGeometry::StaticMeshGeometry(const StaticMeshGeometry& c)
		: aabb_tree(0, AABBTree<unsigned int>::SURFACE_AREA), triangles(c.triangles)
	{
		for (int i = 0; i < triangles.size(); i++) {
			aabb_tree.add(i, i, triangles[i].aabb);
		}
	}

	StaticMeshGeometry::StaticMeshGeometry(const MeshInput& input)
		: aabb_tree(0)
	{
		const int NO_ASSIGNED_ID = -1;
		struct Edge {
			Edge() {}

			Edge(unsigned int p1_indx, unsigned int p2_indx, unsigned int opposite_point_indx) 
				: p1_indx(p1_indx), p2_indx(p2_indx), opposite_point_indx(opposite_point_indx), assigned_id(NO_ASSIGNED_ID)
			{
				unsigned int edge_min_indx, edge_max_indx;
				if (p1_indx > p2_indx) {
					edge_max_indx = p1_indx;
					edge_min_indx = p2_indx;
				}
				else {
					edge_max_indx = p2_indx;
					edge_min_indx = p1_indx;
				}

				key = ((uint64_t(edge_max_indx) << 32) & 0xFFFFFFFF00000000) + edge_min_indx;
			}

			unsigned int p1_indx, p2_indx;
			unsigned int opposite_point_indx;
			int assigned_id = NO_ASSIGNED_ID;
			uint64_t key;

			bool isCompliment(Edge e) {
				return p1_indx == e.p2_indx && p2_indx == e.p1_indx;
			}
		};

		struct TriangleGraphNode {
			unsigned int p1_indx, p2_indx, p3_indx;
			int tri_neighbor_indices[3]; //neighbor1 shares p1 p2 edge, neighbor2 shared p2 p3 edge, neighbor3 shares p3 p1 edge
			Edge edges[3];
			mthz::Vec3 normal;
			Material material;
		};

		std::vector<TriangleGraphNode> neighbor_graph;
		neighbor_graph.reserve(input.triangle_indices.size());
		for (TriIndices t : input.triangle_indices) {
			mthz::Vec3 v1 = input.points[t.i2] - input.points[t.i1];
			mthz::Vec3 v2 = input.points[t.i3] - input.points[t.i1];
			mthz::Vec3 normal = v1.cross(v2).normalize();
			neighbor_graph.push_back(TriangleGraphNode{ t.i1, t.i2, t.i3, {-1, -1, -1}, {Edge{t.i1, t.i2, t.i3}, Edge{t.i2, t.i3, t.i1}, Edge{t.i3, t.i1, t.i2}}, normal, t.material });
		}

		std::unordered_map<uint64_t, unsigned int> edge_face_map;

		for (int i = 0; i < neighbor_graph.size(); i++) {

			for (int j = 0; j < 3; j++) {
				Edge e = neighbor_graph[i].edges[j];

				auto query = edge_face_map.find(e.key);
				if (query != edge_face_map.end()) {
					neighbor_graph[i].tri_neighbor_indices[j] = query->second;

					for (int k = 0; k < 3; k++) {
						if (neighbor_graph[query->second].edges[k].key == e.key) {
							neighbor_graph[query->second].tri_neighbor_indices[k] = i;
						}
					}

					edge_face_map.erase(query);
				}
				else {
					edge_face_map[e.key] = i;
				}

				
			}

		}

		int next_triangle_id = 0;
		int vertex_id_offset = input.triangle_indices.size();
		int next_edge_id = input.triangle_indices.size() + input.points.size();
		//using the neighbor graph to compute all the finalized StaticMeshTri objects
		//neighbor info is needed to determine the gauss arcs for valid edge collisions.
		triangles.reserve(neighbor_graph.size());

		for (TriangleGraphNode t : neighbor_graph) {

			StaticMeshFace tri;
			tri.concave_neighbor_count = 0;
			tri.normal = t.normal;

			tri.vertices[0] = StaticMeshVertex{ input.points[t.p1_indx], (int) t.p1_indx + vertex_id_offset };
			tri.vertices[1] = StaticMeshVertex{ input.points[t.p2_indx], (int) t.p2_indx + vertex_id_offset };
			tri.vertices[2] = StaticMeshVertex{ input.points[t.p3_indx], (int) t.p3_indx + vertex_id_offset };

			tri.material = t.material;
			tri.id = next_triangle_id++;

			for (int i = 0; i < 3; i++) {
				tri.edges[i] = StaticMeshEdge{ tri.vertices[i].p, tri.vertices[(i + 1) % 3].p };
				tri.edges[i].out_direction = (tri.edges[i].p2 - tri.edges[i].p1).cross(tri.normal).normalize();

				if (t.edges[i].assigned_id == NO_ASSIGNED_ID) {
					t.edges[i].assigned_id = next_edge_id++;
				}
				tri.edges[i].id = t.edges[i].assigned_id;

				if (t.tri_neighbor_indices[i] == -1) { //no neighboring triangle on the edge
					tri.gauss_region.push_back(tri.edges[i].out_direction);
				}
				else {
					//neighbors version of the same edge
					Edge complimentary_edge;
					for (int j = 0; j < 3; j++) {
						Edge neighbor_edge = neighbor_graph[t.tri_neighbor_indices[i]].edges[j];
						if (t.edges[i].isCompliment(neighbor_edge)) {
							complimentary_edge = neighbor_edge;
						}
					}

					mthz::Vec3 this_opposite_tip = input.points[t.edges[i].opposite_point_indx];
					mthz::Vec3 neighbor_opposite_tip = input.points[complimentary_edge.opposite_point_indx];

					double EPS = 0.00001;
					bool edge_concave = (neighbor_opposite_tip - this_opposite_tip).normalize().dot(t.normal) >= -EPS;
					if (edge_concave) {
						tri.concave_neighbor_count++;
						if (tri.concave_neighbor_count <= 1) {
							tri.gauss_region.push_back(tri.normal);
						}
					}
					else {
						tri.gauss_region.push_back(neighbor_graph[t.tri_neighbor_indices[i]].normal);
					}
				}
			}

			tri.aabb = tri.computeAABB();
			//tri.concave_neighbor_count = 3;

			triangles.push_back(tri);
		}
	}

	void StaticMeshGeometry::recomputeFromReference(const StaticMeshGeometry& reference, const mthz::Mat3& rot, mthz::Vec3 trans, mthz::Vec3 center_of_rotation) {
		aabb_tree = AABBTree<unsigned int>(0, AABBTree<unsigned int>::SURFACE_AREA); //reset tree

		assert(triangles.size() == reference.triangles.size());
		for (int i = 0; i < triangles.size(); i++) {
			triangles[i].normal = rot * reference.triangles[i].normal;
			

			for (int j = 0; j < triangles[i].gauss_region.size(); j++) {
				triangles[i].gauss_region[j] = rot * reference.triangles[i].gauss_region[j];
			}

			for (int j = 0; j < 3; j++) {
				triangles[i].vertices[j].p = trans + rot * (reference.triangles[i].vertices[j].p - center_of_rotation) + center_of_rotation;

				triangles[i].edges[j].p1 = trans + rot * (reference.triangles[i].edges[j].p1 - center_of_rotation) + center_of_rotation;
				triangles[i].edges[j].p2 = trans + rot * (reference.triangles[i].edges[j].p2 - center_of_rotation) + center_of_rotation;

				triangles[i].edges[j].out_direction = rot * reference.triangles[i].edges[j].out_direction;
			}

			aabb_tree.add(i, i, triangles[i].aabb);
		}
	}

	AABB StaticMeshGeometry::genAABB() const {
		assert(triangles.size() > 0);

		AABB out = triangles[0].aabb;
		for (int i = 1; i < triangles.size(); i++) {
			out = AABB::combine(out, triangles[i].aabb);
		}

		return out;
	}

	RayQueryReturn StaticMeshGeometry::testRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) {
		std::vector<unsigned int> hit_candidates = aabb_tree.raycastHitCandidates(ray_origin, ray_dir);
		
		RayQueryReturn closest_hit{ false }; //false signifies no confirmed hit so far

		for (unsigned int i : hit_candidates) {
			const StaticMeshFace& tri = triangles[i];
			if (abs(tri.normal.dot(ray_dir)) < 0.0000000001) {
				continue;
			}

			//calculate dist where ray intersects the plane the triangle sits on
			double t = -(ray_origin - tri.vertices[1].p).dot(tri.normal) / ray_dir.dot(tri.normal);
			if (t < 0 || (closest_hit.did_hit && closest_hit.intersection_dist < t)) {
				continue;
			}

			mthz::Vec3 hit_pos = ray_origin + t * ray_dir;

			//check the intersection point lies inside the triangle
			if ((hit_pos - tri.edges[0].p1).dot(tri.edges[0].out_direction) > 0
				|| (hit_pos - tri.edges[1].p1).dot(tri.edges[1].out_direction) > 0
				|| (hit_pos - tri.edges[2].p1).dot(tri.edges[2].out_direction) > 0) {
				continue;
			}

			closest_hit = RayQueryReturn{ true, hit_pos, t };
		}

		return closest_hit;
	}
}