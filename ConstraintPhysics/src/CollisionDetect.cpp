#include "CollisionDetect.h"
#include "ConvexPrimitive.h"
#include "Geometry.h"

#include <cassert>

uint32_t getEdgeID(uint16_t p1_id, uint16_t p2_id) {
	int min, max;
	if (p1_id < p2_id) {
		min = p1_id;
		max = p2_id;
	}
	else {
		min = p2_id;
		max = p1_id;
	}

	return (0x0000FFFF & min) + (0xFFFF0000 & ((uint32_t)max << 16));
}

namespace phyz {

	bool operator==(const MagicID& m1, const MagicID& m2) {
		return m1.bID == m2.bID && m1.cID == m2.cID;
	}

	MagicID swapOrder(const MagicID m) {
		uint64_t c1_cID = (0x00000000FFFFFFFF & m.cID);
		uint64_t c2_cID = (0xFFFFFFFF00000000 & m.cID) >> 32;
		uint64_t c1_bID = (0x00000000FFFFFFFF & m.bID);
		uint64_t c2_bID = (0xFFFFFFFF00000000 & m.bID) >> 32;

		MagicID out;
		out.cID = (0x00000000FFFFFFFF | c2_cID) + (0xFFFFFFFF00000000 & (c1_cID << 32));
		out.bID = (0x00000000FFFFFFFF | c2_bID) + (0xFFFFFFFF00000000 & (c1_bID << 32));

		return out;
	}

	static Manifold SAT_PolyPoly(const Polyhedron& a, int a_id, const Material& a_mat, const Polyhedron& b, int b_id, const Material& b_mat);
	static Manifold detectSphereSphere(const Sphere& a, int a_id, const Material& a_mat, const Sphere& b, int b_id, const Material& b_mat);
	static Manifold SAT_PolySphere(const Polyhedron& a, int a_id, const Material& a_mat, const Sphere& b, int b_id, const Material& b_mat);
	static Manifold detectSphereCylinder(const Sphere& a, int a_id, const Material& a_mat, const Cylinder& b, int b_id, const Material& b_mat);
	static std::vector<Manifold> SAT_PolyMesh(const Polyhedron& a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation);
	static std::vector<Manifold> SAT_SphereMesh(const Sphere& a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation);

	Manifold detectCollision(const ConvexPrimitive& a, const ConvexPrimitive& b) {
		switch (a.getType()) {
		case POLYHEDRON:
			switch (b.getType()) {
			case POLYHEDRON:
				return SAT_PolyPoly((const Polyhedron&)*a.getGeometry(), a.getID(), a.material, (const Polyhedron&)*b.getGeometry(), b.getID(), b.material);

			case SPHERE:
				return SAT_PolySphere((const Polyhedron&)*a.getGeometry(), a.getID(), a.material, (const Sphere&)*b.getGeometry(), b.getID(), b.material);
			}
			break;
		case SPHERE:
			switch (b.getType()) {
			case POLYHEDRON:
			{
				Manifold out = SAT_PolySphere((const Polyhedron&)*b.getGeometry(), b.getID(), b.material, (const Sphere&)*a.getGeometry(), a.getID(), a.material);
				out.normal = -out.normal; //physics engine expects the manifold to be facing away from a. SAT_PolySphere generates normal facing away from the polyhedron
				for (ContactP& p : out.points) {
					p.magicID = swapOrder(p.magicID);
				}
				return out;
			}
			case SPHERE:
				return detectSphereSphere((const Sphere&)*a.getGeometry(), a.getID(), a.material, (const Sphere&)*b.getGeometry(), b.getID(), b.material);
			case CYLINDER:
				return detectSphereCylinder((const Sphere&)*a.getGeometry(), a.getID(), a.material, (const Cylinder&)*b.getGeometry(), b.getID(), b.material);
			}
			break;
		case CYLINDER:
			switch (b.getType()) {
			case SPHERE:
				Manifold out = detectSphereCylinder((const Sphere&)*b.getGeometry(), b.getID(), b.material, (const Cylinder&)*a.getGeometry(), a.getID(), a.material);
				out.normal = -out.normal; //physics engine expects the manifold to be facing away from a. detectSphereCylinder generates normal facing away from the sphere
				for (ContactP& p : out.points) {
					p.magicID = swapOrder(p.magicID);
				}
				return out;
			}
		}
		
	}

	std::vector<Manifold> detectCollision(const ConvexPrimitive& a, AABB a_aabb, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation) {
		switch (a.getType()) {
		case POLYHEDRON:
			return SAT_PolyMesh((const Polyhedron&)*a.getGeometry(), a_aabb, a.getID(), a.material, b, b_world_position, b_world_orientation);
		case SPHERE:
			return SAT_SphereMesh((const Sphere&)*a.getGeometry(), a_aabb, a.getID(), a.material, b, b_world_position, b_world_orientation);
		}
	}

	std::vector<Manifold> detectCollision(const StaticMeshGeometry& a, mthz::Vec3 a_world_position, mthz::Quaternion a_world_orientation, const ConvexPrimitive& b, AABB b_aabb) {
		std::vector<Manifold> out;
		switch (b.getType()) {
		case POLYHEDRON:
			out = SAT_PolyMesh((const Polyhedron&)*b.getGeometry(), b_aabb, b.getID(), b.material, a, a_world_position, a_world_orientation);
			break;
		case SPHERE:
			out = SAT_SphereMesh((const Sphere&)*b.getGeometry(), b_aabb, b.getID(), b.material, a, a_world_position, a_world_orientation);
			break;
		}

		for (Manifold& m : out) {
			m.normal = -m.normal; //physics engine expects the manifold to be facing away from a. SAT_PolySphere generates normal facing away from the polyhedron
			for (ContactP& p : m.points) {
				p.magicID = swapOrder(p.magicID);
			}
		}
		return out;
	}

	static struct CheckNormResults {
		int a_maxPID;
		int b_maxPID;
		mthz::Vec3 norm;
		double pen_depth;

		inline bool seprAxisExists() { return pen_depth < 0; }
	};

	static struct ProjP {
		double u;
		double w;
	};

	static struct ContactArea {
		std::vector<ProjP> ps;
		std::vector<int> p_IDs;
		int surfaceID;
	};

	static inline ContactArea projectFace(const Surface& s, mthz::Vec3 u, mthz::Vec3 w) {
		ContactArea out = { std::vector<ProjP>(), std::vector<int>(), -1 };

		int n_points = s.n_points();
		out.ps = std::vector<ProjP>(n_points);
		out.p_IDs = std::vector<int>(n_points);
		for (int i = 0; i < n_points; i++) {
			mthz::Vec3 v = s.getPointI(i);
			out.ps[i] = ProjP{ v.dot(u), v.dot(w) };
			out.p_IDs[i] = s.point_indexes[i];
		}
		out.surfaceID = s.getSurfaceID();

		return out;
	}

	static inline ContactArea projectEdge(const Edge& e, mthz::Vec3 n, mthz::Vec3 p, mthz::Vec3 u, mthz::Vec3 w) {
		ContactArea out = { std::vector<ProjP>(), std::vector<int>(), -1 };

		out.ps = { ProjP{ e.p1().dot(u), e.p1().dot(w) }, ProjP{ e.p2().dot(u), e.p2().dot(w) } };
		out.p_IDs = { e.p1_indx, e.p2_indx };

		return out;
	}

	static ContactArea findContactArea(const Polyhedron& c, mthz::Vec3 n, mthz::Vec3 p, int p_ID, mthz::Vec3 u, mthz::Vec3 w) {

		int best_surface_index = -1;
		double best_surface_tolerance = 1.0;
		for (int surface_index : c.getFaceIndicesAdjacentToPointI(p_ID)) {
			const Surface& s = c.getSurfaces()[surface_index];
			double cos_ang = s.normal().dot(n);
			if (1 - cos_ang < best_surface_tolerance) {
				best_surface_tolerance = 1 - cos_ang;
				best_surface_index = surface_index;
			}
		}
		if (best_surface_tolerance <= COS_TOL) {
			return projectFace(c.getSurfaces()[best_surface_index], u, w);
		}

		int best_edge_index = -1;
		double best_edge_tolerance = 1.0;
		for (int edge_index : c.getEdgeIndicesAdjacentToPointI(p_ID)) {
			const Edge& e = c.getEdges()[edge_index];
			double sin_ang = abs((e.p2() - e.p1()).normalize().dot(n));
			if (sin_ang < best_edge_tolerance) {
				best_edge_tolerance = sin_ang;
				best_edge_index = edge_index;
			}
		}

		if (best_edge_tolerance <= SIN_TOL) {
			return projectEdge(c.getEdges()[best_edge_index], n, p, u, w);
		}

		return ContactArea{
			{ ProjP{ p.dot(u), p.dot(w) } },
			{ p_ID },
			-1
		};
		
	}

	static bool pInside(const std::vector<ProjP> poly, ProjP p) {
		int left = 0;
		int right = 0;

		for (int i = 0; i < poly.size(); i++) {
			ProjP v1 = poly[i];
			ProjP v2 = poly[(i + 1) % poly.size()];


			if (v1.w == p.w) {
				//c2 will be checked for this check next iteration
				if (v1.u > p.u) {
					right++;
				}
				else {
					left++;
				}
			}
			else if ((v1.w - p.w) * (v2.w - p.w) < 0) {
				double intr_u = v1.u + (p.w - v1.w) * (v2.u - v1.u) / (v2.w - v1.w);
				if (intr_u > p.u) {
					right++;
				}
				else {
					left++;
				}
			}
		}

		return (left % 2 != 0) && (right % 2 != 0);
	}

	static bool findIntersection(ProjP a1, ProjP a2, ProjP b1, ProjP b2, ProjP* out) {
		double ma = (a2.w - a1.w) / (a2.u - a1.u);
		double mb = (b2.w - b1.w) / (b2.u - b1.u);

		//edge cases
		if (a1.u == a2.u) {
			double du = a1.u - b1.u;
			double wIntr = b1.w + du * mb;
			if ((b1.u - a1.u) * (b2.u - a1.u) <= 0 && (a1.w - wIntr) * (a2.w - wIntr) <= 0) {
				*out = ProjP{ a1.u, b1.w + (a1.u - b1.u) * (b2.w - b1.w) / (b2.u - b1.u) };
				return true;
			}
			return false;
		}
		if (b1.u == b2.u) {
			double du = b1.u - a1.u;
			double wIntr = a1.w + du * ma;
			if ((a1.u - b1.u) * (a2.u - b1.u) <= 0 && (b1.w - wIntr) * (b2.w - wIntr) <= 0) {
				*out = ProjP{ b1.u, a1.w + (b1.u - a1.u) * (a2.w - a1.w) / (a2.u - a1.u) };
				return true;
			}
			return false;
		}

		double dw0 = b1.w + mb * (a1.u - b1.u) - a1.w;
		double du = dw0 / (ma - mb);
		*out = ProjP{ a1.u + du, a1.w + ma * du };

		//check intersection is in bounds
		if ((out->u - a1.u) * (out->u - a2.u) >= 0 || (out->u - b1.u) * (out->u - b2.u) >= 0) {
			return false;
		}
		return true;
	}

	ExtremaInfo recenter(const ExtremaInfo& info, double old_ref_value, double new_ref_value) {
		double diff = new_ref_value - old_ref_value;
		return ExtremaInfo{ info.min_pID, info.max_pID, info.min_val + diff, info.max_val + diff};
	}

	ExtremaInfo findExtrema(const Polyhedron& c, mthz::Vec3 axis) {
		ExtremaInfo extrema;

		for (int i = 0; i < c.getPoints().size(); i++) {
			mthz::Vec3 p = c.getPoints()[i];
			double val = p.dot(axis);
			if (val < extrema.min_val) {
				extrema.min_pID = i;
				extrema.min_val = val;
			}
			if (val > extrema.max_val) {
				extrema.max_pID = i;
				extrema.max_val = val;
			}
		}

		return extrema;
	}

	ExtremaInfo getSphereExtrema(const Sphere& s, mthz::Vec3 dir) {
		ExtremaInfo out;
		double center_val = dir.dot(s.getCenter());
		out.max_val = center_val + s.getRadius();
		out.min_val = center_val - s.getRadius();
		out.min_pID = -1;
		out.max_pID = -1;

		return out;
	}

	ExtremaInfo getCylinderExtrema(const Cylinder& c, mthz::Vec3 dir) {
		ExtremaInfo out;
		mthz::Vec3 center = c.getCenter();
		mthz::Vec3 height_axis = c.getHeightAxis();
		double height = c.getHeight();
		double radius = c.getRadius();
		mthz::Vec3 topdisk_center = center + height / 2.0 * height_axis;

		//exploiting symmetry
		mthz::Vec3 topdisk_max = Cylinder::getExtremaOfDisk(topdisk_center, height_axis, radius, dir);
		mthz::Vec3 topdisk_min = 2 * topdisk_center - topdisk_max;
		mthz::Vec3 botdisk_max = 2 * center - topdisk_min;
		mthz::Vec3 botdisk_min = 2 * center - topdisk_max;

		
		double topdisk_max_v = dir.dot(topdisk_max);
		double topdisk_min_v = dir.dot(topdisk_min);
		double botdisk_max_v = dir.dot(botdisk_max);
		double botdisk_min_v = dir.dot(botdisk_min);

		out.max_val = std::max<double>(topdisk_max_v, botdisk_max_v);
		out.min_val = std::min<double>(topdisk_min_v, botdisk_min_v);
		out.min_pID = -1;
		out.max_pID = -1;

		return out;
	}

	static CheckNormResults sat_checknorm(const ExtremaInfo& a_info, const ExtremaInfo& b_info, mthz::Vec3 n) {
		double forward_pen_depth = a_info.max_val - b_info.min_val;
		double reverse_pen_depth = b_info.max_val - a_info.min_val;

		if (forward_pen_depth < reverse_pen_depth) {
			return CheckNormResults{ a_info.max_pID, b_info.min_pID, n, forward_pen_depth };
		}
		else {
			return CheckNormResults{ a_info.min_pID, b_info.max_pID, -n, reverse_pen_depth };
		}
	}

	static Manifold SAT_PolyPoly(const Polyhedron& a, int a_id, const Material& a_mat, const Polyhedron& b, int b_id, const Material& b_mat) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
		const GaussMap& ag = a.getGaussMap();
		const GaussMap& bg = b.getGaussMap();

		for (const GaussVert& g : ag.face_verts) {
			if (!g.SAT_redundant) {
				ExtremaInfo recentered_g_extrema = recenter(g.cached_SAT_query, g.SAT_reference_point_value, g.v.dot(a.getPoints()[g.SAT_reference_point_index]));
				CheckNormResults x = sat_checknorm(recentered_g_extrema, findExtrema(b, g.v), g.v);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}
		for (const GaussVert& g : bg.face_verts) {
			if (!g.SAT_redundant) {
				ExtremaInfo recentered_g_extrema = recenter(g.cached_SAT_query, g.SAT_reference_point_value, g.v.dot(b.getPoints()[g.SAT_reference_point_index]));
				CheckNormResults x = sat_checknorm(findExtrema(a, g.v), recentered_g_extrema, g.v);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}
		for (const GaussArc& arc1 : ag.arcs) {
			for (const GaussArc& arc2 : bg.arcs) {

				mthz::Vec3 a1 = ag.face_verts[arc1.v1_indx].v;
				mthz::Vec3 a2 = ag.face_verts[arc1.v2_indx].v;
				mthz::Vec3 b1 = -bg.face_verts[arc2.v1_indx].v;
				mthz::Vec3 b2 = -bg.face_verts[arc2.v2_indx].v;

				//check arcs arent on opposite hemispheres
				mthz::Vec3 a_avg = a1 + a2;
				if (a_avg.dot(b1) + a_avg.dot(b2) <= 0) {
					continue;
				}

				mthz::Vec3 a_perp = a1.cross(a2);
				mthz::Vec3 b_perp = b1.cross(b2);
				//check arc b1b2 crosses plane defined by a1a2 and vice verca
				if (a_perp.dot(b1) * a_perp.dot(b2) > 0 || b_perp.dot(a1) * b_perp.dot(a2) > 0) {
					continue;
				}

				mthz::Vec3 n = a_perp.cross(b_perp);
				if (n.magSqrd() == 0) {
					continue;
				}

				n = n.normalize();
				if (a_avg.dot(n) < 0) {
					n *= -1;
				}

				CheckNormResults x = sat_checknorm(findExtrema(a, n), findExtrema(b, n), n);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}

		out.normal = min_pen.norm;
		mthz::Vec3 norm = min_pen.norm;
		mthz::Vec3 a_maxP = a.getPoints()[min_pen.a_maxPID];
		mthz::Vec3 b_maxP = b.getPoints()[min_pen.b_maxPID];

		mthz::Vec3 u, w;
		norm.getPerpendicularBasis(&u, &w);
		ContactArea a_contact = findContactArea(a, norm, a_maxP, min_pen.a_maxPID, u, w);
		ContactArea b_contact = findContactArea(b, (-1) * norm, b_maxP, min_pen.b_maxPID, u, w);

		std::vector<ProjP> man_pool;
		std::vector<uint64_t> man_pool_magics;
		for (int i = 0; i < a_contact.ps.size(); i++) {
			ProjP p = a_contact.ps[i];
			if (pInside(b_contact.ps, p)) {
				man_pool.push_back(p);
				uint64_t m = 0;
				m |= 0x00000000FFFFFFFF & a_contact.p_IDs[i];
				m |= 0xFFFFFFFF00000000 & (uint64_t(b_contact.surfaceID) << 32);
				man_pool_magics.push_back(m);
			}
		}
		for (int i = 0; i < b_contact.ps.size(); i++) {
			ProjP p = b_contact.ps[i];
			if (pInside(a_contact.ps, p)) {
				man_pool.push_back(p);
				uint64_t m = 0;
				m |= 0x00000000FFFFFFFF & a_contact.surfaceID;
				m |= 0xFFFFFFFF00000000 & (uint64_t(b_contact.p_IDs[i]) << 32);
				man_pool_magics.push_back(m);
			}
		}
		if (a_contact.ps.size() >= 2 && b_contact.ps.size() >= 2) {
			int itr_max_a = (a_contact.ps.size() == 2) ? 1 : a_contact.ps.size();
			int itr_max_b = (b_contact.ps.size() == 2) ? 1 : b_contact.ps.size();
			for (int i = 0; i < itr_max_a; i++) {
				int a2_indx = (i + 1) % a_contact.ps.size();
				ProjP a1 = a_contact.ps[i];
				ProjP a2 = a_contact.ps[a2_indx];
				int a1_ID = a_contact.p_IDs[i];
				int a2_ID = a_contact.p_IDs[a2_indx];

				for (int j = 0; j < itr_max_b; j++) {
					int b2_indx = (j + 1) % b_contact.ps.size();
					ProjP b1 = b_contact.ps[j];
					ProjP b2 = b_contact.ps[b2_indx];
					int b1_ID = b_contact.p_IDs[j];
					int b2_ID = b_contact.p_IDs[b2_indx];

					ProjP out;
					if (findIntersection(a1, a2, b1, b2, &out)) {
						man_pool.push_back(out);
						uint64_t m = 0;
						m |= 0x00000000FFFFFFFF & getEdgeID(a1_ID, a2_ID);
						m |= 0xFFFFFFFF00000000 & (uint64_t(getEdgeID(b1_ID, b2_ID)) << 32);
						man_pool_magics.push_back(m);
					}
				}
			}
		}

		double a_pen = min_pen.pen_depth;
		double a_dot_val = a_maxP.dot(norm);
		mthz::Vec3 n_offset = norm * a_dot_val;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b_id) << 32);

		out.points.reserve(man_pool.size());
		for (int i = 0; i < man_pool.size(); i++) {
			ProjP p = man_pool[i];
			ContactP cp;
			cp.pos = u * p.u + w * p.w + n_offset;
			cp.pen_depth = cp.pos.dot(norm) - a_dot_val + a_pen;
			cp.restitution = std::max<double>(a_mat.restitution, b_mat.restitution);
			cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b_mat.kinetic_friction_coeff) / 2.0;
			cp.static_friction_coeff = (a_mat.static_friction_coeff + b_mat.static_friction_coeff) / 2.0;
			cp.magicID = MagicID{ cID, man_pool_magics[i] };
			out.points.push_back(cp);
		}
		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	static Manifold detectSphereSphere(const Sphere& a, int a_id, const Material& a_mat, const Sphere& b, int b_id, const Material& b_mat) {
		Manifold out;
		mthz::Vec3 diff = b.getCenter() - a.getCenter();
		double center_distance = diff.mag();
		out.max_pen_depth = a.getRadius() + b.getRadius() - center_distance;
		
		//check if spheres touch or not
		if (out.max_pen_depth < 0) {
			return out;
		}

		out.normal = diff.normalize();

		ContactP cp;
		cp.pos = a.getCenter() + out.normal * a.getRadius();
		cp.pen_depth = out.max_pen_depth;
		
		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b_id) << 32);
		cp.restitution = std::max<double>(a_mat.restitution, b_mat.restitution);
		cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b_mat.kinetic_friction_coeff) / 2.0;
		cp.static_friction_coeff = (a_mat.static_friction_coeff + b_mat.static_friction_coeff) / 2.0;
		cp.magicID = MagicID{ cID, 0x0 }; //second term is used to identify different points or faces on polyhedron. just using flat 0 for spheres.

		out.points.push_back(cp);
		return out;
	}

	static Manifold SAT_PolySphere(const Polyhedron& a, int a_id, const Material& a_mat, const Sphere& b, int b_id, const Material& b_mat) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
		const GaussMap& gauss_map = a.getGaussMap();
		uint32_t a_feature_id;

		//very hacky using the fact that gauss verts have the same order as the corresponding surfaces they are made from. SurfaceID's start at points.size().
		int corresponding_surface_id = a.getPoints().size();
		for (const GaussVert& g : gauss_map.face_verts) {
			if (!g.SAT_redundant) {
				ExtremaInfo recentered_g_extrema = recenter(g.cached_SAT_query, g.SAT_reference_point_value, g.v.dot(a.getPoints()[g.SAT_reference_point_index]));
				CheckNormResults x = sat_checknorm(recentered_g_extrema, getSphereExtrema(b, g.v), g.v);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					a_feature_id = corresponding_surface_id;
					min_pen = x;
				}
			}

			corresponding_surface_id++;
		}
		for (int pID = 0; pID < a.getPoints().size(); pID++) {
			mthz::Vec3 p = a.getPoints()[pID];
			mthz::Vec3 n = (p - b.getCenter()).normalize();
			CheckNormResults x = sat_checknorm(findExtrema(a, n), getSphereExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
				a_feature_id = pID;
			}
		}
		for (const Edge& e : a.getEdges()) {
			mthz::Vec3 edge_dir = (e.p2() - e.p1()).normalize();
			mthz::Vec3 sample = e.p1() - b.getCenter();
			mthz::Vec3 n = (sample - edge_dir * edge_dir.dot(sample)).normalize();
			CheckNormResults x = sat_checknorm(findExtrema(a, n), getSphereExtrema(b, n), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
				a_feature_id = getEdgeID(e.p1_indx, e.p2_indx);
			}
		}

		out.normal = min_pen.norm;
		
		ContactP cp;
		cp.pos = b.getCenter() - out.normal * b.getRadius(); //sat_checknorm ensures normals always point away from a. to get normal pointing away from b - sign added
		cp.pen_depth = min_pen.pen_depth;
		cp.restitution = std::max<double>(a_mat.restitution, b_mat.restitution);
		cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b_mat.kinetic_friction_coeff) / 2.0;
		cp.static_friction_coeff = (a_mat.static_friction_coeff + b_mat.static_friction_coeff) / 2.0;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b_id) << 32);

		cp.magicID = MagicID{ cID, a_feature_id }; //not bothering with featureid
 
		out.points.push_back(cp);
		
		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	

	//still some room from optomization
	Manifold detectSphereCylinder(const Sphere& a, int a_id, const Material& a_mat, const Cylinder& b, int b_id, const Material& b_mat) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };

		mthz::Vec3 height_axis = b.getHeightAxis();
		{
			CheckNormResults x = sat_checknorm(getSphereExtrema(a, height_axis), getCylinderExtrema(b, height_axis), height_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		mthz::Vec3 diff = b.getCenter() - a.getCenter();
		mthz::Vec3 barrel_axis = (diff - height_axis * height_axis.dot(diff)).normalize();
		{
			CheckNormResults x = sat_checknorm(getSphereExtrema(a, barrel_axis), getCylinderExtrema(b, barrel_axis), barrel_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		
		mthz::Vec3 topdisk_center = b.getCenter() + height_axis * 0.5 * b.getHeight();
		mthz::Vec3 topdisk_close_point = Cylinder::getExtremaOfDisk(topdisk_center, height_axis, b.getRadius(), -diff.normalize());
		{
			mthz::Vec3 edge_axis = topdisk_close_point - a.getCenter();
			CheckNormResults x = sat_checknorm(getSphereExtrema(a, edge_axis), getCylinderExtrema(b, edge_axis), edge_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}
		mthz::Vec3 botdisk_center = b.getCenter() + height_axis * 0.5 * b.getHeight();
		mthz::Vec3 botdisk_close_point = Cylinder::getExtremaOfDisk(botdisk_center, height_axis, b.getRadius(), -diff.normalize());
		{
			mthz::Vec3 edge_axis = botdisk_close_point - a.getCenter();
			CheckNormResults x = sat_checknorm(getSphereExtrema(a, edge_axis), getCylinderExtrema(b, edge_axis), edge_axis);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}

		out.normal = min_pen.norm;

		mthz::Vec3 botdisk_center = b.getCenter() - height_axis * 0.5 * b.getHeight();

		ContactP cp;
		cp.pos = b.getCenter() - out.normal * b.getRadius(); //sat_checknorm ensures normals always point away from a. to get normal pointing away from b - sign added
		cp.pen_depth = min_pen.pen_depth;
		cp.restitution = std::max<double>(a_mat.restitution, b_mat.restitution);
		cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b_mat.kinetic_friction_coeff) / 2.0;
		cp.static_friction_coeff = (a_mat.static_friction_coeff + b_mat.static_friction_coeff) / 2.0;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b_id) << 32);

		cp.magicID = MagicID{ cID, a_feature_id }; //not bothering with featureid

		out.points.push_back(cp);

		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	//anti tunneling weighted is for trying to force penetration in the correct direction. It can give an incorrect "does seperating axis exist" test though
	static ExtremaInfo findTriangleExtrema(const StaticMeshFace& tri, mthz::Vec3 dir, bool anti_tunneling_weighted) {
		ExtremaInfo extrema;
		if (anti_tunneling_weighted) {
			if (dir.dot(tri.normal) > 0) {
				extrema.min_val = -std::numeric_limits<double>::infinity();
			}
			else {
				extrema.max_val = std::numeric_limits<double>::infinity();
			}
		}

		for (int i = 0; i < 3; i++) {
			mthz::Vec3 p = tri.vertices[i].p;
			double val = p.dot(dir);
			if (val < extrema.min_val) {
				extrema.min_pID = i;
				extrema.min_val = val;
			}
			if (val > extrema.max_val) {
				extrema.max_pID = i;
				extrema.max_val = val;
			}
		}

		return extrema;
	}

	static inline ContactArea projectTriangleFace(const StaticMeshFace& s, mthz::Vec3 u, mthz::Vec3 w) {
		ContactArea out = { std::vector<ProjP>(), std::vector<int>(), -1 };

		out.ps = std::vector<ProjP>(3);
		out.p_IDs = std::vector<int>(3);
		for (int i = 0; i < 3; i++) {
			mthz::Vec3 v = s.vertices[i].p;
			out.ps[i] = ProjP{ v.dot(u), v.dot(w) };
			out.p_IDs[i] = i;
		}
		out.surfaceID = s.id;

		return out;
	}

	static inline ContactArea projectTriangleEdge(const StaticMeshEdge& e, mthz::Vec3 n, mthz::Vec3 p, mthz::Vec3 u, mthz::Vec3 w) {
		ContactArea out = { std::vector<ProjP>(), std::vector<int>(), -1 };

		out.ps = { ProjP{ e.p1.dot(u), e.p1.dot(w) }, ProjP{ e.p2.dot(u), e.p2.dot(w) } };
		out.p_IDs = { e.id, e.id };

		return out;
	}

	static ContactArea findTriangleContactArea(const StaticMeshFace& t, mthz::Vec3 n, mthz::Vec3 p, int p_ID, mthz::Vec3 u, mthz::Vec3 w) {

		double cos_ang = t.normal.dot(n);
		if (1 - cos_ang <= COS_TOL) {
			return projectTriangleFace(t, u, w);
		}		

		for (int i = 0; i < 3; i++) {
			if (i != p_ID && (i + 1) % 3 != p_ID) {
				continue;
			}

			StaticMeshEdge e = t.edges[i];
			double sin_ang = abs((e.p2 - e.p1).normalize().dot(n));
			if (sin_ang <= SIN_TOL) {
				return projectTriangleEdge(e, n, p, u, w);
			}
		}

		return ContactArea{
			{ ProjP{ p.dot(u), p.dot(w) } },
			{ p_ID },
			-1
		};

	}

	//consider neighboring triangles when picking the axis of least penetration
	static bool normalDirectionValid(const StaticMeshFace& s, mthz::Vec3 normal) {
		//return true;
		double EPS = 0.0001;

		switch (s.concave_neighbor_count) {
		case 0:
		case 1:
			//the three points in counter-clockwise widning define a region on the surface of the sphere. the normal should lie in that surface to be valid
			assert(s.gauss_region.size() == 3);
			for (int i = 0; i < s.gauss_region.size(); i++) {
				mthz::Vec3 inner_region_direction = s.gauss_region[i].cross(s.gauss_region[(i + 1) % s.gauss_region.size()]);
				if (normal.dot(inner_region_direction) < 0) return false;
			}
			break;
		case 2:
		{
			//the normal should lie on the arc defined by the two points
			assert(s.gauss_region.size() == 2);
			mthz::Vec3 arc_normal = s.gauss_region[0].cross(s.gauss_region[1]);
			//check vector lies close to the plane
			if (abs(normal.dot(arc_normal)) > EPS) return false;
			mthz::Vec3 v0_up = arc_normal.cross(s.gauss_region[0]);

			//check vector doesnt lie outside the arc within the plane
			if (normal.dot(v0_up) < 0) return false;
			mthz::Vec3 v1_down = s.gauss_region[1].cross(arc_normal);
			if (normal.dot(v1_down) < 0) return false;
			break;
		}
		case 3:
			assert(s.gauss_region.size() == 1);
			if (normal.dot(s.normal) < 1 - EPS) return false; //s.normal is only valid direction
			break;
		}
		
		return true;
	}

	static Manifold SAT_PolyTriangle(const Polyhedron& a, int a_id, const Material& a_mat, const StaticMeshFace& b, double non_gauss_valid_penalty) {
		Manifold out;
		out.max_pen_depth = -1;
		CheckNormResults min_gauss_valid_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
		const GaussMap& ag = a.getGaussMap();

		ExtremaInfo poly_info = findExtrema(a, b.normal);
		CheckNormResults b_norm_x = sat_checknorm(poly_info, findTriangleExtrema(b, b.normal, false), b.normal);
		if (b_norm_x.seprAxisExists()) {
			out.max_pen_depth = -1;
			return out;
		}
		b_norm_x = sat_checknorm(poly_info, findTriangleExtrema(b, b.normal, true), b.normal);
		if (b_norm_x.pen_depth < min_pen.pen_depth) { //no normalDirectionValid check needed for this direction
			min_pen = b_norm_x;
		}
		if (b_norm_x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -b_norm_x.norm)) {
			min_gauss_valid_pen = b_norm_x;
		}

		for (const GaussVert& g : ag.face_verts) {
			if (!g.SAT_redundant) {
				ExtremaInfo recentered_g_extrema = recenter(g.cached_SAT_query, g.SAT_reference_point_value, g.v.dot(a.getPoints()[g.SAT_reference_point_index]));
				CheckNormResults x = sat_checknorm(recentered_g_extrema, findTriangleExtrema(b, g.v, false), g.v);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				x = sat_checknorm(recentered_g_extrema, findTriangleExtrema(b, g.v, true), g.v);
				if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
				if (x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -x.norm)) {
					min_gauss_valid_pen = x;
				}
			}
		}
		
		struct SimpleArc {
			mthz::Vec3 gv1, gv2;
		};

		std::vector<SimpleArc> triangle_arcs =
		{
			SimpleArc{b.normal, b.edges[0].out_direction}, SimpleArc{b.edges[0].out_direction, -b.normal},
			SimpleArc{b.normal, b.edges[1].out_direction}, SimpleArc{b.edges[1].out_direction, -b.normal},
			SimpleArc{b.normal, b.edges[2].out_direction}, SimpleArc{b.edges[2].out_direction, -b.normal},
		};

		for (const GaussArc& arc1 : ag.arcs) {
			for (SimpleArc sa : triangle_arcs) {

				mthz::Vec3 a1 = ag.face_verts[arc1.v1_indx].v;
				mthz::Vec3 a2 = ag.face_verts[arc1.v2_indx].v;
				mthz::Vec3 b1 = -sa.gv1;
				mthz::Vec3 b2 = -sa.gv2;

				//check arcs arent on opposite hemispheres
				mthz::Vec3 a_avg = a1 + a2;
				if (a_avg.dot(b1) + a_avg.dot(b2) <= 0) {
					continue;
				}

				mthz::Vec3 a_perp = a1.cross(a2);
				mthz::Vec3 b_perp = b1.cross(b2);
				//check arc b1b2 crosses plane defined by a1a2 and vice verca
				if (a_perp.dot(b1) * a_perp.dot(b2) > 0 || b_perp.dot(a1) * b_perp.dot(a2) > 0) {
					continue;
				}

				mthz::Vec3 n = a_perp.cross(b_perp);
				if (n.magSqrd() == 0) {
					continue;
				}

				n = n.normalize();
				if (a_avg.dot(n) < 0) {
					n *= -1;
				}

				ExtremaInfo poly_extrema = findExtrema(a, n);
				CheckNormResults x = sat_checknorm(poly_extrema, findTriangleExtrema(b, n, false), n);
				if (x.seprAxisExists()) {
					out.max_pen_depth = -1;
					return out;
				}
				x = sat_checknorm(poly_extrema, findTriangleExtrema(b, n, true), n);
				if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
				if (x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -x.norm)) {
					min_gauss_valid_pen = x;
				}
			}
		}

		CheckNormResults nongauss_min_pen = min_pen;
		if (min_gauss_valid_pen.pen_depth < min_pen.pen_depth + non_gauss_valid_penalty) min_pen = min_gauss_valid_pen;

		out.normal = min_pen.norm;
		mthz::Vec3 norm = min_pen.norm;
		mthz::Vec3 a_maxP = a.getPoints()[nongauss_min_pen.a_maxPID];
		mthz::Vec3 b_maxP = b.vertices[nongauss_min_pen.b_maxPID].p;

		mthz::Vec3 u, w;
		norm.getPerpendicularBasis(&u, &w);
		ContactArea a_contact = findContactArea(a, nongauss_min_pen.norm, a_maxP, nongauss_min_pen.a_maxPID, u, w);
		ContactArea b_contact = findTriangleContactArea(b, (-1) * nongauss_min_pen.norm, b_maxP, nongauss_min_pen.b_maxPID, u, w);

		std::vector<ProjP> man_pool;
		std::vector<uint64_t> man_pool_magics;
		for (int i = 0; i < a_contact.ps.size(); i++) {
			ProjP p = a_contact.ps[i];
			if (pInside(b_contact.ps, p)) {
				man_pool.push_back(p);
				uint64_t m = 0;
				m |= 0x00000000FFFFFFFF & a_contact.p_IDs[i];
				m |= 0xFFFFFFFF00000000 & (uint64_t(b_contact.surfaceID) << 32);
				man_pool_magics.push_back(m);
			}
		}
		for (int i = 0; i < b_contact.ps.size(); i++) {
			ProjP p = b_contact.ps[i];
			if (pInside(a_contact.ps, p)) {
				man_pool.push_back(p);
				uint64_t m = 0;
				m |= 0x00000000FFFFFFFF & a_contact.surfaceID;
				m |= 0xFFFFFFFF00000000 & (uint64_t(b_contact.p_IDs[i]) << 32);
				man_pool_magics.push_back(m);
			}
		}
		if (a_contact.ps.size() >= 2 && b_contact.ps.size() >= 2) {
			int itr_max_a = (a_contact.ps.size() == 2) ? 1 : a_contact.ps.size();
			int itr_max_b = (b_contact.ps.size() == 2) ? 1 : b_contact.ps.size();
			for (int i = 0; i < itr_max_a; i++) {
				int a2_indx = (i + 1) % a_contact.ps.size();
				ProjP a1 = a_contact.ps[i];
				ProjP a2 = a_contact.ps[a2_indx];
				int a1_ID = a_contact.p_IDs[i];
				int a2_ID = a_contact.p_IDs[a2_indx];

				for (int j = 0; j < itr_max_b; j++) {
					int b2_indx = (j + 1) % b_contact.ps.size();
					ProjP b1 = b_contact.ps[j];
					ProjP b2 = b_contact.ps[b2_indx];
					int b1_ID = b_contact.p_IDs[j];
					int b2_ID = b_contact.p_IDs[b2_indx];

					ProjP out;
					if (findIntersection(a1, a2, b1, b2, &out)) {
						man_pool.push_back(out);
						uint64_t m = 0;
						m |= 0x00000000FFFFFFFF & getEdgeID(a1_ID, a2_ID);
						m |= 0xFFFFFFFF00000000 & (uint64_t(getEdgeID(b1_ID, b2_ID)) << 32);
						man_pool_magics.push_back(m);
					}
				}
			}
		}

		double a_pen = min_pen.pen_depth;
		double a_dot_val = a_maxP.dot(norm);
		mthz::Vec3 n_offset = norm * a_dot_val;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b.id) << 32);

		out.points.reserve(man_pool.size());
		for (int i = 0; i < man_pool.size(); i++) {
			ProjP p = man_pool[i];
			ContactP cp;
			cp.pos = u * p.u + w * p.w + n_offset;
			cp.pen_depth = cp.pos.dot(norm) - a_dot_val + a_pen;
			cp.restitution = std::max<double>(a_mat.restitution, b.material.restitution);
			cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b.material.kinetic_friction_coeff) / 2.0;
			cp.static_friction_coeff = (a_mat.static_friction_coeff + b.material.static_friction_coeff) / 2.0;
			cp.magicID = MagicID{ cID, man_pool_magics[i] };
			out.points.push_back(cp);
		}
		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	static Manifold SAT_SphereTriangle(const Sphere& a, int a_id, const Material& a_mat, const StaticMeshFace& b, double non_gauss_valid_penalty) {
		Manifold out;
		out.max_pen_depth = -1;
		uint32_t a_feature_id = -1;
		CheckNormResults min_gauss_valid_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };

		ExtremaInfo sphere_extrema = getSphereExtrema(a, b.normal);
		CheckNormResults b_norm_x = sat_checknorm(sphere_extrema, findTriangleExtrema(b, b.normal, false), b.normal);
		if (b_norm_x.seprAxisExists()) {
			out.max_pen_depth = -1;
			return out;
		}
		b_norm_x = sat_checknorm(sphere_extrema, findTriangleExtrema(b, b.normal, true), b.normal);
		if (b_norm_x.pen_depth < min_pen.pen_depth) {
			min_pen = b_norm_x;
		}
		if (b_norm_x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -b_norm_x.norm)) {
			min_gauss_valid_pen = b_norm_x;
		}

		for (int i = 0; i < 3; i++) {
			mthz::Vec3 p = b.vertices[i].p;
			mthz::Vec3 n = (p - a.getCenter()).normalize();
			ExtremaInfo sphere_extrema = getSphereExtrema(a, n);
			CheckNormResults x = sat_checknorm(sphere_extrema, findTriangleExtrema(b, n, false), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			x = sat_checknorm(sphere_extrema, findTriangleExtrema(b, n, true), n);
			if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
				a_feature_id = i;
			}
			if (x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -x.norm)) {
				min_gauss_valid_pen = x;
			}
		}
		for (StaticMeshEdge e : b.edges) {
			mthz::Vec3 edge_dir = (e.p2 - e.p1).normalize();
			mthz::Vec3 sample = e.p1 - a.getCenter();
			mthz::Vec3 n = (sample - edge_dir * edge_dir.dot(sample)).normalize();
			ExtremaInfo sphere_extrema = getSphereExtrema(a, n);
			CheckNormResults x = sat_checknorm(sphere_extrema, findTriangleExtrema(b, n, false), n);
			if (x.seprAxisExists()) {
				out.max_pen_depth = -1;
				return out;
			}
			x = sat_checknorm(sphere_extrema, findTriangleExtrema(b, n, true), n);
			if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
				a_feature_id = e.id;
			}
			if (x.pen_depth < min_gauss_valid_pen.pen_depth && normalDirectionValid(b, -x.norm)) {
				min_gauss_valid_pen = x;
			}
		}

		CheckNormResults nongauss_min_pen = min_pen;
		if (min_gauss_valid_pen.pen_depth < min_pen.pen_depth + non_gauss_valid_penalty) min_pen = min_gauss_valid_pen;
		out.normal = min_pen.norm;

		ContactP cp;
		cp.pos = a.getCenter() + min_pen.norm * a.getRadius();
		cp.pen_depth = min_pen.pen_depth;
		cp.restitution = std::max<double>(a_mat.restitution, b.material.restitution);
		cp.kinetic_friction_coeff = (a_mat.kinetic_friction_coeff + b.material.kinetic_friction_coeff) / 2.0;
		cp.static_friction_coeff = (a_mat.static_friction_coeff + b.material.static_friction_coeff) / 2.0;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a_id;
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b.id) << 32);

		cp.magicID = MagicID{ cID, a_feature_id }; //not bothering with featureid

		out.points.push_back(cp);

		out.max_pen_depth = min_pen.pen_depth;

		return out;
	}

	static std::vector<Manifold> SAT_PolyMesh(const Polyhedron& a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation) {
		mthz::Mat3 local_to_world_rot = b_world_orientation.getRotMatrix();

		std::vector<unsigned int> tri_candidates;
		bool local_transformation_required = b_world_position != mthz::Vec3() || b_world_orientation != mthz::Quaternion();
		if (!local_transformation_required) {
			tri_candidates = b.getAABBTree().getCollisionCandidatesWith(a_aabb);
		}
		else {
			//local basis transformed to world coordinates
			mthz::Vec3 u = local_to_world_rot * mthz::Vec3(1, 0, 0);
			mthz::Vec3 v = local_to_world_rot * mthz::Vec3(0, 1, 0);
			mthz::Vec3 w = local_to_world_rot * mthz::Vec3(0, 0, 1);

			AABB local_coord_aabb = AABB::conformNewBasis(a_aabb, u, v, w, b_world_position);
			tri_candidates = b.getAABBTree().getCollisionCandidatesWith(local_coord_aabb);
		}

		std::vector<Manifold> manifolds_out;

		for (unsigned int i : tri_candidates) {
			StaticMeshFace tri = local_transformation_required? b.getTriangles()[i].getTransformed(local_to_world_rot, b_world_position, mthz::Vec3()) : b.getTriangles()[i];
			double non_gauss_valid_normal_penalty = 0.1 * std::min<double>(AABB::longestDimension(a_aabb), AABB::longestDimension(tri.aabb)); //soft penalty to avoid internal collisions

			Manifold m = SAT_PolyTriangle(a, a_id, a_mat, tri, non_gauss_valid_normal_penalty);
			if (m.max_pen_depth > 0 && m.points.size() > 0) {
				manifolds_out.push_back(m);
			}
		}

		return manifolds_out;
	}

	static std::vector<Manifold> SAT_SphereMesh(const Sphere& a, AABB a_aabb, int a_id, const Material& a_mat, const StaticMeshGeometry& b, mthz::Vec3 b_world_position, mthz::Quaternion b_world_orientation) {
		std::vector<unsigned int> tri_candidates = b.getAABBTree().getCollisionCandidatesWith(a_aabb);
		std::vector<Manifold> manifolds_out;

		for (unsigned int i : tri_candidates) {
			const StaticMeshFace& tri = b.getTriangles()[i];
			double non_gauss_valid_normal_penalty = 0.4 * std::min<double>(AABB::longestDimension(a_aabb), AABB::longestDimension(tri.aabb)); //soft penalty to avoid internal collisions

			Manifold m = SAT_SphereTriangle(a, a_id, a_mat, tri, non_gauss_valid_normal_penalty);
			if (m.max_pen_depth > 0 && m.points.size() > 0) {
				manifolds_out.push_back(m);
			}
		}

		return manifolds_out;
	}

	Manifold merge_manifold(const Manifold& m1, const Manifold& m2) {
		Manifold out = { std::vector<ContactP>(m1.points.size() + m2.points.size()), mthz::Vec3(), std::max<double>(m1.max_pen_depth, m2.max_pen_depth)};
		out.normal = (m1.normal + m2.normal).normalize();

		for (int i = 0; i < m1.points.size(); i++) {
			out.points[i] = m1.points[i];
		}
		int off = m1.points.size();
		for (int i = 0; i < m2.points.size(); i++) {
			out.points[i + off] = m2.points[i];
		}

		return out;
	}

	Manifold cull_manifold(const Manifold& m, int new_size) {
		if (new_size >= m.points.size()) {
			return m;
		}
		Manifold out = { std::vector<ContactP>(new_size), m.normal, 0 };
		std::vector<bool> p_available(m.points.size(), true);

		mthz::Vec3 u, w;
		m.normal.getPerpendicularBasis(&u, &w);

		for (int i = 0; i < new_size; i++) {
			double vu = cos(2 * M_PI * i / new_size);
			double vw = sin(2 * M_PI * i / new_size);
			mthz::Vec3 target_dir = u * vu + w * vw;

			ContactP max_p;
			int max_indx;
			double max_v = -std::numeric_limits<double>::infinity();
			for (int j = 0; j < m.points.size(); j++) {
				if (p_available[j]) {
					ContactP p = m.points[j];
					double val = p.pos.dot(target_dir);
					if (val > max_v) {
						max_v = val;
						max_p = p;
						max_indx = j;
					}
				}
			}
			out.points[i] = max_p;
			out.max_pen_depth = std::max<double>(out.max_pen_depth, max_p.pen_depth);
			p_available[max_indx] = false;
		}

		return out;
	}
}