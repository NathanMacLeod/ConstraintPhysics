#include "CollisionDetect.h"
#include "ConvexPoly.h"

namespace phyz {

	bool operator==(const MagicID& m1, const MagicID& m2) {
		return m1.bID == m2.bID && m1.cID == m2.cID;
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

	//kinda brute forcey might redo later
	static ContactArea findContactArea(const ConvexPoly& c, mthz::Vec3 n, mthz::Vec3 p, int p_ID, mthz::Vec3 u, mthz::Vec3 w) {

		for (int surface_index : c.getFaceIndicesAdjacentToPointI(p_ID)) {
			const Surface& s = c.getSurfaces()[surface_index];
			double cos_ang = s.normal().dot(n);
			if (1 - cos_ang <= COS_TOL) {
				return projectFace(s, u, w);
			}
		}

		for (int edge_index : c.getEdgeIndicesAdjacentToPointI(p_ID)) {
			const Edge& e = c.getEdges()[edge_index];
			mthz::Vec3 p1 = e.p1();
			mthz::Vec3 p2 = e.p2();
			double sin_ang = abs((e.p2() - e.p1()).normalize().dot(n));
			if ((e.p1() == p || e.p2() == p) && sin_ang <= SIN_TOL) {
				return projectEdge(e, n, p, u, w);
			}
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

	ExtremaInfo findExtrema(const ConvexPoly& c, mthz::Vec3 axis) {
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

	Manifold SAT(const ConvexPoly& a, const GaussMap& ag, const ConvexPoly& b, const GaussMap& bg) {
		Manifold out;
		double pen_depth;
		CheckNormResults min_pen = { -1, -1, mthz::Vec3(), std::numeric_limits<double>::infinity() };

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
				if ((a1 + a2).dot(b1 + b2) <= 0) {
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
				if ((a1 + a2).dot(n) < 0) {
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
						m |= 0x000000000000FFFF & a1_ID;
						m |= 0x00000000FFFF0000 & a2_ID;
						m |= 0x0000FFFF00000000 & (uint64_t(b1_ID) << 32);
						m |= 0xFFFF000000000000 & (uint64_t(b2_ID) << 48);
						man_pool_magics.push_back(m);
					}
				}
			}
		}

		double a_pen = min_pen.pen_depth;
		double a_dot_val = a_maxP.dot(norm);
		mthz::Vec3 n_offset = norm * a_dot_val;

		uint64_t cID = 0;
		cID |= 0x00000000FFFFFFFF & a.getID();
		cID |= 0xFFFFFFFF00000000 & (uint64_t(b.getID()) << 32);

		out.points.reserve(man_pool.size());
		for (int i = 0; i < man_pool.size(); i++) {
			ProjP p = man_pool[i];
			ContactP cp;
			cp.pos = u * p.u + w * p.w + n_offset;
			cp.pen_depth = cp.pos.dot(norm) - a_dot_val + a_pen;
			cp.restitution = std::max<double>(a.material.restitution, b.material.restitution);
			cp.kinetic_friction_coeff = (a.material.kinetic_friction_coeff + b.material.kinetic_friction_coeff) / 2.0;
			cp.static_friction_coeff = (a.material.static_friction_coeff + b.material.static_friction_coeff) / 2.0;
			cp.magicID = MagicID{ cID, man_pool_magics[i] };
			out.points.push_back(cp);
		}
		out.max_pen_depth = min_pen.pen_depth;

		return out;
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