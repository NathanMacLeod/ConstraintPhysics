#include "CollisionDetect.h"

namespace phyz {

	static struct CheckNormResults {
		mthz::Vec3 a_maxP;
		mthz::Vec3 b_maxP;
		mthz::Vec3 norm;
		double pen_depth;

		inline bool seprAxisExists() { return pen_depth < 0; }
	};

	static struct ProjP {
		double u;
		double w;
	};

	//kinda brute forcey might redo later
	static std::vector<ProjP> findContactArea(const ConvexPoly& c, mthz::Vec3 n, mthz::Vec3 p, mthz::Vec3 u, mthz::Vec3 w, double* flatness_out=nullptr) {

		std::vector<ProjP> out;

		for (const Surface& s : c.surfaces) {
			double cos_ang = s.normal().dot(n);
			if (1 - cos_ang <= COS_TOL) {
				int n_points = s.n_points();
				out = std::vector<ProjP>(n_points);
				for (int i = 0; i < n_points; i++) {
					mthz::Vec3 v = s.getPointI(i);
					out[i] = ProjP{ v.dot(u), v.dot(w) };
				}

				if (flatness_out != nullptr) {
					*flatness_out = cos_ang;
				}
				return out;
			}
		}

		for (Edge e : c.edges) {
			mthz::Vec3 p1 = e.p1();
			mthz::Vec3 p2 = e.p2();
			double sin_ang = abs((e.p2() - e.p1()).normalize().dot(n));
			if ((e.p1() == p || e.p2() == p) && sin_ang <= SIN_TOL) {
				out = { ProjP{ e.p1().dot(u), e.p1().dot(w) }, ProjP{ e.p2().dot(u), e.p2().dot(w) } };

				if (flatness_out != nullptr) {
					*flatness_out = sqrt(1 - sin_ang*sin_ang);
				}
				return out;
			}
		}

		if (flatness_out != nullptr) {
			*flatness_out = 1.0;
		}
		return out = { ProjP{ p.dot(u), p.dot(w) } };
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

	static struct ExtremaInfo {
		ExtremaInfo() {
			min_val = std::numeric_limits<double>::infinity();
			max_val = -std::numeric_limits<double>::infinity();
		}

		mthz::Vec3 min_p;
		mthz::Vec3 max_p;
		double min_val;
		double max_val;
	};

	static ExtremaInfo findExtrema(const ConvexPoly& c, mthz::Vec3 axis) {
		ExtremaInfo extrema;

		for (mthz::Vec3 p : c.points) {
			double val = p.dot(axis);
			if (val < extrema.min_val) {
				extrema.min_p = p;
				extrema.min_val = val;
			}
			if (val > extrema.max_val) {
				extrema.max_p = p;
				extrema.max_val = val;
			}
		}

		return extrema;
	}

	static CheckNormResults sat_checknorm(const ConvexPoly& a, const ConvexPoly& b, mthz::Vec3 n) {
		ExtremaInfo e1 = findExtrema(a, n);
		ExtremaInfo e2 = findExtrema(b, n);

		return { e1.max_p, e2.min_p, n, e1.max_val - e2.min_val };
	}

	Manifold SAT(const ConvexPoly& a, const RigidBody::GaussMap& ag, const ConvexPoly& b, const RigidBody::GaussMap& bg) {
		Manifold out;
		CheckNormResults min_pen = { mthz::Vec3(), mthz::Vec3(), mthz::Vec3(), std::numeric_limits<double>::infinity() };

		for (mthz::Vec3 n : ag.face_verts) {
			CheckNormResults x = sat_checknorm(a, b, n);
			if (x.seprAxisExists()) {
				out.pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}
		for (mthz::Vec3 n : bg.face_verts) {
			CheckNormResults x = sat_checknorm(a, b, n);
			if (x.seprAxisExists()) {
				out.pen_depth = -1;
				return out;
			}
			else if (x.pen_depth < min_pen.pen_depth) {
				min_pen = x;
			}
		}
		for (RigidBody::GaussArc arc1 : ag.arcs) {
			for (RigidBody::GaussArc arc2 : bg.arcs) {

				mthz::Vec3 a1 = ag.face_verts[arc1.v1_indx];
				mthz::Vec3 a2 = ag.face_verts[arc1.v2_indx];
				mthz::Vec3 b1 = -bg.face_verts[arc2.v1_indx];
				mthz::Vec3 b2 = -bg.face_verts[arc2.v2_indx];

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

				CheckNormResults x = sat_checknorm(a, b, n);
				if (x.seprAxisExists()) {
					out.pen_depth = -1;
					return out;
				}
				else if (x.pen_depth < min_pen.pen_depth) {
					min_pen = x;
				}
			}
		}

		out.pen_depth = min_pen.pen_depth;
		out.normal = min_pen.norm;
		mthz::Vec3 norm = min_pen.norm;

		//generate manifold
		//make arbitrary perp vector
		const mthz::Vec3 axis1 = mthz::Vec3(1, 0, 0), axis2 = mthz::Vec3(0, 1, 0);
		mthz::Vec3 u = (abs(norm.dot(axis1)) < abs(norm.dot(axis2))) ? norm.cross(axis1).normalize() : norm.cross(axis2).normalize();
		mthz::Vec3 w = norm.cross(u);
		double a_flatness = -1, b_flatness = -1;
		std::vector<ProjP> a_contact = findContactArea(a, norm, min_pen.a_maxP, u, w, &a_flatness);
		std::vector<ProjP> b_contact = findContactArea(b, norm * (-1), min_pen.b_maxP, u, w, &b_flatness);
		out.flatness = std::min<double>(a_flatness, b_flatness);

		std::vector<ProjP> man_pool;
		for (ProjP p : a_contact) {
			if (pInside(b_contact, p)) {
				man_pool.push_back(p);
			}
		}
		for (ProjP p : b_contact) {
			if (pInside(a_contact, p)) {
				man_pool.push_back(p);
			}
		}
		if (a_contact.size() >= 2 && b_contact.size() >= 2) {
			int itr_max_a = (a_contact.size() == 2) ? 1 : a_contact.size();
			int itr_max_b = (b_contact.size() == 2) ? 1 : b_contact.size();
			for (int i = 0; i < itr_max_a; i++) {
				ProjP a1 = a_contact[i];
				ProjP a2 = a_contact[(i + 1) % a_contact.size()];

				for (int j = 0; j < itr_max_b; j++) {
					ProjP b1 = b_contact[j];
					ProjP b2 = b_contact[(j + 1) % b_contact.size()];

					ProjP out;
					if (findIntersection(a1, a2, b1, b2, &out)) {
						man_pool.push_back(out);
					}
				}
			}
		}

		mthz::Vec3 n_offset = norm * min_pen.a_maxP.dot(norm);

		for (ProjP p : man_pool) {
			out.points.push_back(u * p.u + w * p.w + n_offset);
		}

		return out;
	}

	Manifold merge_manifold(const Manifold& m1, const Manifold& m2) {
		Manifold out = { std::vector<mthz::Vec3>(m1.points.size() + m2.points.size()), mthz::Vec3(), -1, std::max<double>(m1.flatness, m2.flatness) };
		if (m1.pen_depth > m2.pen_depth) {
			out.normal = m1.normal;
			out.pen_depth = m1.pen_depth;
		}
		else {
			out.normal = m2.normal;
			out.pen_depth = m2.pen_depth;
		}

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
		Manifold out = { std::vector<mthz::Vec3>(new_size), m.normal, m.pen_depth, m.flatness };
		std::vector<bool> p_available(m.points.size(), true);

		const mthz::Vec3 axis1 = mthz::Vec3(1, 0, 0), axis2 = mthz::Vec3(0, 1, 0);
		mthz::Vec3 u = (abs(m.normal.dot(axis1)) < abs(m.normal.dot(axis2))) ? m.normal.cross(axis1).normalize() : m.normal.cross(axis2).normalize();
		mthz::Vec3 w = m.normal.cross(u);

		for (int i = 0; i < new_size; i++) {
			double vu = cos(2 * M_PI * i / new_size);
			double vw = sin(2 * M_PI * i / new_size);
			mthz::Vec3 target_dir = u * vu + w * vw;

			mthz::Vec3 max_p;
			int max_indx;
			double max_v = -std::numeric_limits<double>::infinity();
			for (int j = 0; j < m.points.size(); j++) {
				if (p_available[j]) {
					mthz::Vec3 p = m.points[j];
					double val = p.dot(target_dir);
					if (val > max_v) {
						max_v = val;
						max_p = p;
						max_indx = j;
					}
				}
			}
			out.points[i] = max_p;
			p_available[max_indx] = false;
		}

		return out;
	}
}