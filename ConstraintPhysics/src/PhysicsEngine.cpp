#include "PhysicsEngine.h"
#include "RigidBody.h"
#include <vector>

#define M_PI 3.14159265358979323846

namespace phyz {

	void PhysicsEngine::timeStep() {
		for (RigidBody* b : bodies) {
			if (!b->fixed) {
				b->vel += gravity * step_time;
			}
		}
		
		for (RigidBody* b1 : bodies) {
			if (!b1->fixed) {
				for (RigidBody* b2 : bodies) {
					if (b1 != b2 && (b1->com - b2->com).mag() < b1->radius + b2->radius) {
						for (const ConvexPoly& c1 : b1->geometry) {
							for (const ConvexPoly& c2 : b2->geometry) {
								Manifold man = SAT(c1, c2, 4);
								if (man.pen_depth > 0) {
									resolve_collision(b1, b2, man, 0.6);
								}
							}
						}
					}
				}
			}
		}

		for (RigidBody* b : bodies) {
			if (!b->fixed) {
				b->com += b->vel * step_time;
				if (b->ang_vel.magSqrd() != 0) {
					b->orientation = mthz::Quaternion(step_time * b->ang_vel.mag(), b->ang_vel) * b->orientation;
				}
				b->updateGeometry();
			}
		}
	}

	RigidBody* PhysicsEngine::createRigidBody(const std::vector<ConvexPoly>& geometry, bool fixed, double density) {
		RigidBody* r = new RigidBody(geometry, density, next_ID++);
		r->fixed = fixed;
		bodies.push_back(r);
		return r;
	}

	static std::vector<double> gj_solve(std::vector<std::vector<double>>* matrix, std::vector<double>* target) {
		int n = matrix->size();

		std::vector<int> piv_picks = std::vector<int>(n);
		std::vector<int> piv_choices = std::vector<int>(n);
		for (int i = 0; i < n; i++) {
			piv_choices[i] = i;
		}

		for (int i = 0; i < n; i++) {
			int piv_pick = piv_choices[0];
			int chosen_indx = 0;
			for (int j = 0; j < piv_choices.size(); j++) {
				if (piv_choices[j] != -1 && (piv_pick == -1 || abs(matrix->at(piv_choices[j])[i]) > abs(matrix->at(piv_pick)[i]))) {
					piv_pick = piv_choices[j];
					chosen_indx = j;
				}
			}

			piv_choices[chosen_indx] = -1;
			piv_picks[i] = piv_pick;

			for (int j = 0; j < n; j++) {
				if (j != piv_pick) {
					double ratio = matrix->at(j)[i] / matrix->at(piv_pick)[i];

					target->at(j) -= target->at(piv_pick) * ratio;
					for (int k = 0; k < n; k++) {
						matrix->at(j)[k] -= matrix->at(piv_pick)[k] * ratio;
					}
				}
			}
		}

		/*for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				printf("%f ", matrix->at(i)[j]);
			}
			printf("| %f\n", target->at(i));
		}

		printf("\nanswer:\n");*/

		std::vector<double> out = std::vector<double>(n);
		for (int i = 0; i < n; i++) {
			out[i] = target->at(piv_picks[i]) / matrix->at(piv_picks[i])[i];
			//printf("%f ", out[i]);
		}

		return out;

	}



	//manifold should be genereated with norm away from a, (meaning A makes call to SAT)
	void PhysicsEngine::resolve_collision(RigidBody* a, RigidBody* b, Manifold manifold, double restitution) {
		//Need to cull points from manifold where the bodies are separating, not collding
		//both this and the rhs of linear system need relative velocity of points so doing both at once
		mthz::Vec3 norm = manifold.normal;
		std::vector<double> target;
		for (int i = 0; i < manifold.points.size(); i++) {
			mthz::Vec3 p = manifold.points[i];
			mthz::Vec3 rA = p - a->com;
			mthz::Vec3 rB = p - b->com;

			mthz::Vec3 vPa = a->vel + a->ang_vel.cross(rA);
			mthz::Vec3 vPb = b->vel + b->ang_vel.cross(rB);
			double rel_vel = (vPa - vPb).dot(norm);
			if (rel_vel > 0) {
				target.push_back((1 + restitution)*rel_vel);
			}
			else {
				manifold.points.erase(manifold.points.begin() + i);
				i--;
			}
		}

		int n = manifold.points.size();
		if (n == 0) {
			return; //bodies aren't colliding, nothing to do
		}

		mthz::Mat3 aI = (a->fixed) ? mthz::Mat3::zero() : a->orientation.getRotMatrix() * (a->tensor.inverse()) * a->orientation.conjugate().getRotMatrix();
		mthz::Mat3 bI = (b->fixed) ? mthz::Mat3::zero() : b->orientation.getRotMatrix() * (b->tensor.inverse()) * b->orientation.conjugate().getRotMatrix();
		double a_invM = (a->fixed) ? 0 : a->mass;
		double b_invM = (b->fixed) ? 0 : b->mass;

		std::vector<mthz::Vec3> v_as = std::vector<mthz::Vec3>(n);
		std::vector<mthz::Vec3> v_bs = std::vector<mthz::Vec3>(n);
		for (int i = 0; i < n; i++) {
			mthz::Vec3 p = manifold.points[i];
			mthz::Vec3 rA = p - a->com;
			mthz::Vec3 rB = p - b->com;

			v_as[i] = aI * (rA.cross(norm));
			v_bs[i] = bI * (rB.cross(norm));
		}

		std::vector<std::vector<double>> matrix = std::vector<std::vector<double>>(n);
		for (int i = 0; i < n; i++) {
			matrix[i] = std::vector<double>(n);

			mthz::Vec3 p = manifold.points[i];
			mthz::Vec3 rA = p - a->com;
			mthz::Vec3 rB = p - b->com;
			for (int j = 0; j < n; j++) {
				mthz::Vec3 pineapple = v_as[j].cross(rA);
				mthz::Vec3 z = v_bs[j].cross(rB);
				matrix[i][j] = a_invM + b_invM + v_as[j].cross(rA).dot(norm) + v_bs[j].cross(rB).dot(norm);
			}
		}


		std::vector<double> impulses = gj_solve(&matrix, &target);
		for (int i = 0; i < impulses.size(); i++) {
			a->applyImpulse(-impulses[i] * norm, manifold.points[i]);
			b->applyImpulse(impulses[i] * norm, manifold.points[i]);
		}
		int banana = 1 + 2;
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

	static struct ProjP {
		double u;
		double w;
	};

	//kinda brute forcey might redo later
	static std::vector<ProjP> findContactArea(const ConvexPoly& c, mthz::Vec3 n, mthz::Vec3 p, mthz::Vec3 u, mthz::Vec3 w) {
		const int COS_TOL = 0.00015; //~1 degree
		const int SIN_TOL = 0.015; //~1 degree

		std::vector<ProjP> out;

		for (const Surface& s : c.surfaces) {
			if (1 - s.normal().dot(n) <= COS_TOL) {
				int n_points = s.n_points();
				out = std::vector<ProjP>(n_points);
				for (int i = 0; i < n_points; i++) {
					mthz::Vec3 v = s.getPointI(i);
					out[i] = ProjP{ v.dot(u), v.dot(w) };
				}
				return out;
			}
		}

		for (Edge e : c.edges) {
			mthz::Vec3 p1 = e.p1();
			mthz::Vec3 p2 = e.p2();
			double d = abs((e.p2() - e.p1()).normalize().dot(n));
			if ((e.p1() == p || e.p2() == p) && abs((e.p2() - e.p1()).normalize().dot(n)) <= SIN_TOL) {
				out = { ProjP{ e.p1().dot(u), e.p1().dot(w) }, ProjP{ e.p2().dot(u), e.p2().dot(w) } };
				return out;
			}
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

	static void sat_checknorm(const ConvexPoly& a, const ConvexPoly& b, mthz::Vec3 n, mthz::Vec3* norm_out, mthz::Vec3* a_maxP, mthz::Vec3* b_maxP, double* pen_depth, bool* sepr_axis, bool* first_check) {
		ExtremaInfo e1 = findExtrema(a, n);
		ExtremaInfo e2 = findExtrema(b, n);

		double pen = e1.max_val - e2.min_val;
		if (pen < 0) {
			*sepr_axis = true;
			return;
		}

		if (*first_check || pen < *pen_depth) {
			*first_check = false;
			*a_maxP = e1.max_p;
			*b_maxP = e2.min_p;
			*pen_depth = pen;
			*norm_out = n;
		}
	}

	Manifold PhysicsEngine::SAT(const ConvexPoly& a, const ConvexPoly& b, int max_man_size) const {
		Manifold out;
		bool sepr_axis = false; //assumed false at first
		bool first_check = true;
		mthz::Vec3 norm;
		mthz::Vec3 a_maxP, b_maxP;
		double pen_depth;

		for (const Surface& s : a.surfaces) {
			mthz::Vec3 n = s.normal();
			sat_checknorm(a, b, n, &norm, &a_maxP, &b_maxP, &pen_depth, &sepr_axis, &first_check);
			if (sepr_axis) {
				out.pen_depth = -1;
				return out;
			}
		}

		for (const Surface& s : b.surfaces) {
			mthz::Vec3 n = s.normal();
			sat_checknorm(a, b, n, &norm, &a_maxP, &b_maxP, &pen_depth, &sepr_axis, &first_check);
			if (sepr_axis) {
				out.pen_depth = -1;
				return out;
			}
		}

		for (const Edge e1 : a.edges) {
			for (const Edge e2 : b.edges) {

				mthz::Vec3 v1 = e1.p2() - e1.p1();
				mthz::Vec3 v2 = e2.p2() - e2.p1();

				mthz::Vec3 n = v1.cross(v2);
				if (n.magSqrd() == 0) {
					continue;
				}
				n = n.normalize();
				if ((e1.p1() - a.interior_point).dot(n) < 0) {
					n *= -1;
				}

				sat_checknorm(a, b, n, &norm, &a_maxP, &b_maxP, &pen_depth, &sepr_axis, &first_check);
				if (sepr_axis) {
					out.pen_depth = -1;
					return out;
				}
			}
		}

		out.pen_depth = pen_depth;
		out.normal = norm;

		//generate manifold
		//make arbitrary perp vector
		const mthz::Vec3 temp1 = mthz::Vec3(1, 0, 0), temp2 = mthz::Vec3(0, 1, 0);
		mthz::Vec3 u = (abs(norm.dot(temp1)) < abs(norm.dot(temp2))) ? norm.cross(temp1).normalize() : norm.cross(temp2).normalize();
		mthz::Vec3 w = norm.cross(u);
		std::vector<ProjP> a_contact = findContactArea(a, norm, a_maxP, u, w);
		std::vector<ProjP> b_contact = findContactArea(b, norm * (-1), b_maxP, u, w);

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

		mthz::Vec3 n_offset = norm * a_maxP.dot(norm);


		if (man_pool.size() > max_man_size) {
			for (int i = 0; i < max_man_size; i++) {
				double vu = cos(2 * M_PI * i / max_man_size);
				double vw = sin(2 * M_PI * i / max_man_size);

				ProjP max;
				int max_i;
				double max_v = -std::numeric_limits<double>::infinity();
				for (int j = 0; j < man_pool.size(); j++) {
					ProjP p = man_pool[j];
					double val = vu * p.u + vw * p.w;
					if (val > max_v) {
						max_v = val;
						max = p;
						max_i = j;
					}
				}
				out.points.push_back(u * max.u + w * max.w + n_offset);
				man_pool.erase(man_pool.begin() + max_i); //delete to prevent same point from being selected twice
			}
		}
		else {
			for (ProjP p : man_pool) {
				out.points.push_back(u * p.u + w * p.w + n_offset);
			}

		}

		return out;
	}

}