#include "PhysicsEngine.h"
#include "RigidBody.h"
#include <vector>
#include <algorithm>

static const double M_PI = 3.14159265358979323846;
static const double TOL_ANG = 1.5 * M_PI / 180;
static const double COS_TOL = 1 - cos(TOL_ANG);
static const double SIN_TOL = sin(TOL_ANG);
//static const double FLATTENING_BIAS_MAG = 0.05;
const double CUTOFF_MAG = 0.00000001;

//static const double COS_TOL = 0.00001523; //~1 degree
//static const double SIN_TOL = 0.0174; //~1 degree

namespace phyz {

	Manifold merge_manifold(const Manifold& m1, const Manifold& m2) {
		Manifold out = { std::vector<mthz::Vec3>(m1.points.size() + m2.points.size()), mthz::Vec3(), -1 };
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
			out.points[i+off] = m2.points[i];
		}

		return out;
	}

	Manifold cull_manifold(const Manifold& m, int new_size) {
		if (new_size >= m.points.size()) {
			return m;
		}
		Manifold out = { std::vector<mthz::Vec3>(new_size), m.normal, m.pen_depth };
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

	void PhysicsEngine::timeStep() {
		for (RigidBody* b : bodies) {
			if (!b->fixed) {
				b->vel += gravity * step_time;
				b->applyGyroAccel(step_time);
			}
		}
		
		for (RigidBody* b1 : bodies) {
			if (b1->fixed) {
				continue;
			}
			for (RigidBody* b2 : bodies) {
				if (b1 == b2 || (b1->com - b2->com).mag() > b1->radius + b2->radius) {
					continue;
				}
				std::vector<Manifold> manifolds;
				for (int i = 0; i < b1->geometry.size(); i++) {
					for (int j = 0; j < b2->geometry.size(); j++) {
						Manifold man = gaussSAT(b1->geometry[i], b1->gauss_maps[i], b2->geometry[j], b2->gauss_maps[j]);

						if (man.pen_depth > 0) {
							manifolds.push_back(man);
						}
					}
				}
				if (manifolds.size() > 0) {

					int max_indx = 0;
					for (int i = 1; i < manifolds.size(); i++) {
						if (manifolds[i].pen_depth > manifolds[max_indx].pen_depth) {
							max_indx = i;
						}
					}
					for (int i = 0; i < manifolds.size(); i++) {
						if (i != max_indx) {
							double v = manifolds[max_indx].normal.dot(manifolds[i].normal);
							if (1 - v < COS_TOL) {
								manifolds[max_indx] = merge_manifold(manifolds[max_indx], manifolds[i]);
							}
						}
					}

					Manifold man = cull_manifold(manifolds[max_indx], 4);
					//printf("n manifolds: %d, final manifold size: %d, culled_size : %d\n", manifolds.size(), manifolds[max_indx].points.size(), man.points.size());
					
					if (man.points.size() == 4 && manifolds.size() == 2) {
						bool peenut = true;
					}

					bool collision_occured = resolve_collision(b1, b2, man, 0.33);
					if (collision_occured) {
						resolve_penetration(b1, b2, manifolds[max_indx], 0.25);
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

	static std::vector<double> gj_solve(std::vector<std::vector<double>>* matrix, std::vector<double>* target, bool positive_sol_bias=true) {
		const bool DEBUG_PRINT = true;
		int n = matrix->size();
		int n_free_var = 0;

		if (DEBUG_PRINT) {
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < n; j++) {
					printf(" % .5f ", matrix->at(i)[j]);
				}
				printf("| % .5f\n", target->at(i));
			}
			printf("\n");
		}

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

			if (abs(matrix->at(piv_pick)[i]) < CUTOFF_MAG) {
				n_free_var++;
				piv_picks[i] = -1; //signifies invalid
				continue; //variable probably zero, and divding by this will cause high innacuracy
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

		if (DEBUG_PRINT) {
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < n; j++) {
					printf("% .5f ", matrix->at(i)[j]);
				}
				printf("| % .5f\n", target->at(i));
			}

			printf("\nanswer:\n");
		}
		std::vector<double> out = std::vector<double>(n);

		//linear system with free variables have infinite solution, some of which involve negative impulses
		//even if a positive impulse only solution exists. This tries to bias towards positive only solutions, if possible
		if (positive_sol_bias && n_free_var == 1) {
			int free_col = -1;
			for (int i = 0; i < n; i++) {
				if (piv_picks[i] == -1) {
					free_col = i;
				}
			}
			
			//initial constraint from fv = fv
			double min_free_val = 0;
			double max_free_val = std::numeric_limits<double>::infinity();
			bool pos_sol_exist = true;
			for (int i = 0; i < n && pos_sol_exist; i++) {
				if (i == free_col) {
					continue;
				}

				double ki = matrix->at(piv_picks[i])[i];
				double kf = matrix->at(piv_picks[i])[free_col];
				double si = target->at(piv_picks[i]);

				if (abs(kf) > CUTOFF_MAG) {
					//find range where free var results in positive impulse
					if (-kf / ki > 0) {
						min_free_val = std::max<double>(min_free_val, si / kf);
					}
					else {
						max_free_val = std::min<double>(max_free_val, si / kf);
					}
				}
				else if (si / ki <= 0) {
					//impulse always negative independant of free_var
					pos_sol_exist = false;
				}
				else {
					//impulse always positive independant of free_var
				}
			}
			
			if (pos_sol_exist && max_free_val >= min_free_val) {
				double free_val = (min_free_val + max_free_val) / 2;
				//printf("FREE COL: %d, FREE VAL: %f\n", free_col, free_val);

				for (int i = 0; i < n; i++) {
					if (i == free_col) {
						out[i] = free_val;
					}
					else {
						out[i] = (target->at(piv_picks[i]) - matrix->at(piv_picks[i])[free_col]*free_val) / matrix->at(piv_picks[i])[i];
					}
					if (DEBUG_PRINT) {
						printf("%f ", out[i]);
					}
				}
				if (DEBUG_PRINT) {
					printf("\n\n");
				}

				return out;
			}
		}

		for (int i = 0; i < n; i++) {
			if (piv_picks[i] == -1 || abs(target->at(piv_picks[i])) < CUTOFF_MAG) { //counter innacuracy
				out[i] = 0;
			}
			else {
				out[i] = target->at(piv_picks[i]) / matrix->at(piv_picks[i])[i];
			}
			if (DEBUG_PRINT) {
				printf("%f ", out[i]);
			}
		}
		if (DEBUG_PRINT) {
			printf("\n\n");
		}

		return out;

	}



	//manifold should be genereated with norm away from a, (meaning A makes call to SAT)
	bool PhysicsEngine::resolve_collision(RigidBody* a, RigidBody* b, Manifold manifold, double restitution) {

		if (!a->fixed && !b->fixed) {
			int a = 1;
		}

		//Need to cull points from manifold where the bodies are separating, not collding
		//both this and the rhs of linear system need relative velocity of points so doing both at once
		mthz::Vec3 norm = manifold.normal;
		std::vector<double> target;
		for (int i = 0; i < manifold.points.size(); i++) {
			mthz::Vec3 p = manifold.points[i];

			mthz::Vec3 vPa = a->getVelOfPoint(p);
			mthz::Vec3 vPb = b->getVelOfPoint(p);
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
			return false; //bodies aren't colliding, nothing to do
		}

		mthz::Mat3 aI = (a->fixed) ? mthz::Mat3::zero() : a->orientation.getRotMatrix() * a->invTensor * a->orientation.conjugate().getRotMatrix();
		mthz::Mat3 bI = (b->fixed) ? mthz::Mat3::zero() : b->orientation.getRotMatrix() * b->invTensor * b->orientation.conjugate().getRotMatrix();
		double a_invM = (a->fixed) ? 0 : 1.0 / a->mass;
		double b_invM = (b->fixed) ? 0 : 1.0 / b->mass;

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
				mthz::Vec3 z = v_bs[j].cross(rB);
				matrix[i][j] = a_invM + b_invM + v_as[j].cross(rA).dot(norm) + v_bs[j].cross(rB).dot(norm);
			}
		}

		std::vector<double> impulses = gj_solve(&matrix, &target);
		//gj_solve can have solutions with negative impulses (suction)
		//need to make sure that isn't occuring, and re-solve with only pushing points if it is
		bool suction_exists = false;
		for (int i = 0; i < impulses.size(); i++) {
			if (impulses[i] < -CUTOFF_MAG) {
				suction_exists = true;
				break;
			}
		}

		if (suction_exists) {
			Manifold new_man = { std::vector<mthz::Vec3>(0), manifold.normal, manifold.pen_depth };
			for (int i = 0; i < manifold.points.size(); i++) {
				if (impulses[i] > -CUTOFF_MAG) {
					new_man.points.push_back(manifold.points[i]);
				}
			}
			return resolve_collision(a, b, new_man, restitution);
		}
		else {

			//apply flattening bias to encourage objects to sit more perfectly flat
			/*if (n >= 2) {
				std::vector<double> n_axis_depth(n);
				double avg_depth = 0.0;
				double min_depth = std::numeric_limits<double>::infinity();
				double max_depth = -std::numeric_limits<double>::infinity();
				for (int i = 0; i < n; i++) {
					double d = norm.dot(manifold.points[i]);
					min_depth = std::min<double>(min_depth, d);
					max_depth = std::max<double>(max_depth, d);
					n_axis_depth[i] = d;
					avg_depth += d;
				}
				avg_depth /= n;
				double depth_range = max_depth - min_depth;
				

				if (depth_range > CUTOFF_MAG) {
					for (int i = 0; i < n; i++) {
						double depth_diff = n_axis_depth[i] - avg_depth;
						impulses[i] *= 1 - FLATTENING_BIAS_MAG * depth_diff / (depth_range);
					}
				}
			}*/

			//apply bounce impulses
			for (int i = 0; i < impulses.size(); i++) {
				a->applyImpulse(-impulses[i] * norm, manifold.points[i]);
				b->applyImpulse(impulses[i] * norm, manifold.points[i]);
			}

			//calculate and apply friction impulses
			double total_impulse = 0;
			std::vector<mthz::Vec3> drift_dir(n);
			for (int i = 0; i < n; i++) {
				total_impulse += impulses[i];

				mthz::Vec3 p = manifold.points[i];
				mthz::Vec3 vel_rel = a->getVelOfPoint(p) - b->getVelOfPoint(p);
				drift_dir[i] = vel_rel - norm * norm.dot(vel_rel);
			}
			for (int i = 0; i < n; i++) {
				if (drift_dir[i].magSqrd() > CUTOFF_MAG) {
					double frict_coeff = 0;// 0.5;

					mthz::Vec3 p = manifold.points[i];
					mthz::Vec3 rA = p - a->com;
					mthz::Vec3 rB = p - b->com;


					mthz::Vec3 dd_norm = drift_dir[i].normalize();
					double frict_imp = 2 * (impulses[i] / total_impulse) * drift_dir[i].dot(dd_norm) / (a_invM + b_invM +
						(aI * (rA.cross(dd_norm))).cross(rA).dot(dd_norm) + (bI * (rB.cross(dd_norm))).cross(rB).dot(dd_norm));

					if (frict_imp > abs(impulses[i]) * frict_coeff) {
						frict_imp = abs(impulses[i]) * frict_coeff;
					}

					a->applyImpulse(-frict_imp * dd_norm, p);
					b->applyImpulse(frict_imp * dd_norm, p);
				}
			}

			return true;
		}
	}

	void PhysicsEngine::resolve_penetration(RigidBody* a, RigidBody* b, const Manifold& manifold, double slack) {
		double a_mr;
		double b_mr;
		if (a->fixed && b->fixed) {
			return;
		}
		else if (a->fixed) {
			a_mr = 1;
			b_mr = 0;
		}
		else if (b->fixed) {
			a_mr = 0;
			b_mr = 1;
		}
		else {
			a_mr = a->mass / (a->mass + b->mass);
			b_mr = 1 - a_mr;
		}

		if (!a->fixed) {
			a->com -= manifold.normal * slack * b_mr * manifold.pen_depth;
			a->updateGeometry();
		}
		if (!b->fixed) {
			b->com += manifold.normal * slack * a_mr * manifold.pen_depth;
			b->updateGeometry();
		}
		
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

	Manifold PhysicsEngine::gaussSAT(const ConvexPoly& a, const RigidBody::GaussMap& ag, const ConvexPoly& b, const RigidBody::GaussMap bg) const {
		Manifold out;
		bool sepr_axis = false; //assumed false at first
		bool first_check = true;
		mthz::Vec3 norm;
		mthz::Vec3 a_maxP, b_maxP;
		double pen_depth;

		for (mthz::Vec3 n : ag.face_verts) {
			sat_checknorm(a, b, n, &norm, &a_maxP, &b_maxP, &pen_depth, &sepr_axis, &first_check);
			if (sepr_axis) {
				out.pen_depth = -1;
				return out;
			}
		}

		for (mthz::Vec3 n : bg.face_verts) {
			sat_checknorm(a, b, n, &norm, &a_maxP, &b_maxP, &pen_depth, &sepr_axis, &first_check);
			if (sepr_axis) {
				out.pen_depth = -1;
				return out;
			}
		}

		for (RigidBody::GaussArc arc1 : ag.arcs) {
			for (RigidBody::GaussArc arc2 : bg.arcs) {

				mthz::Vec3 a1 = ag.face_verts[arc1.v1_indx];
				mthz::Vec3 a2 = ag.face_verts[arc1.v2_indx];
				mthz::Vec3 b1 = -bg.face_verts[arc2.v1_indx];
				mthz::Vec3 b2 = -bg.face_verts[arc2.v2_indx];

				//check arcs arent on opposite hemispheres
				if (((a1 + a2) / 2).dot((b1 + b2) / 2) <= 0) {
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
				if ((a1+a2).dot(n) < 0) {
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
		const mthz::Vec3 axis1 = mthz::Vec3(1, 0, 0), axis2 = mthz::Vec3(0, 1, 0);
		mthz::Vec3 u = (abs(norm.dot(axis1)) < abs(norm.dot(axis2))) ? norm.cross(axis1).normalize() : norm.cross(axis2).normalize();
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

		for (ProjP p : man_pool) {
			out.points.push_back(u * p.u + w * p.w + n_offset);
		}

		return out;
	}

	Manifold PhysicsEngine::SAT(const ConvexPoly& a, const ConvexPoly& b) const {
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
		const mthz::Vec3 axis1 = mthz::Vec3(1, 0, 0), axis2 = mthz::Vec3(0, 1, 0);
		mthz::Vec3 u = (abs(norm.dot(axis1)) < abs(norm.dot(axis2))) ? norm.cross(axis1).normalize() : norm.cross(axis2).normalize();
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


		/*if (man_pool.size() > max_man_size) {
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
		}*/
		//else {
		for (ProjP p : man_pool) {
			out.points.push_back(u * p.u + w * p.w + n_offset);
		}
		//}

		return out;
	}

}