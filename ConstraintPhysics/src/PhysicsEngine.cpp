#include "PhysicsEngine.h"
#include "RigidBody.h"
#include <vector>
#include <algorithm>

//static const double COS_TOL = 0.00001523; //~1 degree
//static const double SIN_TOL = 0.0174; //~1 degree

namespace phyz {

	void PhysicsEngine::timeStep() {
		for (RigidBody* b : bodies) {
			if (!b->fixed) {
				b->vel += gravity * step_time;
				b->applyGyroAccel(step_time);
			}
		}
		
		//for (int i = 0; i < 5; i++) {

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
						Manifold man = SAT(b1->geometry[i], b1->gauss_maps[i], b2->geometry[j], b2->gauss_maps[j]);

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

					/*printf("n manifolds: %d, ", manifolds.size());
					int i = 0;
					for (const Manifold& m : manifolds) {
						printf("s%d.n_points: %d ", i++, m.points.size());
					}
					printf("final manifold size: %d, culled_size : %d\n", manifolds.size(), manifolds[max_indx].points.size(), man.points.size());*/

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
		//}

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
		const bool DEBUG_PRINT = false;
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

	bool PhysicsEngine::resolve_collision(RigidBody* a, RigidBody* b, Manifold manifold, double restitution) {

		if (!a->fixed && !b->fixed) {
			int a = 1;
		}

		//Need to cull points from manifold where the bodies are separating, not collding
		//both this and the rhs of linear system need relative velocity of points so doing both at once
		mthz::Vec3 norm = manifold.normal;
		//if (norm.dot(manifold.points[0] - a->com) < 0) {
		//	norm = -norm; //make sure norm is pointing away from body a
		//}
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
			//WHEN IM BACK FIGURE OUT HOW MANIFOLDS ARE HAVING 0 POINTS
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
				if (isnan(a->com.x)) {
					int a = 1;
				}
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
					double frict_coeff = 0.5;

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

}