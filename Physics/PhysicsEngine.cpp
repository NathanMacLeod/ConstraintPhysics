#include "PhysicsEngine.h"
#include "RigidBody.h"
#include <vector>

namespace phyz {

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

		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				printf("%f ", matrix->at(i)[j]);
			}
			printf("| %f\n", target->at(i));
		}

		printf("\nanswer:\n");

		std::vector<double> out = std::vector<double>(n);
		for (int i = 0; i < n; i++) {
			out[i] = target->at(piv_picks[i]) / matrix->at(piv_picks[i])[i];
			printf("%f ", out[i]);
		}

		return out;

	}

	//manifold should be genereated with norm away from a, (meaning A makes call to SAT)
	static void resolve_collision(RigidBody* a, RigidBody* b, const Manifold& manifold, double restitution) {
		Mat3 aI = (a->orientation.getRotMatrix() * a->tensor).inverse();
		Mat3 bI = (b->orientation.getRotMatrix() * b->tensor).inverse();

		int n = manifold.points.size();
		Vec3 norm = manifold.normal;

		std::vector<Vec3> v_as = std::vector<Vec3>(n);
		std::vector<Vec3> v_bs = std::vector<Vec3>(n);
		for (int i = 0; i < n; i++) {
			Vec3 p = manifold.points[i];
			Vec3 rA = p - a->com;
			Vec3 rB = p - b->com;

			v_as[i] = aI * (rA.cross(norm));
			v_bs[i] = bI * (rB.cross(norm));
		}

		std::vector<double> target = std::vector<double>(n);
		for (int i = 0; i < n; i++) {
			Vec3 p = manifold.points[i];
			Vec3 rA = p - a->com;
			Vec3 rB = p - b->com;

			Vec3 vPa = a->vel + a->ang_vel.cross(rA);
			Vec3 vPb = b->vel + b->ang_vel.cross(rB);
			target[i] = (1 + restitution) * (vPa - vPb).dot(norm);
		}

		std::vector<std::vector<double>> matrix = std::vector<std::vector<double>>(n);
		for (int i = 0; i < n; i++) {
			matrix[i] = std::vector<double>(n);

			Vec3 p = manifold.points[i];
			Vec3 rA = p - a->com;
			Vec3 rB = p - b->com;
			for (int j = 0; j < n; j++) {
				matrix[i][j] = 1 / a->mass + 1 / b->mass + v_as[j].cross(rA).dot(norm) + v_bs[j].cross(rB).dot(norm);
			}
		}



		std::vector<double> impulses = gj_solve(&matrix, &target);
		for (int i = 0; i < impulses.size(); i++) {
			a->applyImpulse(-impulses[i] * norm, manifold.points[i]);
			b->applyImpulse(impulses[i] * norm, manifold.points[i]);
		}
		int banana = 1 + 2;
	}

	void PhysicsEngine::foo() {
		/*std::vector<double> v1 = { 3, -1, 4 };
		std::vector<double> v2 = { 6, -1, 3 };
		std::vector<double> v3 = { 5, 5, -5 };

		std::vector<std::vector<double>> matrix = { v1, v2, v3 };
		std::vector<double> target = { -5, -9, 20 };

		std::vector<double> impulses = gj_solve(&matrix, &target);*/

		Quaternion q1 = Quaternion(3.141592 / 4.0, Vec3(0, 0, 1));
		Quaternion q2 = Quaternion(3.141592 / 4.0, Vec3(0, 1, 0));

		ConvexPoly c1 = getRect(0, 0, 0, 2.1, 2, 2);
		//c1.rotate(q1, Vec3(1, 1, 1));
		ConvexPoly c2 = getRect(2, -1, -1, 2, 4, 4);
		//c2.rotate(q2, Vec3(3, 1, 1));

		std::vector<ConvexPoly> geom1 = { c1 };
		RigidBody r1 = RigidBody(geom1, 1);
		//r1.vel = Vec3(1, 0, 0);
		r1.ang_vel = Vec3(0, 0, 1);

		std::vector<ConvexPoly> geom2 = { c2 };
		RigidBody r2 = RigidBody(geom2, 1);
		//r2.vel = Vec3(-1, 0, 0);
		//r2.ang_vel = Vec3(0, 0, -1);

		Manifold info = c1.SAT(c2, 5);

		resolve_collision(&r1, &r2, info, 0.5);
	}

}