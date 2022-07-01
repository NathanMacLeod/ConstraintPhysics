#pragma once
#include "RigidBody.h"
#include <thread>
#include <atomic>
#include <mutex>

namespace phyz {
	static const double M_PI = 3.14159265358979323846;
	static const double TOL_ANG = 1.5 * M_PI / 180;
	static const double COS_TOL = 1 - cos(TOL_ANG);
	static const double SIN_TOL = sin(TOL_ANG);
	const double CUTOFF_MAG = 0.00000001;

	void init_multithreaded_sat(int n_helper_threads = 4);

	struct Manifold {
		std::vector<mthz::Vec3> points;
		mthz::Vec3 normal;
		double pen_depth;
	};
	Manifold merge_manifold(const Manifold& m1, const Manifold& m2);
	Manifold cull_manifold(const Manifold& m, int new_size);

	Manifold SAT(const ConvexPoly& a, const RigidBody::GaussMap& ag, const ConvexPoly& b, const RigidBody::GaussMap& bg);

}