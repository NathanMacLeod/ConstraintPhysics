#include "RigidBody.h"
#include <limits>
#include <algorithm>


namespace phyz {

	static void calculateMassProperties(const std::vector<ConvexPoly>& geometry, double density, mthz::Vec3* com, mthz::Mat3* tensor, double* mass);

	RigidBody::RigidBody(const std::vector<ConvexPoly>& geometry, double density, int id)
		: geometry(geometry), reference_geometry(geometry), vel(0, 0, 0), ang_vel(0, 0, 0)
	{
		this->id = id;
		fixed = false;
		calculateMassProperties(geometry, density, &this->com, &this->tensor, &this->mass);
		invTensor = tensor.inverse();

		radius = 0;
		for (ConvexPoly& c : reference_geometry) {
			c.interior_point -= com;
			for (mthz::Vec3& p : c.points) {
				p -= com;
				double r = p.magSqrd();
				if (r > radius) {
					radius = r;
				}
			}
		}
		radius = sqrt(radius);
	}

	RigidBody::PKey RigidBody::track_point(mthz::Vec3 p) {
		mthz::Vec3 r = p - com;
		track_p.push_back(orientation.conjugate().applyRotation(r));
		return track_p.size() - 1;
	}

	mthz::Vec3 RigidBody::getTrackedP(PKey pk) {
		if (pk < 0 || pk >= track_p.size()) {
			return mthz::Vec3(-1, -1, -1);
		}
		return com + orientation.applyRotation(track_p[pk]);
	}

	mthz::Vec3 RigidBody::getVelOfPoint(mthz::Vec3 p) const {
		return vel + ang_vel.cross(p - com);
	}

	void RigidBody::applyImpulse(mthz::Vec3 impulse, mthz::Vec3 position) {
		if (fixed) {
			return;
		}

		vel += impulse / mass;

		static mthz::Mat3 mat;

		ang_vel += invTensor * ((position - com).cross(impulse));

	}

	//implicit integration method from Erin Catto, https://www.gdcvault.com/play/1022196/Physics-for-Game-Programmers-Numerical
	void RigidBody::applyGyroAccel(float fElapsedTime, int n_substeps) {
		const int CUTOFF_MAG = 0.00000000001;
		const int NEWTON_STEPS = 1;
		if (ang_vel.magSqrd() == 0) {
			return;
		}

		mthz::Vec3 w_local = orientation.conjugate().applyRotation(ang_vel);
		mthz::Vec3 w_old = w_local;
		mthz::Vec3 w_new = w_local;

		//n_itr is number of sub-timesteps
		float t_step = fElapsedTime / n_substeps;
		for (int i = 0; i < n_substeps; i++) {
			w_new = w_old; //initial guess

			//using newtons method to approximate result of implicit integration
			for (int i = 0; i < NEWTON_STEPS; i++) {
				mthz::Vec3 f = t_step * w_new.cross(tensor * w_new) - tensor * (w_local - w_new);
				mthz::Mat3 jacobian = tensor + t_step * (mthz::Mat3::cross_mat(w_new) * tensor - mthz::Mat3::cross_mat(tensor * w_new));
				//printf("det: %f\n", jacobian.det());
				if (abs(jacobian.det()) > CUTOFF_MAG) {
					w_new = w_new - jacobian.inverse() * f;
				}
			}

			w_old = w_new; //for next iteration
		}
		

		ang_vel = orientation.applyRotation(w_new);
	}

	void RigidBody::updateGeometry() {
		mthz::Mat3 rot = orientation.getRotMatrix();
		for (int i = 0; i < reference_geometry.size(); i++) {
			for (int j = 0; j < reference_geometry[i].points.size(); j++) {
				geometry[i].points[j] = com + rot*reference_geometry[i].points[j];
			}
			geometry[i].interior_point = com + rot * reference_geometry[i].interior_point;
		}
	}

	static struct IntrgVals {
		IntrgVals() {
			v = 0;
			v2 = 0;
			v2t = 0;
			v3 = 0;
		}

		double v;
		double v2;
		double v2t;
		double v3;
	};

	static enum Axis { X, Y, Z };
	static enum MappedAxis { A, B, C };
	static double getAxisVal(mthz::Vec3 v, MappedAxis axis, Axis proj_axis) {
		switch (proj_axis) {
		case X:
			switch (axis) {
			case A:
				return v.y;
			case B:
				return v.z;
			case C:
				return v.x;
			}
		case Y:
			switch (axis) {
			case A:
				return v.z;
			case B:
				return v.x;
			case C:
				return v.y;
			}

		case Z:
			switch (axis) {
			case A:
				return v.x;
			case B:
				return v.y;
			case C:
				return v.z;
			}
		}
	}

	//based off of 'Fast and Accurate Computation of Polyhedral Mass Properties' (Brian Miritch)
	static void calculateMassProperties(const std::vector<ConvexPoly>& geometry, double density, mthz::Vec3* com, mthz::Mat3* tensor, double* mass) {
		*com = mthz::Vec3(0, 0, 0);
		*mass = 0;
		*tensor = mthz::Mat3(); //default zeroed
		for (const ConvexPoly& g : geometry) {
			double vol = 0, vol_x = 0, vol_y = 0, vol_z = 0, vol_xy = 0, vol_yz = 0, vol_zx = 0, vol_x2 = 0, vol_y2 = 0, vol_z2 = 0;

			for (const Surface& s : g.surfaces) {
				mthz::Vec3 n = s.normal();
				IntrgVals a, b, c;
				Axis proj_axis;

				//project onto most parralel plane
				if (abs(n.x) >= abs(n.y) && abs(n.x) >= abs(n.z)) {
					proj_axis = X;
				}
				else if (abs(n.y) >= abs(n.x) && abs(n.y) >= abs(n.z)) {
					proj_axis = Y;
				}
				else {
					proj_axis = Z;
				}

				//green's theorem
				double g1 = 0, ga = 0, gb = 0, gab = 0, ga2 = 0, gb2 = 0, ga2b = 0, gab2 = 0, ga3 = 0, gb3 = 0;
				for (int i = 0; i < s.n_points(); i++) {
					mthz::Vec3 v = s.getPointI(i);
					mthz::Vec3 v2 = s.getPointI((i + 1) % s.n_points());

					double va = getAxisVal(v, A, proj_axis);
					double vb = getAxisVal(v, B, proj_axis);
					double dA = getAxisVal(v2, A, proj_axis) - va;
					double dB = getAxisVal(v2, B, proj_axis) - vb;

					//o_o
					g1 += dB * (va + dA / 2.0);
					ga += dB * (dA * dA / 3.0 + va * dA + va * va) / 2.0;
					gb -= dA * (dB * dB / 3.0 + vb * dB + vb * vb) / 2.0;
					gab -= dA * (va * vb * vb + dA * vb * vb / 2.0 + va * vb * dB + (va * dB * dB + 2 * vb * dA * dB) / 3.0 + dA * dB * dB / 4.0) / 2.0;
					ga2 += dB * (va * va * va + 3 * va * va * dA / 2.0 + va * dA * dA + dA * dA * dA / 4.0) / 3.0;
					gb2 -= dA * (vb * vb * vb + 3 * vb * vb * dB / 2.0 + vb * dB * dB + dB * dB * dB / 4.0) / 3.0;
					ga2b -= dA * (va * va * vb * vb + va * va * vb * dB + va * vb * vb * dA + (va * va * dB * dB + vb * vb * dA * dA + 4 * va * vb * dA * dB) / 3.0
						+ (va * dA * dB * dB + vb * dB * dA * dA) / 2.0 + dA * dA * dB * dB / 4.0) / 2.0;
					gab2 += dB * (va * va * vb * vb + va * va * vb * dB + va * vb * vb * dA + (va * va * dB * dB + vb * vb * dA * dA + 4 * va * vb * dA * dB) / 3.0
						+ (va * dA * dB * dB + vb * dB * dA * dA) / 2.0 + dA * dA * dB * dB / 4.0) / 2.0;
					ga3 += dB * (va * va * va * va + 2 * va * va * va * dA + 2 * va * va * dA * dA + va * dA * dA * dA + dA * dA * dA * dA / 5.0) / 4.0;
					gb3 -= dA * (vb * vb * vb * vb + 2 * vb * vb * vb * dB + 2 * vb * vb * dB * dB + vb * dB * dB * dB + dB * dB * dB * dB / 5.0) / 4.0;
				}

				double na = getAxisVal(n, A, proj_axis);
				double nb = getAxisVal(n, B, proj_axis);
				double nc = getAxisVal(n, C, proj_axis);
				double nc_i = 1.0 / nc;

				mthz::Vec3 sample_point = s.getPointI(0);
				//constant from projecting surface integral onto the AB plane
				double k = nc * getAxisVal(sample_point, C, proj_axis) + nb * getAxisVal(sample_point, B, proj_axis) + na * getAxisVal(sample_point, A, proj_axis);

				//value propogation from projected surface integrals to the actual surface integrals
				a.v = nc_i * ga;
				b.v = nc_i * gb;
				c.v = nc_i * nc_i * (k * g1 - na * ga - nb * gb);
				a.v2 = nc_i * ga2;
				b.v2 = nc_i * gb2;
				c.v2 = nc_i * nc_i * nc_i * (k * k * g1 + na * na * ga2 + nb * nb * gb2 + 2 * (na * nb * gab - k * na * ga - k * nb * gb));
				a.v2t = nc_i * ga2b;
				b.v2t = nc_i * nc_i * (k * gb2 - na * gab2 - nb * gb3);
				c.v2t = nc_i * nc_i * nc_i * (k * k * ga + na * na * ga3 + nb * nb * gab2 + 2 * (na * nb * ga2b - k * na * ga2 - k * nb * gab));
				a.v3 = nc_i * ga3;
				b.v3 = nc_i * gb3;
				c.v3 = nc_i * nc_i * nc_i * nc_i * (k * k * k * g1 - na * na * na * ga3 - nb * nb * nb * gb3 - 3 * k * k * na * ga + 3 * k * na * na * ga2 - 3 * na * na * nb * ga2b - 3 * na * nb * nb * gab2
					+ 3 * k * nb * nb * gb2 - 3 * k * k * nb * gb + 6 * k * na * nb * gab);

				//map back from a,b,c to x,y,z
				IntrgVals x, y, z;
				switch (proj_axis) {
				case X:
					x = c;
					y = a;
					z = b;
					break;
				case Y:
					x = b;
					y = c;
					z = a;
					break;
				case Z:
					x = a;
					y = b;
					z = c;
					break;
				}
				//value propogation from surface integral to volume integral (divergence theorem)
				vol += n.x * x.v;
				vol_x += n.x * x.v2 / 2.0;
				vol_y += n.y * y.v2 / 2.0;
				vol_z += n.z * z.v2 / 2.0;
				vol_xy += n.x * x.v2t / 2.0;
				vol_yz += n.y * y.v2t / 2.0;
				vol_zx += n.z * z.v2t / 2.0;
				vol_x2 += n.x * x.v3 / 3.0;
				vol_y2 += n.y * y.v3 / 3.0;
				vol_z2 += n.z * z.v3 / 3.0;
			}

			*mass += density * vol;
			*com += mthz::Vec3(vol_x, vol_y, vol_z);

			tensor->v[0][0] += vol_y2 + vol_z2;
			tensor->v[0][1] -= vol_xy;
			tensor->v[0][2] -= vol_zx;
			tensor->v[1][1] += vol_x2 + vol_z2;
			tensor->v[1][2] -= vol_yz;
			tensor->v[2][2] += vol_x2 + vol_y2;
		}

		*com /= *mass / density; // volume = mass/density
		*tensor *= density;

		//transposition of inertia tensor to be relative to CoM
		tensor->v[0][0] -= *mass * (com->y * com->y + com->z * com->z);
		tensor->v[0][1] += *mass * com->x * com->y;
		tensor->v[0][2] += *mass * com->z * com->x;
		tensor->v[1][1] -= *mass * (com->x * com->x + com->z * com->z);
		tensor->v[1][2] += *mass * com->y * com->z;
		tensor->v[2][2] -= *mass * (com->x * com->x + com->y * com->y);

		//intertia tensor is symmetric
		tensor->v[1][0] = tensor->v[0][1];
		tensor->v[2][0] = tensor->v[0][2];
		tensor->v[2][1] = tensor->v[1][2];
	}

}