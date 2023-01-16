#include "RigidBody.h"
#include <limits>
#include <algorithm>


namespace phyz {

	static void calculateMassProperties(const Geometry& geometry, mthz::Vec3* com, mthz::Mat3* tensor, double* mass);

	RigidBody::RigidBody(const Geometry& source_geometry, const mthz::Vec3& pos, const mthz::Quaternion& orientation)
		: geometry(source_geometry.getPolyhedra()), reference_geometry(source_geometry.getPolyhedra()), vel(0, 0, 0), ang_vel(0, 0, 0),
		psuedo_vel(0, 0, 0), psuedo_ang_vel(0, 0, 0), asleep(false), sleep_ready_counter(0)
	{
		fixed = false;
		calculateMassProperties(source_geometry, &this->com, &this->reference_tensor, &this->mass);
		reference_invTensor = reference_tensor.inverse();
		tensor = reference_tensor;
		invTensor = reference_invTensor;

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

		for (const ConvexPoly& c : reference_geometry) {
			reference_gauss_maps.push_back(c.computeGaussMap());
		}
		geometry_AABB = std::vector<AABB>(geometry.size());
		for (int i = 0; i < reference_geometry.size(); i++) {
			geometry_AABB[i] = geometry[i].gen_AABB();
		}
		aabb = AABB::combine(geometry_AABB);
		gauss_maps = reference_gauss_maps;
		radius = sqrt(radius);
		local_coord_origin = -com;
		origin_pkey = trackPoint(mthz::Vec3(0,0,0));
		setToPosition(pos);
		recievedWakingAction = false;
	}

	void RigidBody::applyImpulse(mthz::Vec3 impulse, mthz::Vec3 position) {
		mthz::Vec3 torque = (position - com).cross(impulse);
		vel += getInvMass() * impulse;
		ang_vel += getInvTensor() * impulse;
	}

	void RigidBody::setToPosition(const mthz::Vec3& pos) {
		translate(pos - getTrackedP(origin_pkey));
		alertWakingAction();
	}

	void RigidBody::setCOMtoPosition(const mthz::Vec3& pos) {
		com = pos;
		updateGeometry();
		alertWakingAction();
	}

	void RigidBody::translate(const mthz::Vec3& v) {
		com += v;
		updateGeometry();
	}

	void RigidBody::setOrientation(const mthz::Quaternion orientation) {
		this->orientation = orientation;
		updateGeometry();
		alertWakingAction();
	}

	void RigidBody::setVel(mthz::Vec3 vel) { 
		this->vel = vel; 
		alertWakingAction();
	}

	void RigidBody::setAngVel(mthz::Vec3 ang_vel) { 
		this->ang_vel = ang_vel; 
		alertWakingAction();
	}

	void RigidBody::setFixed(bool fixed) { 
		this->fixed = fixed; 
		if (!fixed) {
			alertWakingAction();
		}
	}

	void RigidBody::setNoCollision(bool no_collision) {
		this->no_collision = no_collision;
		alertWakingAction();
	}

	RigidBody::PKey RigidBody::trackPoint(mthz::Vec3 p) {
		track_p.push_back(local_coord_origin + p);
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

	double RigidBody::getMass() {
		return (fixed)? std::numeric_limits<double>::infinity() : mass;
	}

	double RigidBody::getInvMass() {
		return (fixed) ? 0 : 1.0 / mass;
	}

	mthz::Vec3 RigidBody::getVel() { 
		return (fixed)? mthz::Vec3(0, 0, 0) : vel;
	}

	mthz::Vec3 RigidBody::getAngVel() { 
		return (fixed) ? mthz::Vec3(0, 0, 0) : ang_vel;
	}

	mthz::Mat3 RigidBody::getInvTensor() {
		return (fixed) ? mthz::Mat3::zero() : invTensor;
	}

	//implicit integration method from Erin Catto, https://www.gdcvault.com/play/1022196/Physics-for-Game-Programmers-Numerical
	void RigidBody::rotateWhileApplyingGyroAccel(float fElapsedTime, int n_itr) {
		const int CUTOFF_MAG = 0.00000000001;
		const int NEWTON_STEPS = 2;
		if (ang_vel.magSqrd() == 0) {
			return;
		}

		mthz::Mat3 itr_tensor = tensor;
		mthz::Quaternion itr_orientation = orientation;
		mthz::Vec3 itr_ang_vel = ang_vel;

		for (int i = 0; i < n_itr; i++) {

			mthz::Vec3 w = itr_ang_vel; //initial guess

			//n_itr is number of sub-timesteps
			float t_step = fElapsedTime / n_itr;
			//using newtons method to approximate result of implicit integration
			for (int i = 0; i < NEWTON_STEPS; i++) {
				mthz::Vec3 f = t_step * w.cross(itr_tensor * w) - itr_tensor * (itr_ang_vel - w);
				mthz::Mat3 jacobian = itr_tensor + t_step * (mthz::Mat3::cross_mat(w) * itr_tensor - mthz::Mat3::cross_mat(itr_tensor * w));
				if (abs(jacobian.det()) > CUTOFF_MAG) {
					w -= jacobian.inverse() * f;
				}
			}

			itr_ang_vel = w;
			itr_orientation = mthz::Quaternion(t_step * itr_ang_vel.mag(), itr_ang_vel) * itr_orientation;
			if (i + 1 < n_itr) {
				//update for next step
				mthz::Mat3 rot = itr_orientation.getRotMatrix();
				mthz::Mat3 rot_conjugate = itr_orientation.conjugate().getRotMatrix();
				itr_tensor = rot * reference_tensor * rot_conjugate;
			}
		}

		orientation = itr_orientation;
		ang_vel = itr_ang_vel;
	}

	void RigidBody::updateGeometry() {

		orientation = orientation.normalize();
		mthz::Mat3 rot = orientation.getRotMatrix();
		mthz::Mat3 rot_conjugate = orientation.conjugate().getRotMatrix();
		tensor = rot * reference_tensor * rot_conjugate;
		invTensor = rot * reference_invTensor * rot_conjugate;

		for (int i = 0; i < reference_geometry.size(); i++) {
			for (int j = 0; j < reference_geometry[i].points.size(); j++) {
				geometry[i].points[j] = com + rot * reference_geometry[i].points[j];
			}
			geometry[i].interior_point = com + rot * reference_geometry[i].interior_point;
		}
		for (int i = 0; i < reference_gauss_maps.size(); i++) {
			for (int j = 0; j < reference_gauss_maps[i].face_verts.size(); j++) {
				gauss_maps[i].face_verts[j].v = rot * reference_gauss_maps[i].face_verts[j].v;
			}
		}
		for (int i = 0; i < reference_geometry.size(); i++) {
			geometry_AABB[i] = geometry[i].gen_AABB();
		}
		aabb = AABB::combine(geometry_AABB);
		
	}

	void RigidBody::sleep() {
		asleep = true;
		vel = mthz::Vec3(0, 0, 0);
		ang_vel = mthz::Vec3(0, 0, 0);
	}

	void RigidBody::wake() {
		if (asleep) {
			asleep = false;
			sleep_ready_counter = 0;
			history.clear();
		}
	}

	void RigidBody::alertWakingAction() {
		recievedWakingAction = true;
		sleep_ready_counter = 0;
		history.clear();
	}

	void RigidBody::recordMovementState(int history_length) {
		history.push_back(MovementState{vel, ang_vel});
		while (history.size() > history_length) {
			history.erase(history.begin());
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
	static void calculateMassProperties(const Geometry& geometry, mthz::Vec3* com, mthz::Mat3* tensor, double* mass) {
		*com = mthz::Vec3(0, 0, 0);
		*mass = 0;
		*tensor = mthz::Mat3(); //default zeroed
		for (const ConvexPoly& g : geometry.getPolyhedra()) {
			double vol = 0, vol_x = 0, vol_y = 0, vol_z = 0, vol_xy = 0, vol_yz = 0, vol_zx = 0, vol_x2 = 0, vol_y2 = 0, vol_z2 = 0;

			for (const Surface& s : g.getSurfaces()) {
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
						+ (va * dA * dB * dB + vb * dB * dA * dA) / 2.0 + dA * dA * dB * dB / 5.0) / 2.0;
					gab2 += dB * (va * va * vb * vb + va * va * vb * dB + va * vb * vb * dA + (va * va * dB * dB + vb * vb * dA * dA + 4 * va * vb * dA * dB) / 3.0
						+ (va * dA * dB * dB + vb * dB * dA * dA) / 2.0 + dA * dA * dB * dB / 5.0) / 2.0;
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

			*mass += g.material.density * vol;
			*com += mthz::Vec3(vol_x, vol_y, vol_z) * g.material.density;

			tensor->v[0][0] += (vol_y2 + vol_z2) * g.material.density;
			tensor->v[0][1] -= vol_xy * g.material.density;
			tensor->v[0][2] -= vol_zx * g.material.density;
			tensor->v[1][1] += (vol_x2 + vol_z2) * g.material.density;
			tensor->v[1][2] -= vol_yz * g.material.density;
			tensor->v[2][2] += (vol_x2 + vol_y2) * g.material.density;
		}

		*com /= *mass;

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