#include "RigidBody.h"
#include "PhysicsEngine.h"
#include <limits>
#include <algorithm>
#include <cassert>


namespace phyz {

	static void calculateMassProperties(const ConvexUnionGeometry& geometry, mthz::Vec3* com, mthz::Mat3* tensor, double* mass, bool override_center_of_mass, mthz::Vec3 center_of_mass_override);

	RigidBody::RigidBody(const ConvexUnionGeometry& source_geometry, const mthz::Vec3& pos, const mthz::Quaternion& orientation, unsigned int id, bool overide_center_of_mass = false, mthz::Vec3 local_coords_com_override = mthz::Vec3(0, 0, 0))
		: geometry_type(CONVEX_UNION), geometry(source_geometry.getPolyhedra()), reference_geometry(source_geometry.getPolyhedra()), vel(0, 0, 0), ang_vel(0, 0, 0),
		psuedo_vel(0, 0, 0), psuedo_ang_vel(0, 0, 0), asleep(false), sleep_ready_counter(0), non_sleepy_tick_count(0), id(id)
	{
		movement_type = DYNAMIC;
		calculateMassProperties(source_geometry, &this->com, &this->reference_tensor, &this->mass, overide_center_of_mass, local_coords_com_override);
		reference_invTensor = reference_tensor.inverse();
		tensor = reference_tensor;
		invTensor = reference_invTensor;

		for (ConvexPrimitive& c : reference_geometry) {
			c.recomputeFromReference(*c.getGeometry(), mthz::Mat3::iden(), -com);
		}

		geometry_AABB = std::vector<AABB>(geometry.size());
		for (int i = 0; i < reference_geometry.size(); i++) {
			geometry_AABB[i] = geometry[i].gen_AABB();
		}
		aabb = AABB::combine(geometry_AABB);
		local_coord_origin = -com;
		origin_pkey = trackPoint(mthz::Vec3(0,0,0));
		setToPosition(pos);
		recievedWakingAction = false;
	}

	RigidBody::RigidBody(const StaticMeshGeometry& source_geometry, unsigned int id) 
		: geometry_type(STATIC_MESH), reference_mesh(source_geometry), mesh(source_geometry), vel(0, 0, 0), ang_vel(0, 0, 0), psuedo_vel(0, 0, 0), psuedo_ang_vel(0, 0, 0), 
		asleep(false), sleep_ready_counter(0), non_sleepy_tick_count(0), id(id)
	{
		movement_type = FIXED;
		mass = std::numeric_limits<double>::quiet_NaN();
		reference_tensor *= std::numeric_limits<double>::quiet_NaN();

		reference_invTensor = reference_tensor.inverse();
		tensor = reference_tensor;
		invTensor = reference_invTensor;

		reference_aabb = reference_mesh.genAABB();
		aabb = reference_aabb;
		local_coord_origin = -com;
		origin_pkey = trackPoint(mthz::Vec3(0, 0, 0));
		recievedWakingAction = false;
	}

	void RigidBody::applyImpulse(mthz::Vec3 impulse, mthz::Vec3 position) {
		assert(!isnan(impulse.mag()) && !isnan(position.mag()));

		mthz::Vec3 torque = (position - com).cross(impulse);
		vel += getInvMass() * impulse;
		ang_vel += getInvTensor() * torque;
		alertWakingAction();
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

	void RigidBody::setMass(double mass, bool adjust_inertia_tensor_proportionally) {
		if (adjust_inertia_tensor_proportionally) {
			double new_mass_ratio = mass / this->mass;
			reference_tensor *= new_mass_ratio;
			reference_invTensor /= new_mass_ratio;
			tensor *= new_mass_ratio;
			invTensor /= new_mass_ratio;
		}
		this->mass = mass;
	}

	void RigidBody::rotate(mthz::Quaternion q) {
		this->orientation = q * orientation;
		updateGeometry();
		alertWakingAction();
	}

	void RigidBody::translate(const mthz::Vec3& v) {
		com += v;
		updateGeometry();
		alertWakingAction();
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
		assert(!isnan(ang_vel.mag()));
		this->ang_vel = ang_vel; 
		alertWakingAction();
	}

	void RigidBody::setMovementType(MovementType type) { 
		assert(geometry_type != STATIC_MESH || type != DYNAMIC);

		this->movement_type = type; 
		if (type != FIXED) {
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
		return (movement_type == FIXED || movement_type == KINEMATIC)? std::numeric_limits<double>::infinity() : mass;
	}

	double RigidBody::getInvMass() {
		return (movement_type == FIXED || movement_type == KINEMATIC) ? 0 : 1.0 / mass;
	}

	mthz::Mat3 RigidBody::getTensor() {
		return (movement_type == FIXED || movement_type == KINEMATIC) ? tensor * std::numeric_limits<double>::infinity() : tensor;
	}

	mthz::Mat3 RigidBody::getInvTensor() {
		return (movement_type == FIXED || movement_type == KINEMATIC) ? mthz::Mat3::zero() : invTensor;
	}

	mthz::Vec3 RigidBody::getVel() { 
		return (movement_type == FIXED)? mthz::Vec3(0, 0, 0) : vel;
	}

	mthz::Vec3 RigidBody::getAngVel() { 
		return (movement_type == FIXED) ? mthz::Vec3(0, 0, 0) : ang_vel;
	}

	bool RigidBody::getAsleep() {
		return asleep && movement_type == DYNAMIC;
	}

	RayHitInfo RigidBody::checkRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) {
		RayQueryReturn closest_hit_info = { false };

		for (int i = 0; i < geometry.size(); i++) {
			//check ray actually hits the convex primitive AABB. if geometry.size == 1 then the convex primitive AABB == the rigid body AABB, which is redundant to check
			if (geometry.size() != 1 && !AABB::rayIntersectsAABB(geometry_AABB[i], ray_origin, ray_dir)) continue;

			RayQueryReturn hit_info = geometry[i].testRayIntersection(ray_origin, ray_dir);
			if (hit_info.did_hit && (!closest_hit_info.did_hit || hit_info.intersection_dist < closest_hit_info.intersection_dist)) {
				closest_hit_info = hit_info;
			}
		}
		
		return RayHitInfo{ closest_hit_info.did_hit, this, closest_hit_info.intersection_point, closest_hit_info.intersection_dist };
	}

	//implicit integration method from Erin Catto, https://www.gdcvault.com/play/1022196/Physics-for-Game-Programmers-Numerical
	void RigidBody::rotateWhileApplyingGyroAccel(float fElapsedTime, int n_itr, bool gyro_accel_disabled) {
		const int CUTOFF_MAG = 0.00000000001;
		const int NEWTON_STEPS = 2;

		if (ang_vel.magSqrd() == 0) {
			return;
		}

		if (gyro_accel_disabled) {
			//simple update to orientation
			mthz::Quaternion rot(ang_vel.mag() * fElapsedTime, ang_vel.normalize());
			orientation = rot * orientation;
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

		assert(!isnan(itr_ang_vel.mag()));

		orientation = itr_orientation;
		ang_vel = itr_ang_vel;
	}

	void RigidBody::updateGeometry() {

		orientation = orientation.normalize();
		mthz::Mat3 rot = orientation.getRotMatrix();
		mthz::Mat3 rot_conjugate = orientation.conjugate().getRotMatrix();
		tensor = rot * reference_tensor * rot_conjugate;
		invTensor = rot * reference_invTensor * rot_conjugate;
		
		if (geometry_type == CONVEX_UNION) {
			for (int i = 0; i < reference_geometry.size(); i++) {
				geometry[i].recomputeFromReference(*reference_geometry[i].getGeometry(), rot, com);
				geometry_AABB[i] = geometry[i].gen_AABB();
			}
			aabb = AABB::combine(geometry_AABB);
		}
		else if (geometry_type == STATIC_MESH) {
			if (movement_type == FIXED) {
				mesh.recomputeFromReference(reference_mesh, rot, com);
				aabb = mesh.genAABB();
			}
			else {
				//u, v, w, origin are the world's x, y, z, origin from perspective of local coordinates
				mthz::Vec3 u = rot_conjugate * mthz::Vec3(1, 0, 0);
				mthz::Vec3 v = rot_conjugate * mthz::Vec3(0, 1, 0);
				mthz::Vec3 w = rot_conjugate * mthz::Vec3(0, 0, 1);
				mthz::Vec3 origin = -rot_conjugate * com;

				aabb = AABB::conformNewBasis(reference_aabb, u, v, w, origin);
			}
		}
		
	}

	void RigidBody::translateNoGeomUpdate(mthz::Vec3 v) {
		com += v;
	}

	void RigidBody::rotateNoGeomUpdate(mthz::Quaternion q) {
		orientation = q * orientation;
	}

	void RigidBody::sleep() {
		assert(movement_type == DYNAMIC);
		if (movement_type != DYNAMIC) return;

		asleep = true;
		vel = mthz::Vec3(0, 0, 0);
		ang_vel = mthz::Vec3(0, 0, 0);
	}

	void RigidBody::wake() {
		if (asleep) {
			asleep = false;
			sleep_ready_counter = 0;
			non_sleepy_tick_count = 0;
			history.clear();
		}
	}

	void RigidBody::alertWakingAction() {
		recievedWakingAction = true;
		sleep_ready_counter = 0;
		non_sleepy_tick_count = 0;
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

	static mthz::Mat3 recenterTensor(double mass, const mthz::Mat3& tensor, mthz::Vec3 new_center_of_rotation, mthz::Vec3 old_center_of_rotation = mthz::Vec3(0, 0, 0)) {
		mthz::Mat3 out = tensor;
		
		//transposition of inertia tensor to be relative to new point
		mthz::Vec3 d = new_center_of_rotation - old_center_of_rotation;
		mthz::Vec3 com = new_center_of_rotation;
		out.v[0][0] += mass * (d.y * d.y + d.z * d.z - 2 * d.y * com.y - 2 * d.z * com.z);
		out.v[0][1] += mass * (d.x * com.y + d.y * com.x - d.x * d.y);
		out.v[0][2] += mass * (d.x * com.z + d.z * com.x - d.x * d.z);
		out.v[1][1] += mass * (d.x * d.x + d.z * d.z - 2 * d.x * com.x - 2 * d.z * com.z);
		out.v[1][2] += mass * (d.y * com.z + d.z * com.y - d.y * d.z);
		out.v[2][2] += mass * (d.x * d.x + d.y * d.y - 2 * d.x * com.x - 2 * d.y * com.y);

		//intertia tensor is symmetric
		out.v[1][0] = out.v[0][1];
		out.v[2][0] = out.v[0][2];
		out.v[2][1] = out.v[1][2];

		return out;
	}

	//based off of 'Fast and Accurate Computation of Polyhedral Mass Properties' (Brian Miritch)
	static void calculateMassProperties(const ConvexUnionGeometry& geometry, mthz::Vec3* com, mthz::Mat3* tensor, double* mass, bool override_center_of_mass, mthz::Vec3 center_of_mass_override) {
		*com = override_center_of_mass? center_of_mass_override : mthz::Vec3(0, 0, 0);
		*mass = 0;
		*tensor = mthz::Mat3(); //default zeroed
		for (const ConvexPrimitive& primitive : geometry.getPolyhedra()) {

			switch (primitive.getType()) {
			case POLYHEDRON:
			{
				const Polyhedron& g = (const Polyhedron&)*primitive.getGeometry();
				double vol = 0, vol_x = 0, vol_y = 0, vol_z = 0, vol_xy = 0, vol_yz = 0, vol_zx = 0, vol_x2 = 0, vol_y2 = 0, vol_z2 = 0;

				for (const Surface& s : g.getSurfaces()) {
					mthz::Vec3 n = s.normal();
					
					IntrgVals a, b, c;
					Axis proj_axis;

					if (abs(n.z) > 0.01) {
						proj_axis = Z;
					}
					else {

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
					double surf_area = nc_i * g1;
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

				*mass += primitive.material.density * vol;
				if (!override_center_of_mass) *com += mthz::Vec3(vol_x, vol_y, vol_z) * primitive.material.density;

				tensor->v[0][0] += (vol_y2 + vol_z2) * primitive.material.density;
				tensor->v[0][1] -= vol_xy * primitive.material.density;
				tensor->v[0][2] -= vol_zx * primitive.material.density;
				tensor->v[1][1] += (vol_x2 + vol_z2) * primitive.material.density;
				tensor->v[1][2] -= vol_yz * primitive.material.density;
				tensor->v[2][2] += (vol_x2 + vol_y2) * primitive.material.density;
			}
			break;
			case SPHERE:
			{
				const Sphere& s = (const Sphere&)*primitive.getGeometry();
				double sphere_mass = (4.0 / 3.0) * PI * s.getRadius() * s.getRadius() * s.getRadius() * primitive.material.density;

				*mass += sphere_mass;
				if (!override_center_of_mass) *com += sphere_mass * s.getCenter();

				double k = 2.0 / 5.0 * sphere_mass * s.getRadius() * s.getRadius();
				mthz::Mat3 sphere_tensor = recenterTensor(sphere_mass, k * mthz::Mat3::iden(), mthz::Vec3(0, 0, 0), s.getCenter());

				*tensor += sphere_tensor;
			}
			case CYLINDER:
			{
				const Cylinder& c = (const Cylinder&)*primitive.getGeometry();
				double h = c.getHeight();
				double r = c.getRadius();
				double cylinder_mass = PI * r * r * h * primitive.material.density;

				*mass += cylinder_mass;
				if (!override_center_of_mass) *com += cylinder_mass * c.getCenter();
				mthz::Mat3 cylinder_tensor = mthz::Mat3::zero();
				cylinder_tensor.v[0][0] = cylinder_mass * (h * h / 12 + r * r / 4);
				cylinder_tensor.v[1][1] = cylinder_mass * r * r / 2;
				cylinder_tensor.v[2][2] = cylinder_tensor.v[0][0];

				double dp = c.getHeightAxis().dot(mthz::Vec3(0, 1, 0));
				//rotate tensor to cylinders orientation
				if (abs(dp) < 0.999999999) {
					double rotation_angle = acos(dp);
					mthz::Vec3 rotation_axis = mthz::Vec3(0, 1, 0).cross(c.getHeightAxis()).normalize();
					mthz::Quaternion q(rotation_angle, rotation_axis);

					mthz::Mat3 rot = q.getRotMatrix();
					mthz::Mat3 rot_invert = q.conjugate().getRotMatrix();
					cylinder_tensor = rot * cylinder_tensor * rot_invert;
				}

				cylinder_tensor = recenterTensor(cylinder_mass, cylinder_tensor, mthz::Vec3(0, 0, 0), c.getCenter());
				*tensor += cylinder_tensor;
			}
			break;
			}
			

		}

		if (!override_center_of_mass) *com /= *mass;
		*tensor = recenterTensor(*mass, *tensor, *com);
	}
}