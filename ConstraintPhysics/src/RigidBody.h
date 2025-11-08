#pragma once

#include "../../Math/src/Vec3.h"
#include "../../Math/src/Quaternion.h"
#include "../../Math/src/Mat3.h"
#include "Geometry.h"
#include "ConvexPrimitive.h"
#include "AABB.h"
#include <Vector>
#include <set>

namespace phyz {
	struct RayHitInfo;

	class RigidBody {
	private:
		RigidBody(const ConvexUnionGeometry& source_geometry, const mthz::Vec3& pos, const mthz::Quaternion& orientation, unsigned int id, bool overide_center_of_mass, mthz::Vec3 local_coords_com_override);
		RigidBody(const StaticMeshGeometry& source_geometry, unsigned int id);
	public:
		typedef int PKey;
		enum GeometryType { CONVEX_UNION, STATIC_MESH };
		enum MovementType { DYNAMIC, FIXED, KINEMATIC };
		enum CenterOfMassType { CUSTOM, PHYSICALLY_BASED };

		PKey trackPoint(mthz::Vec3 p); //track the movement of p which is on the body b. P given in local coordinates
		mthz::Vec3 getTrackedP(PKey pk) const;
		mthz::Vec3 getExtrapolatedTrackedP(PKey pk) const;
		mthz::Vec3 getVelOfPoint(mthz::Vec3 p) const;
		inline unsigned int getID() const { return id; }

		double getMass() const;
		double getInvMass() const;
		mthz::Mat3 getTensor() const;
		mthz::Mat3 getInvTensor() const;
		bool getAsleep() const;
		inline MovementType getMovementType() const { return movement_type; }
		inline bool getNoCollision() const { return no_collision; }
		mthz::Vec3 getPos() const { return getTrackedP(origin_pkey); }
		mthz::Vec3 getExtrapolatedPos() const { return getExtrapolatedTrackedP(origin_pkey); }
		inline mthz::Vec3 getCOM() const { return com_type == PHYSICALLY_BASED? com : getTrackedP(custom_com_pos); }
		inline mthz::Vec3 getExtrapoltedCOM() const { return com_type == PHYSICALLY_BASED ? extrapolated_com : getExtrapolatedTrackedP(custom_com_pos); }
		inline mthz::Quaternion getOrientation() const { return orientation; }
		inline mthz::Quaternion getExtrapolatedOrientation() const { return extrapolated_orientation; }
		mthz::Vec3 getVel() const;
		mthz::Vec3 getAngVel() const;
		inline GeometryType getGeometryType() const { return geometry_type; }
		RayHitInfo checkRayIntersection(mthz::Vec3 ray_origin, mthz::Vec3 ray_dir) const;

		void applyForce(mthz::Vec3 force) { vel += force * getInvMass(); }
		void applyTorque(mthz::Vec3 torque) { ang_vel += getInvTensor() * torque; }
		void applyImpulse(mthz::Vec3 impulse, mthz::Vec3 position);
		void setToPosition(const mthz::Vec3& pos);
		void translateSoCOMAtPosition(const mthz::Vec3& pos);
		void setCOMType(CenterOfMassType com_type);
		void setCustomCOMLocalPosition(mthz::Vec3 new_local_com_pos);
		void setMass(double mass, bool adjust_inertia_tensor_proportionally = true);
		void rotate(mthz::Quaternion q);
		void translate(const mthz::Vec3& v);
		void setOrientation(const mthz::Quaternion orientation);
		void setVel(mthz::Vec3 vel);
		void setAngVel(mthz::Vec3 ang_vel);
		void setMovementType(MovementType type);
		void setNoCollision(bool no_collision);
		void setSleepDisabled(bool b) { sleep_disabled = b; }
		void translateExtrapolatedPos(mthz::Vec3 translation) { extrapolated_com += translation; }
		void rotateExtrapolatedOrientation(mthz::Quaternion rotation) { extrapolated_orientation = rotation * extrapolated_orientation; }

		friend class PhysicsEngine;
	private:
		AABB aabb;
		MovementType movement_type;
		mthz::Vec3 local_coord_origin;
		PKey origin_pkey;
		unsigned int id;

		mthz::Quaternion orientation;
		mthz::Quaternion extrapolated_orientation;
		CenterOfMassType com_type;
		PKey custom_com_pos;
		mthz::Vec3 com;
		mthz::Vec3 extrapolated_com;
		mthz::Vec3 vel;
		mthz::Vec3 ang_vel;
		mthz::Vec3 psuedo_vel;
		mthz::Vec3 psuedo_ang_vel;
		bool recievedWakingAction;
		bool sleep_disabled;

		mthz::Vec3 prev_com;
		mthz::Quaternion prev_orientation;

		void sleep();
		void wake();
		void alertWakingAction();

		void rotateWhileApplyingGyroAccel(double fElapsedTime, int n_itr = 1, bool gyro_accel_disabled=false);
		void updateGeometry();
		void translateNoGeomUpdate(mthz::Vec3 v);
		void rotateNoGeomUpdate(mthz::Quaternion q);

		struct MovementState {
			mthz::Vec3 vel;
			mthz::Vec3 ang_vel;
		};

		std::vector<MovementState> history;
		void recordMovementState(int history_length);

		std::set<RigidBody*> no_collision_set;

		mthz::Mat3 invTensor;
		mthz::Mat3 tensor;
		mthz::Mat3 reference_invTensor;
		mthz::Mat3 custom_com_referenceTensor;
		mthz::Mat3 custom_com_referenceInvTensor;
		mthz::Mat3 reference_tensor;
		double mass;
		bool asleep;
		bool no_collision;
		double sleep_ready_counter;
		int non_sleepy_tick_count;
		
		GeometryType geometry_type;

		//used for convex union
		std::vector<AABB> geometry_AABB;
		std::vector<ConvexPrimitive> geometry;
		std::vector<ConvexPrimitive> reference_geometry;

		//for static mesh
		StaticMeshGeometry reference_mesh;
		AABB reference_aabb;
		StaticMeshGeometry mesh;
		
		std::vector<mthz::Vec3> track_p;
	};

	class OrderedBodyPair {
	public:
		OrderedBodyPair(RigidBody* t1, RigidBody* t2) {
			//keep ordering consistent to avoid {x, y}; {y, x} duplicates
			if (t1->getID() < t2->getID()) {
				this->t1 = t1;
				this->t2 = t2;
			}
			else {
				this->t1 = t2;
				this->t2 = t1;
			}
		}

		RigidBody* t1 = nullptr;
		RigidBody* t2 = nullptr;

		//for use in std::set
		bool operator<(const OrderedBodyPair& p) const {
			if (t1->getID() < p.t1->getID()) {
				return true;
			}
			else if (t1->getID() > p.t1->getID()) {
				return false;
			}
			else {
				return t2->getID() < p.t2->getID();
			}
		}

		bool operator==(const OrderedBodyPair& p) const {
			return t1 == p.t1 && t2 == p.t2;
		}
	};
}