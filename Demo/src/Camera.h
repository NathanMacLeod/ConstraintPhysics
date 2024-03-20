#pragma once
#include "../../Math/src/Vec3.h"
#include "../../Math/src/Quaternion.h"

class Camera {
public:
	inline void setRotateSpeed(double speed) { rotate_speed = speed; }
	inline void setDampenEnabled(bool enabled) { dampen_velocity = enabled; }
	inline void setTargetOrientation(mthz::Quaternion orientation) { target_orientation = orientation; }
	inline void setOrbitPosition(mthz::Vec3 pos) { orbit_position = pos; }
	inline void setOffsetFromOrbit(mthz::Vec3 offset) { orbit_offset = offset; } //offset is used to have the camera orbit a point while rotating, as opposed to rotating in place

	void setOrientation(mthz::Quaternion orientation) {
		current_orientation = orientation;
		target_orientation = orientation;
	}

	void update(double fElapsedTime) {

		double angle_off = current_orientation.angleTo(target_orientation);
		double effective_speed = rotate_speed;
		if (dampen_velocity && angle_off < dampen_angle) {
			effective_speed *= min_rotation_factor + (1 - min_rotation_factor) * angle_off / dampen_angle;
		}

		if (current_orientation == target_orientation) {}
		else if (angle_off <= effective_speed * fElapsedTime) {
			current_orientation = target_orientation;
		}
		else {
			mthz::Quaternion rot_quat = target_orientation * current_orientation.conjugate();
			mthz::Vec3 rotate_axis = rot_quat.getRotAxis();
			current_orientation = mthz::Quaternion(effective_speed * fElapsedTime, rotate_axis) * current_orientation;
		}
	}

	//0,0 is on the bottom left corner in screen coords
	struct ScreenPosQueryOut {
		bool behind_camera;
		double screen_x;
		double screen_y;
	};

	ScreenPosQueryOut getScreenPosition(mthz::Vec3 world_position, int screen_width, int screen_height) {
		mthz::Vec3 pos = current_orientation.conjugate().applyRotation(world_position - getPos());
		if (pos.z >= 0) return { true }; //behind the camera

		double k = -screen_height / (2 * pos.z * tan(fov * PI / 360)); //projection factor;
	
		return ScreenPosQueryOut{
			false,
			screen_width / 2.0 + pos.x * k,
			screen_height / 2.0 + pos.y * k
		};
	}

	inline mthz::Quaternion getTargetOrientation() const { return target_orientation; }
	inline mthz::Quaternion getOrientation() const { return current_orientation; }
	inline mthz::Vec3 getPos() const { return current_orientation.applyRotation(orbit_offset) + orbit_position; }
	inline double getFOV() const { return fov; }

private:
	mthz::Quaternion current_orientation;
	mthz::Quaternion target_orientation;
	double rotate_speed = 5;
	double dampen_angle = 1;
	double min_rotation_factor = 0.00;
	bool dampen_velocity = false;
	double fov = 60;

	mthz::Vec3 orbit_position;
	mthz::Vec3 orbit_offset;
};