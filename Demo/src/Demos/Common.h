#pragma once

#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

// ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~
// ~=~=~=~=Duration Based State Transitions (used in many tests)~=~=~=~=
// ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~

struct StateWithDuration {
	std::string name;
	float duration;
};

struct CurrentState {
	bool final_state_has_elapsed;
	StateWithDuration current;
	float time_into_current_state;
};

CurrentState get_current_state(const std::initializer_list<StateWithDuration>& states, float current_time);

// ~=~=~=~=~=~=~=~=~=~=~=~=~=~=
// ~=~=~=~=Scissor Lift~=~=~=~=
// ~=~=~=~=~=~=~=~=~=~=~=~=~=~=

struct ScissorLiftConstruct {
	phyz::RigidBody* top_platform;
	phyz::ConstraintID left_distance_constraint;
	phyz::ConstraintID right_distance_constraint;
	float min_height, max_height;
	float move_speed;

	enum MovementState { LOWERING, FIXED, RAISING };
	MovementState state;
};
ScissorLiftConstruct create_scissor_lift(phyz::PhysicsEngine* p, std::vector<PhysBod>* body_dest, mthz::Vec3 pos);
void set_scissor_lift_movement_input(phyz::PhysicsEngine* p, ScissorLiftConstruct* s, ScissorLiftConstruct::MovementState desired_state);


// ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
// ~=~=~=~=Circular Tower~=~=~=~=
// ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
std::vector<phyz::RigidBody*> createCircularTower(phyz::PhysicsEngine* p, std::vector<PhysBod>* body_dest, mthz::Vec3 block_dim, double radius, double n_blocks_per_layer, mthz::Vec3 pos, int n_layers);
