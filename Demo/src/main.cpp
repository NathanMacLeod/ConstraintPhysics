
#define GLEW_STATIC
#define USE_MULTITHREAD
#include "renderer/Renderer.h"
#include "renderer/Mat4.h"
#include "../../Math/src/Vec3.h"
#include "../../Math/src/Quaternion.h"
#include "../../ConstraintPhysics/src/PhysicsEngine.h"
#include "../../ConstraintPhysics/src/Rigidbody.h"
#include "../../ConstraintPhysics/src/ConvexPoly.h"
#include "../../ConstraintPhysics/src/HACD.h"
#include <cmath>
#include <chrono>

struct Vertex {
	float x;
	float y;
	float z;
	float r;
	float g;
	float b;
};

static struct Mesh {
	rndr::VertexArray* va;
	rndr::IndexBuffer* ib;
};

float frand() {
	return (float)std::rand() / RAND_MAX;
}

Mesh fromPhyzMesh(const phyz::Mesh& m) {
	std::vector<Vertex> vertices;
	vertices.reserve(m.vertices.size());
	for (mthz::Vec3 v : m.vertices) {
		vertices.push_back(Vertex{ (float)v.x, (float)v.y, (float)v.z, frand(), frand(), frand()});
	}

	std::vector<unsigned int> indices;
	for (const std::vector<unsigned int>& face_indices : m.face_indices) {
		//naive- only works if all faces are convex
		for (int i = 2; i < face_indices.size(); i++) {
			indices.push_back(face_indices[0]);
			indices.push_back(face_indices[i-1]); 
			indices.push_back(face_indices[i]);
		}
	}

	rndr::VertexArrayLayout layout;
	layout.Push<float>(3);
	layout.Push<float>(3);
	return { new rndr::VertexArray(vertices.data(), vertices.size() * sizeof(Vertex), layout), new rndr::IndexBuffer(indices.data(), indices.size())};
}

Mesh fromGeometry(const phyz::Geometry g) {
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	int vertex_offset = 0;

	for (const phyz::ConvexPoly& c : g.getPolyhedra()) {
		for (mthz::Vec3 v : c.getPoints()) {
			vertices.push_back(Vertex{ (float)v.x, (float)v.y, (float)v.z, frand(), frand(), frand() });
		}

		for (const phyz::Surface& s : c.getSurfaces()) {
			//naive- only works if all faces are convex
			for (int i = 2; i < s.n_points(); i++) {
				indices.push_back(s.point_indexes[0] + vertex_offset);
				indices.push_back(s.point_indexes[i - 1] + vertex_offset);
				indices.push_back(s.point_indexes[i] + vertex_offset);
			}
		}

		vertex_offset += c.getPoints().size();
	}

	rndr::VertexArrayLayout layout;
	layout.Push<float>(3);
	layout.Push<float>(3);
	return { new rndr::VertexArray(vertices.data(), vertices.size() * sizeof(Vertex), layout), new rndr::IndexBuffer(indices.data(), indices.size()) };
}

static Mesh rect(float x, float y, float z, float w, float h, float d) {
	std::vector<Vertex> verticies = {
		{ x - w/2, y - h/2, z + d/2, 1.0f, 0.0f, 0.0f}, //0
		{ x + w/2, y - h/2, z + d/2, 1.0f, 1.0f, 0.0f}, //1
		{ x + w/2, y + h/2, z + d/2, 1.0f, 0.0f, 1.0f}, //2
		{ x - w/2, y + h/2, z + d/2, 1.0f, 1.0f, 1.0f}, //3
		{ x - w/2, y - h/2, z - d/2, 0.0f, 0.0f, 0.0f}, //4
		{ x + w/2, y - h/2, z - d/2, 0.0f, 0.0f, 1.0f}, //5
		{ x + w/2, y + h/2, z - d/2, 1.0f, 0.0f, 1.0f}, //6
		{ x - w/2, y + h/2, z - d/2, 0.0f, 1.0f, 1.0f}, //7
	};

	unsigned int indices[] = {
		0, 1, 2,
		2, 3, 0,

		1, 5, 6,
		6, 2, 1,

		5, 4, 7,
		7, 6, 5,

		4, 0, 3,
		3, 7, 4,

		2, 6, 7,
		7, 3, 2,

		0, 4, 5,
		5, 1, 0,
	};

	rndr::VertexArrayLayout layout;
	layout.Push<float>(3);
	layout.Push<float>(3);
	return { new rndr::VertexArray(verticies.data(), verticies.size() * sizeof(Vertex), layout), new rndr::IndexBuffer(indices, 36) };
}

static Mesh tetra(mthz::Vec3 p1, mthz::Vec3 p2, mthz::Vec3 p3, mthz::Vec3 p4) {

	std::vector<Vertex> verticies = {
		Vertex{ (float)p1.x, (float)p1.y, (float)p1.z, 1.0f, 0.0f, 0.0f}, //0
		Vertex{ (float)p2.x, (float)p2.y, (float)p2.z, 1.0f, 1.0f, 0.0f}, //1
		Vertex{ (float)p3.x, (float)p3.y, (float)p3.z, 1.0f, 0.0f, 1.0f}, //2
		Vertex{ (float)p4.x, (float)p4.y, (float)p4.z, 1.0f, 1.0f, 1.0f}, //3
	};

	unsigned int indices[] = {
		0, 1, 2,
		0, 3, 1,
		0, 2, 3,
		2, 1, 3,
	};

	rndr::VertexArrayLayout layout;
	layout.Push<float>(3);
	layout.Push<float>(3);
	return { new rndr::VertexArray(verticies.data(), verticies.size() * sizeof(Vertex), layout), new rndr::IndexBuffer(indices, 12) };
}

static Mesh ramp(float l, float h, float w) {

	std::vector<Vertex> verticies = {
		{ 0, 0, 0, 1.0f, 0.0f, 0.0f },
		{ l, 0, 0, 1.0f, 1.0f, 0.0f },
		{ 0, h, 0, 1.0f, 0.0f, 1.0f },
		{ 0, 0, w, 1.0f, 1.0f, 0.0f },
		{ l, 0, w, 1.0f, 0.0f, 1.0f },
		{ 0, h, w, 1.0f, 0.0f, 0.0f },
	};

	unsigned int indices[] = {
		0, 2, 1,
		3, 4, 5,
		0, 1, 4,
		4, 3, 0,
		1, 2, 5,
		5, 4, 1,
		2, 0, 3,
		3, 5, 2,
	};

	rndr::VertexArrayLayout layout;
	layout.Push<float>(3);
	layout.Push<float>(3);
	return { new rndr::VertexArray(verticies.data(), verticies.size() * sizeof(Vertex), layout), new rndr::IndexBuffer(indices, 24) };
}

struct PhysBod {
	std::vector<Mesh> meshes;
	phyz::RigidBody* r;
};

phyz::RigidBody* box(std::vector<PhysBod>* bodies, phyz::PhysicsEngine* p, mthz::Vec3 pos, mthz::Vec3 dim) {
	//std::vector<Mesh> m1 = { rect(0, 0, 0, dim.x, dim.y, dim.z) };
	phyz::Geometry geom1 = phyz::Geometry::box(-dim / 2.0, dim.x, dim.y, dim.z);
	std::vector<Mesh> m1 = { fromGeometry(geom1) };
	phyz::RigidBody* r1 = p->createRigidBody(geom1, false, pos);
	bodies->push_back({ m1, r1 });
	return r1;
}

phyz::Mesh bunny_mesh = phyz::readOBJ("resources/mesh/bunny.obj", 15.0);
phyz::Mesh teapot_mesh = phyz::readOBJ("resources/mesh/teapot.obj", 0.5);
phyz::Mesh cow_mesh = phyz::readOBJ("resources/mesh/cow.obj", 0.5);

int main() {

	phyz::PhysicsEngine p;

	if (false) {
		double timestep = 1 / 120.0;
		p.setStep_time(timestep);
		p.setGravity(mthz::Vec3(0, -2.8, 0));

		phyz::Geometry plane = phyz::Geometry::box(mthz::Vec3(0, 0, 0), 1, 1, 1);
		phyz::Geometry cube = phyz::Geometry::box(mthz::Vec3(-500, 0, -500), 1000, 1, 1000);

		for (int i = 0; i < 1400; i++) {
			p.createRigidBody(cube, false, mthz::Vec3(0, i * 2, 0));
		}
		p.createRigidBody(plane, true);

		double total_time = 0;

		for (int i = 1;; i++) {
			auto t1 = std::chrono::high_resolution_clock::now();
			p.timeStep();
			auto t2 = std::chrono::high_resolution_clock::now();
			total_time += std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

			if (i % 1000 == 0) {
				printf("Average tick time: %f\n", total_time / i);
			}
		}
	}

	if (true) {
		rndr::init(960, 960, "Hello World");

		std::vector<PhysBod> bodies;


		double s = 400;
		std::vector<Mesh> m2 = { rect(0, 0, 0, s, 2, s) };
		phyz::Geometry geom2= phyz::Geometry::box(mthz::Vec3(-s / 2, -1, -s / 2), s, 2, s);
		phyz::RigidBody* r2 = p.createRigidBody(geom2, true);
		phyz::RigidBody::PKey draw_p = r2->trackPoint(mthz::Vec3(0, -2, 0));
		r2->setCOMtoPosition(mthz::Vec3(0, -5, 0));
		bodies.push_back({ m2, r2 });

		phyz::PhysicsEngine::MotorID throttle;
		phyz::PhysicsEngine::MotorID steering;
		//{
			mthz::Vec3 pos(0, 1, 0);
			
			mthz::Vec3 base_dim(1, 0.1, 1);
			phyz::Geometry base = phyz::Geometry::box(pos, base_dim.x, base_dim.y, base_dim.z);

			double base_gear_radius = 0.15;
			double base_gear_tooth_size = 0.1;
			double base_gear_height = 0.1;
			int base_gear_teeth = 8;
			mthz::Vec3 base_gear_position = pos + mthz::Vec3(base_dim.x / 2.0, base_dim.y, base_dim.z / 2.0);
			phyz::Geometry base_gear = phyz::Geometry::gear(base_gear_position, base_gear_radius, base_gear_tooth_size, base_gear_height, base_gear_teeth, true, 5);

			double transmission_gear1_radius = 0.1;
			double transmission_gear1_tooth_size = 0.1;
			double transmission_gear1_height = 0.1;
			int transmission_gear1_teeth = 15;
			mthz::Vec3 transmission_gear1_position = base_gear_position + mthz::Vec3(0.25, 0.3, 0);
			phyz::Geometry transmission_gear1 = phyz::Geometry::gear(transmission_gear1_position, transmission_gear1_radius, transmission_gear1_tooth_size, transmission_gear1_height, transmission_gear1_teeth, false)
				.getRotated(mthz::Quaternion(-3.1415926535/2.0, mthz::Vec3(0, 0, 1)), transmission_gear1_position);

			double drive_shaft_length = 0.75;
			double drive_shaft_width = 0.10;
			mthz::Vec3 drive_shaft_position = transmission_gear1_position + mthz::Vec3(transmission_gear1_height, 0, 0);
			phyz::Geometry drive_shaft = phyz::Geometry::box(drive_shaft_position + mthz::Vec3(0, -drive_shaft_width / 2.0, -drive_shaft_width / 2.0), drive_shaft_length, drive_shaft_width, drive_shaft_width);

			double transmission_gear2_radius = 0.15;
			double transmission_gear2_tooth_size = 0.075;
			double transmission_gear2_height = 0.1;
			int transmission_gear2_teeth = 10;
			mthz::Vec3 transmission_gear2_position = drive_shaft_position + mthz::Vec3(drive_shaft_length, 0, 0);
			phyz::Geometry transmission_gear2 = phyz::Geometry::gear(transmission_gear2_position, transmission_gear2_radius, transmission_gear2_tooth_size, transmission_gear2_height, transmission_gear2_teeth, true)
				.getRotated(mthz::Quaternion(-3.1415926535 / 2.0, mthz::Vec3(0, 0, 1)), transmission_gear2_position);

			double differential_gear1_radius = 0.44;
			double differential_gear1_inner_radius = differential_gear1_radius * 0.5;
			double differential_gear1_height = 0.1;
			double differential_gear1_tooth_height = 0.075;
			double differential_gear1_tooth_radius = 0.1;
			double differential_gear1_tooth_width = 3.1415926535 * transmission_gear2_radius / transmission_gear2_teeth;
			int differential_gear1_teeth = 19;
			mthz::Vec3 differential_center = transmission_gear2_position + mthz::Vec3(differential_gear1_radius, 0, 0);
			mthz::Vec3 differential_gear1_position = differential_center + mthz::Vec3(0, 0, transmission_gear2_radius + transmission_gear2_tooth_size / 3.0 + differential_gear1_height + differential_gear1_tooth_height);
			phyz::Geometry differential_gear1 = phyz::Geometry::bevelGear(differential_gear1_position, differential_gear1_radius, differential_gear1_tooth_radius, differential_gear1_tooth_width, 
				differential_gear1_tooth_height, differential_gear1_height, differential_gear1_teeth, false, 1.0, differential_gear1_inner_radius).getRotated(mthz::Quaternion(-3.1415926535 / 2.0, mthz::Vec3(1, 0, 0)), differential_gear1_position);

			double differential_arm_radius = 0.6 * differential_gear1_radius;
			double differential_arm_width = 0.05;
			double differential_arm_length = differential_gear1_position.z - differential_gear1_height - drive_shaft_position.z + differential_arm_width/2.0;
			mthz::Vec3 differential_arm1_position = differential_gear1_position + mthz::Vec3(0, differential_arm_radius, -differential_gear1_height);
			mthz::Vec3 differential_arm2_position = differential_gear1_position + mthz::Vec3(0, -differential_arm_radius, -differential_gear1_height);
			phyz::Geometry differential_arm1 = phyz::Geometry::box(differential_arm1_position - mthz::Vec3(differential_arm_width/2.0, differential_arm_width/2.0, 0), differential_arm_width, differential_arm_width, -differential_arm_length);
			phyz::Geometry differential_arm2 = phyz::Geometry::box(differential_arm2_position - mthz::Vec3(differential_arm_width/2.0, differential_arm_width/2.0, 0), differential_arm_width, differential_arm_width, -differential_arm_length);

			double differential_gear2_height = 0.04;
			double differential_gear2_radius = 0.1;
			double differential_gear2_gap = 0.01;
			mthz::Vec3 differential_elbow1_position = mthz::Vec3(differential_arm1_position.x, differential_arm1_position.y - differential_arm_width / 2.0, drive_shaft_position.z);
			double differential_elbow_height = differential_elbow1_position.y - differential_center.y - differential_gear2_height - differential_gear2_radius - differential_gear2_gap;
			phyz::Geometry differential_elbow1 = phyz::Geometry::box(differential_elbow1_position - mthz::Vec3(differential_arm_width / 2.0, 0, differential_arm_width / 2.0), differential_arm_width, -differential_elbow_height, differential_arm_width);

			mthz::Vec3 differential_elbow2_position = mthz::Vec3(differential_arm2_position.x, differential_arm2_position.y + differential_arm_width / 2.0, drive_shaft_position.z);
			phyz::Geometry differential_elbow2 = phyz::Geometry::box(differential_elbow2_position - mthz::Vec3(differential_arm_width / 2.0, 0, differential_arm_width / 2.0), differential_arm_width, differential_elbow_height, differential_arm_width);


			double differential_gear2_tooth_size = 0.05;
			double differential_gear2_teeth = 12;
			mthz::Vec3 differential_gear2_position = differential_elbow1_position + mthz::Vec3(0, -differential_elbow_height, 0);
			phyz::Geometry differential_gear2 = phyz::Geometry::gear(differential_gear2_position, differential_gear2_radius, differential_gear2_tooth_size, differential_gear2_height, differential_gear2_teeth, false, 8)
				.getRotated(mthz::Quaternion(3.1415926535, mthz::Vec3(0, 0, 1)), differential_gear2_position);

			mthz::Vec3 differential_gear3_position = differential_elbow2_position + mthz::Vec3(0, differential_elbow_height, 0);
			phyz::Geometry differential_gear3 = phyz::Geometry::gear(differential_gear3_position, differential_gear2_radius, differential_gear2_tooth_size, differential_gear2_height, differential_gear2_teeth, false, 8);

			double axle1_gear_radius = differential_gear2_radius;
			double axle1_gear_tooth_size = differential_gear2_tooth_size;
			double axle1_gear_height = differential_gear2_height;
			int axle1_gear_teeth = differential_gear2_teeth;
			mthz::Vec3 axle1_gear_position = differential_center + mthz::Vec3(0, 0, differential_gear2_radius + differential_gear2_tooth_size / 4.0);
			phyz::Geometry axle1_gear = phyz::Geometry::gear(axle1_gear_position, axle1_gear_radius, axle1_gear_tooth_size, axle1_gear_height, axle1_gear_teeth, true, 0.1)
				.getRotated(mthz::Quaternion(3.1415926535 / 2.0, mthz::Vec3(1, 0, 0)), axle1_gear_position);

			mthz::Vec3 axle2_gear_position = differential_center - mthz::Vec3(0, 0, differential_gear2_radius + differential_gear2_tooth_size / 4.0);
			phyz::Geometry axle2_gear = phyz::Geometry::gear(axle2_gear_position, axle1_gear_radius, axle1_gear_tooth_size, axle1_gear_height, axle1_gear_teeth, true, 0.1)
				.getRotated(mthz::Quaternion(-3.1415926535 / 2.0, mthz::Vec3(1, 0, 0)), axle2_gear_position);

			double axle1_width = 0.1;
			double axle1_length = 0.75;
			mthz::Vec3 axle1_position = axle1_gear_position + mthz::Vec3(0, 0, axle1_gear_height);
			phyz::Geometry axle1 = phyz::Geometry::box(axle1_position + mthz::Vec3(-axle1_width / 2.0, -axle1_width / 2.0, 0), axle1_width, axle1_width, axle1_length, 0.1);

			mthz::Vec3 axle2_position = axle2_gear_position - mthz::Vec3(0, 0, axle1_gear_height);
			phyz::Geometry axle2 = phyz::Geometry::box(axle2_position + mthz::Vec3(-axle1_width / 2.0, -axle1_width / 2.0, 0), axle1_width, axle1_width, -axle1_length, 0.1);

			double rear_wheel1_radius = 0.75;
			double rear_wheel1_height = 0.15;
			mthz::Vec3 rear_wheel1_position = axle1_position + mthz::Vec3(0, 0, axle1_length);
			phyz::Geometry rear_wheel1 = phyz::Geometry::cylinder(rear_wheel1_position, rear_wheel1_radius, rear_wheel1_height, 20, 0.1)
				.getRotated(mthz::Quaternion(3.1415926535 / 2.0, mthz::Vec3(1, 0, 0)), rear_wheel1_position);

			mthz::Vec3 rear_wheel2_position = axle2_position - mthz::Vec3(0, 0, axle1_length);
			phyz::Geometry rear_wheel2 = phyz::Geometry::cylinder(rear_wheel2_position, rear_wheel1_radius, rear_wheel1_height, 20, 0.1)
				.getRotated(mthz::Quaternion(-3.1415926535 / 2.0, mthz::Vec3(1, 0, 0)), rear_wheel2_position);

			double steering_wheel_radius = 0.2;
			double steering_wheel_width = 0.1;
			mthz::Vec3 steering_wheel_position = pos + mthz::Vec3(1.25, 1.0, 0.85);
			mthz::Quaternion steering_wheel_orientation(1.2 * 3.1415926535 / 2.0, mthz::Vec3(0, 0, 1));
			phyz::Geometry steering_wheel_w = phyz::Geometry::cylinder(steering_wheel_position, steering_wheel_radius, steering_wheel_width);

			double steering_wheel_rod1_length = 0.5;
			double steering_wheel_rod1_width = 0.1;
			mthz::Vec3 steering_wheel_rod1_position = steering_wheel_position + mthz::Vec3(0, steering_wheel_width, 0);
			phyz::Geometry steering_wheel_rod1 = phyz::Geometry::box(steering_wheel_rod1_position - mthz::Vec3(steering_wheel_rod1_width/2.0, 0, steering_wheel_rod1_width/2.0), 
				steering_wheel_rod1_width, steering_wheel_rod1_length, steering_wheel_rod1_width);

			double sw_joint1_arm1_length = 0.1;
			double sw_joint1_arm1_width = steering_wheel_rod1_width * 0.45;
			double sw_joint1_arm1_height = 0.015;
			mthz::Vec3 sw_joint1_arm1_position = steering_wheel_rod1_position + mthz::Vec3(steering_wheel_rod1_width/2.0, steering_wheel_rod1_length, 0);
			phyz::Geometry sw_joint1_arm1 = phyz::Geometry::box(sw_joint1_arm1_position + mthz::Vec3(-sw_joint1_arm1_height, 0, -sw_joint1_arm1_width / 2.0), sw_joint1_arm1_height, sw_joint1_arm1_length, sw_joint1_arm1_width);

			mthz::Vec3 sw_joint1_arm2_position = steering_wheel_rod1_position + mthz::Vec3(-steering_wheel_rod1_width/2.0, steering_wheel_rod1_length, 0);
			phyz::Geometry sw_joint1_arm2 = phyz::Geometry::box(sw_joint1_arm2_position + mthz::Vec3(0, 0, -sw_joint1_arm1_width / 2.0), sw_joint1_arm1_height, sw_joint1_arm1_length, sw_joint1_arm1_width);

			double sw_joint1_pivot_radius = steering_wheel_rod1_width * 0.38;
			double sw_joint1_pivot_width = 0.03;
			double sw_joint1_gap = sw_joint1_arm1_length * 0.8;
			mthz::Vec3 sw_joint1_pivot_position = steering_wheel_rod1_position + mthz::Vec3(0, steering_wheel_rod1_length + sw_joint1_gap - sw_joint1_pivot_width / 2.0, 0);
			mthz::Vec3 sw_joint1_pivot_pos_oriented = steering_wheel_orientation.rotateAbout(sw_joint1_pivot_position, steering_wheel_position);
			phyz::Geometry sw_joint1_pivot = phyz::Geometry::cylinder(sw_joint1_pivot_position, sw_joint1_pivot_radius, sw_joint1_pivot_width, 10, 70).getRotated(steering_wheel_orientation, steering_wheel_position);

			mthz::Vec3 steering_wheel_rod2_target = pos + mthz::Vec3(0.1, 0.55, base_dim.z/2.0);
			mthz::Vec3 rod2_target_diff = steering_wheel_rod2_target - sw_joint1_pivot_pos_oriented;
			double rod2_target_diff_dist = rod2_target_diff.mag();
			double elev_angle = asin(-rod2_target_diff.y / rod2_target_diff_dist);
			mthz::Quaternion rod2_orientation = mthz::Quaternion(asin(rod2_target_diff.z / (rod2_target_diff_dist * cos(elev_angle))), mthz::Vec3(0, 1, 0))
				* mthz::Quaternion(3.1415926535/2.0 + elev_angle, mthz::Vec3(0, 0, 1));

			double steering_wheel_rod2_length = rod2_target_diff_dist - 2 * sw_joint1_gap;
			mthz::Vec3 steering_wheel_rod2_position = mthz::Vec3(0, sw_joint1_gap, 0);
			phyz::Geometry steering_wheel_rod2 = phyz::Geometry::box(steering_wheel_rod2_position - mthz::Vec3(steering_wheel_rod1_width / 2.0, 0, steering_wheel_rod1_width / 2.0),
				steering_wheel_rod1_width, steering_wheel_rod2_length, steering_wheel_rod1_width);

			mthz::Vec3 sw_joint1_arm3_position = steering_wheel_rod2_position + mthz::Vec3(0, 0, steering_wheel_rod1_width / 2.0);
			phyz::Geometry sw_joint1_arm3 = phyz::Geometry::box(sw_joint1_arm3_position + mthz::Vec3(-sw_joint1_arm1_width / 2.0, 0,  -sw_joint1_arm1_height), sw_joint1_arm1_width, -sw_joint1_arm1_length, sw_joint1_arm1_height);
			mthz::Vec3 sw_joint1_arm4_position = steering_wheel_rod2_position + mthz::Vec3(0, 0, -steering_wheel_rod1_width / 2.0);
			phyz::Geometry sw_joint1_arm4 = phyz::Geometry::box(sw_joint1_arm4_position + mthz::Vec3(-sw_joint1_arm1_width / 2.0, 0, -sw_joint1_arm1_height / 2.0), sw_joint1_arm1_width, -sw_joint1_arm1_length, sw_joint1_arm1_height);
			mthz::Vec3 sw_joint2_arm1_position = steering_wheel_rod2_position + mthz::Vec3(steering_wheel_rod1_width / 2.0, steering_wheel_rod2_length, 0);
			phyz::Geometry sw_joint2_arm1 = phyz::Geometry::box(sw_joint2_arm1_position + mthz::Vec3(-sw_joint1_arm1_height, 0, -sw_joint1_arm1_width / 2.0), sw_joint1_arm1_height, sw_joint1_arm1_length, sw_joint1_arm1_width);
			mthz::Vec3 sw_joint2_arm2_position = steering_wheel_rod2_position + mthz::Vec3(-steering_wheel_rod1_width / 2.0, steering_wheel_rod2_length, 0);
			phyz::Geometry sw_joint2_arm2 = phyz::Geometry::box(sw_joint2_arm2_position + mthz::Vec3(0, 0, -sw_joint1_arm1_width / 2.0), sw_joint1_arm1_height, sw_joint1_arm1_length, sw_joint1_arm1_width);

			mthz::Vec3 sw_joint2_pivot_position = steering_wheel_rod2_position + mthz::Vec3(0, steering_wheel_rod2_length + sw_joint1_gap - sw_joint1_pivot_width / 2.0, 0);
			mthz::Vec3 sw_joint2_pivot_pos_oriented = sw_joint1_pivot_pos_oriented + rod2_orientation.rotateAbout(sw_joint2_pivot_position, mthz::Vec3(0, 0, 0));
			phyz::Geometry sw_joint2_pivot = phyz::Geometry::cylinder(sw_joint2_pivot_position, sw_joint1_pivot_radius, sw_joint1_pivot_width, 10, 70).getRotated(rod2_orientation).getTranslated(sw_joint1_pivot_pos_oriented);

			double steering_wheel_rod3_length = 0.5;
			mthz::Vec3 steering_wheel_rod3_position = mthz::Vec3(0, sw_joint1_gap, 0);
			phyz::Geometry steering_wheel_rod3 = phyz::Geometry::box(steering_wheel_rod3_position - mthz::Vec3(steering_wheel_rod1_width / 2.0, 0, steering_wheel_rod1_width / 2.0),
				steering_wheel_rod1_width, steering_wheel_rod3_length, steering_wheel_rod1_width);
			mthz::Quaternion rod3_orientation = mthz::Quaternion(3.1415926535 / 2.0, mthz::Vec3(0, 0, 1));

			mthz::Vec3 sw_joint2_arm3_position = steering_wheel_rod3_position + mthz::Vec3(0, 0, steering_wheel_rod1_width / 2.0);
			phyz::Geometry sw_joint2_arm3 = phyz::Geometry::box(sw_joint2_arm3_position + mthz::Vec3(-sw_joint1_arm1_width / 2.0, 0, -sw_joint1_arm1_height), sw_joint1_arm1_width, -sw_joint1_arm1_length, sw_joint1_arm1_height);
			mthz::Vec3 sw_joint2_arm4_position = steering_wheel_rod3_position + mthz::Vec3(0, 0, -steering_wheel_rod1_width / 2.0);
			phyz::Geometry sw_joint2_arm4 = phyz::Geometry::box(sw_joint2_arm4_position + mthz::Vec3(-sw_joint1_arm1_width / 2.0, 0, -sw_joint1_arm1_height / 2.0), sw_joint1_arm1_width, -sw_joint1_arm1_length, sw_joint1_arm1_height);

			double steering_wheel_gear_radius = 0.05;
			double steering_wheel_gear_height = 0.1;
			double steering_wheel_gear_tooth_size = 0.045;
			int steering_wheel_gear_teeth = 10;
			mthz::Vec3 steering_wheel_gear_position = steering_wheel_rod3_position + mthz::Vec3(0, steering_wheel_rod3_length, 0);
			phyz::Geometry steering_wheel_gear = phyz::Geometry::gear(steering_wheel_gear_position, steering_wheel_gear_radius, steering_wheel_gear_tooth_size, steering_wheel_gear_height, steering_wheel_gear_teeth);

			double pinion_tooth_width = 3.1415926535 * steering_wheel_gear_radius / (steering_wheel_gear_teeth);
			double pinion_gap_width = 3.1415926535 * (steering_wheel_gear_radius + steering_wheel_gear_tooth_size) / (steering_wheel_gear_teeth);
			int pinion_n_teeth = 8;
			double pinion_length = (1 + pinion_n_teeth) * pinion_gap_width + pinion_n_teeth * pinion_tooth_width;
			double pinion_height = 0.1;
			mthz::Vec3 pinion_position = rod3_orientation.applyRotation(steering_wheel_gear_position) + sw_joint2_pivot_pos_oriented + 
				mthz::Vec3(-steering_wheel_gear_height, -steering_wheel_gear_radius - pinion_height - 1.25 * steering_wheel_gear_tooth_size, -pinion_length / 2.0);
			phyz::Geometry pinion = phyz::Geometry::pinion(pinion_position, pinion_height, steering_wheel_gear_height, steering_wheel_gear_tooth_size, pinion_tooth_width, pinion_gap_width, pinion_n_teeth);

			mthz::Vec3 front_wheel1_position = mthz::Vec3(pinion_position.x - 0.5, rear_wheel1_position.y, rear_wheel1_position.z);
			phyz::Geometry front_wheel1 = phyz::Geometry::cylinder(front_wheel1_position, rear_wheel1_radius, rear_wheel1_height, 20, 0.1)
				.getRotated(mthz::Quaternion(3.1415926535 / 2.0, mthz::Vec3(1, 0, 0)), rear_wheel1_position);

			double front_wheel1_elbow_length = 0.125;
			double front_wheel1_arm_width = 0.125;
			phyz::Geometry front_wheel1_elbow = phyz::Geometry::box(front_wheel1_position - mthz::Vec3(front_wheel1_arm_width / 2.0, front_wheel1_arm_width / 2.0, 0), front_wheel1_arm_width, front_wheel1_arm_width,
				-front_wheel1_elbow_length);
			
			double front_wheel1_arm_length = 0.4;
			mthz::Vec3 front_wheel1_arm_pos = front_wheel1_position + mthz::Vec3(0, 0, -front_wheel1_elbow_length - front_wheel1_arm_width / 2.0);
			phyz::Geometry front_wheel1_forearm = phyz::Geometry::box(front_wheel1_arm_pos + mthz::Vec3(-front_wheel1_arm_width / 2.0, -front_wheel1_arm_width / 2.0, -front_wheel1_arm_width / 2.0),
				front_wheel1_arm_length + front_wheel1_arm_width / 2.0, front_wheel1_arm_width, front_wheel1_arm_width);

			mthz::Vec3 front_wheel1_rod_pinion_connect = pinion_position + mthz::Vec3(steering_wheel_gear_height/2.0, pinion_height/2.0, pinion_length);
			mthz::Vec3 front_wheel1_rod_arm_connect = front_wheel1_arm_pos + mthz::Vec3(front_wheel1_arm_length, 0, 0);
			mthz::Vec3 front_wheel1_rod_dir = front_wheel1_rod_arm_connect - front_wheel1_rod_pinion_connect;
			double front_wheel1_rod_width = 0.05;
			double front_wheel1_arm_dist = front_wheel1_rod_dir.mag();
			double elev = asin(-front_wheel1_rod_dir.y / front_wheel1_arm_dist);
			double rot = asin(front_wheel1_rod_dir.z / (cos(elev) * front_wheel1_arm_dist));
			phyz::Geometry front_wheel1_rod = phyz::Geometry::box(front_wheel1_rod_pinion_connect + mthz::Vec3(0, -front_wheel1_rod_width / 2.0, -front_wheel1_rod_width / 2.0),
				-front_wheel1_arm_dist, front_wheel1_rod_width, front_wheel1_rod_width)
				.getRotated(mthz::Quaternion(elev, mthz::Vec3(0, 0, 1)), front_wheel1_rod_pinion_connect).getRotated(mthz::Quaternion(rot, mthz::Vec3(0, 1, 0)), front_wheel1_rod_pinion_connect);

			mthz::Vec3 front_wheel2_position = mthz::Vec3(pinion_position.x - 0.5, rear_wheel1_position.y, rear_wheel2_position.z);
			phyz::Geometry front_wheel2 = phyz::Geometry::cylinder(front_wheel2_position, rear_wheel1_radius, rear_wheel1_height, 20, 0.1)
				.getRotated(mthz::Quaternion(-3.1415926535 / 2.0, mthz::Vec3(1, 0, 0)), rear_wheel2_position);

			phyz::Geometry front_wheel2_elbow = phyz::Geometry::box(front_wheel2_position - mthz::Vec3(front_wheel1_arm_width / 2.0, front_wheel1_arm_width / 2.0, 0), front_wheel1_arm_width, front_wheel1_arm_width,
				front_wheel1_elbow_length);

			mthz::Vec3 front_wheel2_arm_pos = front_wheel2_position + mthz::Vec3(0, 0, front_wheel1_elbow_length + front_wheel1_arm_width / 2.0);
			phyz::Geometry front_wheel2_forearm = phyz::Geometry::box(front_wheel2_arm_pos + mthz::Vec3(-front_wheel1_arm_width / 2.0, -front_wheel1_arm_width / 2.0, -front_wheel1_arm_width / 2.0),
				front_wheel1_arm_length + front_wheel1_arm_width / 2.0, front_wheel1_arm_width, front_wheel1_arm_width);

			mthz::Vec3 front_wheel2_rod_pinion_connect = pinion_position + mthz::Vec3(steering_wheel_gear_height / 2.0, pinion_height / 2.0, 0);
			mthz::Vec3 front_wheel2_rod_arm_connect = front_wheel2_arm_pos + mthz::Vec3(front_wheel1_arm_length, 0, 0);
			phyz::Geometry front_wheel2_rod = phyz::Geometry::box(front_wheel2_rod_pinion_connect + mthz::Vec3(0, -front_wheel1_rod_width / 2.0, -front_wheel1_rod_width / 2.0),
				-front_wheel1_arm_dist, front_wheel1_rod_width, front_wheel1_rod_width)
				.getRotated(mthz::Quaternion(elev, mthz::Vec3(0, 0, 1)), front_wheel2_rod_pinion_connect).getRotated(mthz::Quaternion(-rot, mthz::Vec3(0, 1, 0)), front_wheel2_rod_pinion_connect);

			phyz::Geometry transmission = { transmission_gear1, drive_shaft, transmission_gear2 };
			phyz::Geometry differential = { differential_gear1, differential_arm1, differential_elbow1, differential_arm2, differential_elbow2 };
			phyz::Geometry rear_wheel_c1 = { axle1_gear, axle1, rear_wheel1 };
			phyz::Geometry rear_wheel_c2 = { axle2_gear, axle2, rear_wheel2 };
			
			phyz::Geometry steering_wheel = phyz::Geometry({ steering_wheel_w, steering_wheel_rod1, sw_joint1_arm1, sw_joint1_arm2 })
				.getRotated(steering_wheel_orientation, steering_wheel_position);
			phyz::Geometry steering_wheel_rod_c2 = phyz::Geometry{ steering_wheel_rod2, sw_joint1_arm3, sw_joint1_arm4, sw_joint2_arm1, sw_joint2_arm2 }
				.getRotated(rod2_orientation).getTranslated(sw_joint1_pivot_pos_oriented);
			phyz::Geometry steering_wheel_rod_c3 = phyz::Geometry{ steering_wheel_rod3, sw_joint2_arm3, sw_joint2_arm4, steering_wheel_gear }
				.getRotated(rod3_orientation).getTranslated(sw_joint2_pivot_pos_oriented);

			mthz::Vec3 steering_wheel_axis = steering_wheel_orientation.applyRotation(mthz::Vec3(0, 1, 0));
			mthz::Vec3 steering_wheel_rod2_axis = rod2_orientation.applyRotation(mthz::Vec3(0, 1, 0));
			mthz::Vec3 steering_wheel_rod3_axis = rod3_orientation.applyRotation(mthz::Vec3(0, 1, 0));

			phyz::Geometry front_wheel1_arm = { front_wheel1_elbow, front_wheel1_forearm };
			phyz::Geometry front_wheel2_arm = { front_wheel2_elbow, front_wheel2_forearm };
			bool steering_lock = false;

			phyz::RigidBody* base_r = p.createRigidBody(base, true);
			std::vector<Mesh> base_m = { fromGeometry(base) };
			//phyz::RigidBody* base_gear_r = p.createRigidBody(base_gear);
			//std::vector<Mesh> base_gear_m = { fromGeometry(base_gear) };
			phyz::RigidBody* transmission_r = p.createRigidBody(transmission);
			std::vector<Mesh> transmission_m = { fromGeometry(transmission) };
			phyz::RigidBody* differential_r = p.createRigidBody(differential);
			std::vector<Mesh> differential_m = { fromGeometry(differential) };
			phyz::RigidBody* differential_gear2_r = p.createRigidBody(differential_gear2);
		    std::vector<Mesh> differential_gear2_m = { fromGeometry(differential_gear2) };
			phyz::RigidBody* differential_gear3_r = p.createRigidBody(differential_gear3);
			std::vector<Mesh> differential_gear3_m = { fromGeometry(differential_gear3) };
			phyz::RigidBody* rear_wheel1_r = p.createRigidBody(rear_wheel_c1);
			std::vector<Mesh> rear_wheel1_m = { fromGeometry(rear_wheel_c1) };
			phyz::RigidBody* rear_wheel2_r = p.createRigidBody(rear_wheel_c2);
			std::vector<Mesh> rear_wheel2_m = { fromGeometry(rear_wheel_c2) };
			phyz::RigidBody* front_wheel1_r = p.createRigidBody(front_wheel1);
			std::vector<Mesh> front_wheel1_m = { fromGeometry(front_wheel1) };
			phyz::RigidBody* front_wheel1_arm_r = p.createRigidBody(front_wheel1_arm);
			std::vector<Mesh> front_wheel1_arm_m = { fromGeometry(front_wheel1_arm) };
			phyz::RigidBody* front_wheel2_r = p.createRigidBody(front_wheel2);
			std::vector<Mesh> front_wheel2_m = { fromGeometry(front_wheel2) };
			phyz::RigidBody* front_wheel2_arm_r = p.createRigidBody(front_wheel2_arm);
			std::vector<Mesh> front_wheel2_arm_m = { fromGeometry(front_wheel2_arm) };
			phyz::RigidBody* steering_wheel_r = p.createRigidBody(steering_wheel, steering_lock);
			std::vector<Mesh> steering_wheel_m = { fromGeometry(steering_wheel) };
			phyz::RigidBody* sw_joint1_pivot_r = p.createRigidBody(sw_joint1_pivot);
			std::vector<Mesh> sw_joint1_pivot_m = { fromGeometry(sw_joint1_pivot) };
			phyz::RigidBody* sw_joint2_pivot_r = p.createRigidBody(sw_joint2_pivot);
			std::vector<Mesh> sw_joint2_pivot_m = { fromGeometry(sw_joint2_pivot) };
			phyz::RigidBody* steering_wheel_rod2_r = p.createRigidBody(steering_wheel_rod_c2, steering_lock);
			std::vector<Mesh> steering_wheel_rod2_m = { fromGeometry(steering_wheel_rod_c2) };
			phyz::RigidBody* steering_wheel_rod3_r = p.createRigidBody(steering_wheel_rod_c3, steering_lock);
			std::vector<Mesh> steering_wheel_rod3_m = { fromGeometry(steering_wheel_rod_c3) };
			phyz::RigidBody* pinion_r = p.createRigidBody(pinion, steering_lock);
			std::vector<Mesh> pinion_m = { fromGeometry(pinion) };
			phyz::RigidBody* front_wheel1_rod_r = p.createRigidBody(front_wheel1_rod);
			std::vector<Mesh> front_wheel1_rod_m = { fromGeometry(front_wheel1_rod) };
			phyz::RigidBody* front_wheel2_rod_r = p.createRigidBody(front_wheel2_rod, false);
			std::vector<Mesh> front_wheel2_rod_m = { fromGeometry(front_wheel2_rod) };

			//p.addHingeConstraint(base_r, base_gear_r, base_gear_position, base_gear_position, mthz::Vec3(0, 1, 0), mthz::Vec3(0, 1, 0));
			throttle = p.addHingeConstraint(base_r, transmission_r, transmission_gear1_position, transmission_gear1_position, mthz::Vec3(1, 0, 0), mthz::Vec3(1, 0, 0));
			p.addHingeConstraint(base_r, differential_r, differential_gear1_position, differential_gear1_position, mthz::Vec3(0, 0, -1), mthz::Vec3(0, 0, -1));
			p.addHingeConstraint(differential_r, differential_gear2_r, differential_gear2_position, differential_gear2_position, mthz::Vec3(0, 1, 0), mthz::Vec3(0, 1, 0));
			p.addHingeConstraint(differential_r, differential_gear3_r, differential_gear3_position, differential_gear3_position, mthz::Vec3(0, 1, 0), mthz::Vec3(0, 1, 0));
			p.addHingeConstraint(base_r, rear_wheel1_r, axle1_gear_position, axle1_gear_position, mthz::Vec3(0, 0, -1), mthz::Vec3(0, 0, -1));
			p.addHingeConstraint(base_r, rear_wheel2_r, axle2_gear_position, axle2_gear_position, mthz::Vec3(0, 0, -1), mthz::Vec3(0, 0, -1));
			//p.addHingeConstraint(base_r, front_wheel2_r, rear_wheel2_position + mthz::Vec3(-2.5, 0, 0), rear_wheel2_position + mthz::Vec3(-2.5, 0, 0), mthz::Vec3(0, 0, -1), mthz::Vec3(0, 0, -1));
			steering = p.addHingeConstraint(base_r, steering_wheel_r, steering_wheel_position, steering_wheel_position, steering_wheel_axis, steering_wheel_axis);
			p.addHingeConstraint(base_r, steering_wheel_rod2_r, sw_joint1_pivot_pos_oriented, sw_joint1_pivot_pos_oriented, steering_wheel_rod2_axis, steering_wheel_rod2_axis);
			mthz::Vec3 sw_joint1_pivot_axis1 = steering_wheel_orientation.applyRotation(mthz::Vec3(1, 0, 0));
			mthz::Vec3 sw_joint1_pivot_axis2 = steering_wheel_orientation.applyRotation(mthz::Vec3(0, 0, 1));
			mthz::Vec3 sw_joint1_pivot_axis3 = rod2_orientation.applyRotation(mthz::Vec3(0, 0, 1));
			p.addHingeConstraint(steering_wheel_r, sw_joint1_pivot_r, sw_joint1_pivot_pos_oriented, sw_joint1_pivot_pos_oriented, sw_joint1_pivot_axis1, sw_joint1_pivot_axis1);
			p.addHingeConstraint(steering_wheel_rod2_r, sw_joint1_pivot_r, sw_joint1_pivot_pos_oriented, sw_joint1_pivot_pos_oriented, sw_joint1_pivot_axis3, sw_joint1_pivot_axis2);
			p.addHingeConstraint(base_r, steering_wheel_rod3_r, sw_joint2_pivot_pos_oriented, sw_joint2_pivot_pos_oriented, steering_wheel_rod3_axis, steering_wheel_rod3_axis);
			mthz::Vec3 sw_joint2_pivot_axis1 = rod2_orientation.applyRotation(mthz::Vec3(1, 0, 0));
			mthz::Vec3 sw_joint2_pivot_axis2 = rod2_orientation.applyRotation(mthz::Vec3(0, 0, 1));
			mthz::Vec3 sw_joint2_pivot_axis3 = rod3_orientation.applyRotation(mthz::Vec3(0, 0, 1));
			p.addHingeConstraint(steering_wheel_rod2_r, sw_joint2_pivot_r, sw_joint2_pivot_pos_oriented, sw_joint2_pivot_pos_oriented, sw_joint2_pivot_axis1, sw_joint2_pivot_axis1);
			p.addHingeConstraint(steering_wheel_rod3_r, sw_joint2_pivot_r, sw_joint2_pivot_pos_oriented, sw_joint2_pivot_pos_oriented, sw_joint2_pivot_axis3, sw_joint2_pivot_axis2);
			p.addSliderConstraint(base_r, pinion_r, pinion_position, pinion_position, mthz::Vec3(0, 0, 1), mthz::Vec3(0, 0, 1));
			p.addHingeConstraint(front_wheel1_rod_r, front_wheel1_arm_r, front_wheel1_rod_arm_connect, front_wheel1_rod_arm_connect, mthz::Vec3(0, 1, 0), mthz::Vec3(0, 1, 0));
			p.addBallSocketConstraint(front_wheel1_rod_r, pinion_r, front_wheel1_rod_pinion_connect, front_wheel1_rod_pinion_connect);
			p.addHingeConstraint(front_wheel1_arm_r, front_wheel1_r, front_wheel1_position, front_wheel1_position, mthz::Vec3(0, 0, -1), mthz::Vec3(0, 0, -1));
			p.addHingeConstraint(base_r, front_wheel1_arm_r, front_wheel1_arm_pos, front_wheel1_arm_pos, mthz::Vec3(0, 1, 0), mthz::Vec3(0, 1, 0));
			p.addHingeConstraint(front_wheel2_rod_r, front_wheel2_arm_r, front_wheel2_rod_arm_connect, front_wheel2_rod_arm_connect, mthz::Vec3(0, 1, 0), mthz::Vec3(0, 1, 0));
			p.addBallSocketConstraint(front_wheel2_rod_r, pinion_r, front_wheel2_rod_pinion_connect, front_wheel2_rod_pinion_connect);
			p.addHingeConstraint(front_wheel2_arm_r, front_wheel2_r, front_wheel2_position, front_wheel2_position, mthz::Vec3(0, 0, -1), mthz::Vec3(0, 0, -1));
			p.addHingeConstraint(base_r, front_wheel2_arm_r, front_wheel2_arm_pos, front_wheel2_arm_pos, mthz::Vec3(0, 1, 0), mthz::Vec3(0, 1, 0));

			bodies.push_back({ base_m, base_r });
			//bodies.push_back({ base_gear_m, base_gear_r });
			bodies.push_back({ transmission_m, transmission_r });
			bodies.push_back({ differential_m, differential_r });
			bodies.push_back({ differential_gear2_m, differential_gear2_r });
			bodies.push_back({ differential_gear3_m, differential_gear3_r });
			bodies.push_back({ rear_wheel1_m, rear_wheel1_r });
			bodies.push_back({ rear_wheel2_m, rear_wheel2_r });
			bodies.push_back({ steering_wheel_m, steering_wheel_r });
			bodies.push_back({ sw_joint1_pivot_m, sw_joint1_pivot_r });
			bodies.push_back({ steering_wheel_rod2_m, steering_wheel_rod2_r });
			bodies.push_back({ sw_joint2_pivot_m, sw_joint2_pivot_r });
			bodies.push_back({ steering_wheel_rod3_m, steering_wheel_rod3_r });
			bodies.push_back({ pinion_m, pinion_r });
			bodies.push_back({ front_wheel1_m, front_wheel1_r });
			bodies.push_back({ front_wheel1_arm_m, front_wheel1_arm_r });
			bodies.push_back({ front_wheel1_rod_m, front_wheel1_rod_r });
			bodies.push_back({ front_wheel2_m, front_wheel2_r });
			bodies.push_back({ front_wheel2_arm_m, front_wheel2_arm_r });;
			bodies.push_back({ front_wheel2_rod_m, front_wheel2_rod_r });
		//}

		p.setMotor(throttle, -0.5, 35);
		p.setMotor(steering, -0.5, 35);


		rndr::Shader shader("resources/shaders/Basic.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Quaternion orient;
		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		double timestep = 1 / 90.0;
		p.setStep_time(timestep);
		p.setGravity(mthz::Vec3(0, -9.8, 0));

		while (rndr::render_loop(&fElapsedTime)) {

			if (rndr::getKeyDown(GLFW_KEY_W)) {
				pos += orient.applyRotation(mthz::Vec3(0, 0, -1) * fElapsedTime * mv_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_S)) {
				pos += orient.applyRotation(mthz::Vec3(0, 0, 1) * fElapsedTime * mv_speed);
			}
			if (rndr::getKeyDown(GLFW_KEY_A)) {
				pos += orient.applyRotation(mthz::Vec3(-1, 0, 0) * fElapsedTime * mv_speed);
			}
			else if (rndr::getKeyDown(GLFW_KEY_D)) {
				pos += orient.applyRotation(mthz::Vec3(1, 0, 0) * fElapsedTime * mv_speed);
			}

			if (rndr::getKeyDown(GLFW_KEY_UP)) {
				orient = orient * mthz::Quaternion(fElapsedTime * rot_speed, mthz::Vec3(1, 0, 0));
			}
			else if (rndr::getKeyDown(GLFW_KEY_DOWN)) {
				orient = orient * mthz::Quaternion(-fElapsedTime * rot_speed, mthz::Vec3(1, 0, 0));
			}
			if (rndr::getKeyDown(GLFW_KEY_LEFT)) {
				orient = mthz::Quaternion(fElapsedTime * rot_speed, mthz::Vec3(0, 1, 0)) * orient;
			}
			else if (rndr::getKeyDown(GLFW_KEY_RIGHT)) {
				orient = mthz::Quaternion(-fElapsedTime * rot_speed, mthz::Vec3(0, 1, 0)) * orient;
			}

			t += fElapsedTime;

			if (rndr::getKeyPressed(GLFW_KEY_V)) {
				static int i = -1;
				

				p.setMotor(throttle, --i, 100);
				//p.setMotor(steering, i*j, 100);
			}
			if (rndr::getKeyPressed(GLFW_KEY_G)) {
				static int i = 1;
				p.setMotor(steering, 1 - (i++ % 3), 100);
			}
			if (rndr::getKeyPressed(GLFW_KEY_X)) {
				
				base_r->setFixed(!base_r->getIsFixed());
				
			}
			if (rndr::getKeyPressed(GLFW_KEY_B)) {
				/*phyz::AABB aabb = bunny_mesh.getAABB();
				phyz::Geometry bunny_geom = phyz::Geometry::box(mthz::Vec3(aabb.min.x, aabb.min.y, aabb.min.z), aabb.max.x - aabb.min.x, aabb.max.y - aabb.min.y, aabb.max.z - aabb.min.z, 0.1);
				phyz::RigidBody* bunny_r = p.createRigidBody(bunny_geom);
				Mesh bunny = fromPhyzMesh(bunny_mesh);
				bodies.push_back({ std::vector<Mesh>{bunny}, bunny_r });*/

				rear_wheel2_r->setFixed(!rear_wheel2_r->getIsFixed());
			}
			if (rndr::getKeyPressed(GLFW_KEY_T)) {
				phyz::AABB aabb = teapot_mesh.getAABB();
				phyz::Geometry teapot_geom = phyz::Geometry::box(mthz::Vec3(aabb.min.x, aabb.min.y, aabb.min.z), aabb.max.x - aabb.min.x, aabb.max.y - aabb.min.y, aabb.max.z - aabb.min.z, 0.1);
				phyz::RigidBody* teapot_r = p.createRigidBody(teapot_geom);
				Mesh teapot = fromPhyzMesh(teapot_mesh);
				bodies.push_back({ std::vector<Mesh>{teapot}, teapot_r });
			}
			if (rndr::getKeyPressed(GLFW_KEY_X)) {
				phyz::AABB aabb = cow_mesh.getAABB();
				phyz::Geometry cow_geom = phyz::Geometry::box(mthz::Vec3(aabb.min.x, aabb.min.y, aabb.min.z), aabb.max.x - aabb.min.x, aabb.max.y - aabb.min.y, aabb.max.z - aabb.min.z, 0.1);
				phyz::RigidBody* cow_r = p.createRigidBody(cow_geom);
				Mesh cow = fromPhyzMesh(cow_mesh);
				bodies.push_back({ std::vector<Mesh>{cow}, cow_r });
			}

			//square
			if (rndr::getKeyPressed(GLFW_KEY_K)) {
				std::vector<Mesh> m1 = { rect(0, 0, 0, 1, 1, 1) };
				phyz::Geometry geom1 = phyz::Geometry::box(mthz::Vec3(-0.5, -0.5, -0.5), 1, 1, 1);
				double h = 10;
				phyz::RigidBody* r1 = p.createRigidBody(geom1, false, mthz::Vec3(0, h, 0));
				//r1->ang_vel = mthz::Vec3(0, 100, 1);
				//r1->vel = mthz::Vec3(1, 0, 0);
				//r1->com.x = 2.3;
				//r1->orientation = mthz::Quaternion(3.1415926535 / 4, mthz::Vec3(0, 0, 1)) * mthz::Quaternion(3.1415926535 / 4, mthz::Vec3(0, 1, 0));
				bodies.push_back({ m1, r1 });
			}
			//tetrahedron
			if (rndr::getKeyPressed(GLFW_KEY_I)) {
				mthz::Vec3 p1(0, 0, 0), p2(1, 0, 0), p3(0, 1, 0), p4(0, 0, 1);
				std::vector<Mesh> m1 = { tetra(p1, p2, p3, p4) };
				phyz::Geometry geom1 = phyz::Geometry::tetra(p1, p2, p3, p4);
				phyz::RigidBody* r1 = p.createRigidBody(geom1, false, mthz::Vec3(0, 10, 0));
				//r1->setAngVel(mthz::Vec3(0, 0, 0));
				//r1->setVel(mthz::Vec3(0, 0, 0));
				//r1->setOrientation(mthz::Quaternion(3.1415926535 / 2, mthz::Vec3(0, 0, 1)));
				bodies.push_back({ m1, r1 });
			}
			//top
			if (rndr::getKeyPressed(GLFW_KEY_J)) {
				double h1 = 1.8, w1 = 0.05, h2 = 0.2, w2 = 2, s = 0.13;
				std::vector<Mesh> m1 = { rect(0, 0, 0, w1, h1, w1), rect(0, -h1 * s / 2, 0, w2, h2, w2) };
				phyz::Geometry geom1 = { phyz::Geometry::box(mthz::Vec3(-w1 / 2, -h1 / 2, -w1 / 2), w1, h1, w1), phyz::Geometry::box(mthz::Vec3(-w2 / 2, -h2 / 2 - h1 * s / 2, -w2 / 2), w2, h2, w2)};
				phyz::RigidBody* r1 = p.createRigidBody(geom1);
				phyz::RigidBody::PKey draw_p = r1->trackPoint(mthz::Vec3(0, 0, 0));
				r1->setCOMtoPosition(mthz::Vec3(0, 10, 0));
				r1->setAngVel(mthz::Vec3(0, 40, 2.5));
				bodies.push_back({ m1, r1 });
			}

			//barbell
			if (rndr::getKeyPressed(GLFW_KEY_L)) {
				double h1 = 1.8, w1 = 0.2, h2 = 0.5, w2 = 1, s = 0.5;
				std::vector<Mesh> m1 = { rect(0, 0, 0, w1, h1, w1), rect(0, -h1 * s / 2, 0, w2, h2, w2), rect(0, h1 * s / 2, 0, w2, h2, w2) };
				phyz::Geometry geom1 = { 
					phyz::Geometry::box(mthz::Vec3(-w1 / 2, -h1 / 2, -w1 / 2), w1, h1, w1), 
					phyz::Geometry::box(mthz::Vec3(-w2 / 2, -h2 / 2 - h1 * s / 2, -w2 / 2), w2, h2, w2),
					phyz::Geometry::box(mthz::Vec3(-w2 / 2, -h2 / 2 + h1 * s / 2, -w2 / 2), w2, h2, w2) 
				};

				phyz::RigidBody* r1 = p.createRigidBody(geom1);
				phyz::RigidBody::PKey draw_p = r1->trackPoint(mthz::Vec3(0, 0, 0));
				r1->setCOMtoPosition(mthz::Vec3(0, 10, 0));
				//r1->orientation = mthz::Quaternion(3.1415926535 / 2.0, mthz::Vec3(0, 0, 1));
				r1->setAngVel(mthz::Vec3(0, 6, 1.4));
				//r1->vel = mthz::Vec3(10, 0, 0);
				bodies.push_back({ m1, r1 });
			}

			//unbalanced_barbell
			if (rndr::getKeyPressed(GLFW_KEY_U)) {
				double h1 = 2.8, w1 = 0.2, h2 = 0.5, w2 = 1, s = 0.5;
				std::vector<Mesh> m1 = { rect(0, 0, 0, w1, h1, w1), rect(0, -h1 * s / 2, 0, w2, h2 / 10, w2), rect(0, h1 * s / 2, 0, w2 * 3 / 4.0, h2, w2 * 3 / 4.0) };
				phyz::Geometry geom1 {
					phyz::Geometry::box(mthz::Vec3(-w1 / 2, -h1 / 2, -w1 / 2), w1, h1, w1),
					phyz::Geometry::box(mthz::Vec3(-w2 / 2, -h2 / 20 - h1 * s / 2, -w2 / 2), w2, h2 / 10, w2),
					phyz::Geometry::box(mthz::Vec3(-w2 * 3 / 8.0, -h2 / 2 + h1 * s / 2, -w2 * 3 / 8.0), w2 * 3 / 4.0, h2, w2 * 3 / 4.0)
				};
				phyz::RigidBody* r1 = p.createRigidBody(geom1);
				phyz::RigidBody::PKey draw_p = r1->trackPoint(mthz::Vec3(0, 0, 0));
				r1->setCOMtoPosition(mthz::Vec3(0, 10, 0));
				r1->setOrientation(mthz::Quaternion(0.67895851999693979, 0.0070540392161153693, 0.022756044727275559, 0.73378997750219643));
				r1->setAngVel(mthz::Vec3(0, 0.5, 10));
				//r1->vel = mthz::Vec3(10, 0, 0);
				bodies.push_back({ m1, r1 });
			}

			//chain
			if (rndr::getKeyPressed(GLFW_KEY_O)) {
				double end_y = 10;
				mthz::Vec3 link_dimensions(0.25, 0.85, 0.25);
				int n_links = 50;

				phyz::RigidBody* prev_link = nullptr;
				for (int i = 0; i < n_links; i++) {
					std::vector<Mesh> m1 = { rect(0, 0, 0, link_dimensions.x, link_dimensions.y, link_dimensions.z) };
					phyz::Geometry geom1 = phyz::Geometry::box(mthz::Vec3(-link_dimensions.x / 2.0, -link_dimensions.y / 2.0, -link_dimensions.z / 2.0), link_dimensions.x, link_dimensions.y, link_dimensions.z);
					phyz::RigidBody* r1 = p.createRigidBody(geom1);
					phyz::RigidBody::PKey draw_p1 = r1->trackPoint(mthz::Vec3(0, 0, 0));
					bodies.push_back({ m1, r1 });
					double y = end_y + link_dimensions.y * (n_links - i);
					r1->setCOMtoPosition(mthz::Vec3(0, y, 0));

					if (prev_link != nullptr) {
						p.addBallSocketConstraint(r1, prev_link, mthz::Vec3(0, link_dimensions.y / 2.0, 0), mthz::Vec3(0, -link_dimensions.y / 2.0, 0));
					}
					prev_link = r1;
				}
			}

			//cyllinder
			if (rndr::getKeyPressed(GLFW_KEY_H)) {
				phyz::Geometry geom = phyz::Geometry::cylinder(mthz::Vec3(0, 0, 0), 0.66, 0.3, 10);
				std::vector<Mesh> m = { fromGeometry(geom) };
				phyz::RigidBody* r = p.createRigidBody(geom, false, mthz::Vec3(0, 10, 0));
				bodies.push_back({ m, r });
			}

			//simple hinge
			if (rndr::getKeyPressed(GLFW_KEY_P)) {
				double end_y = 1;
				mthz::Vec3 box1_dimensions(1, 0.5, 1);
				mthz::Vec3 box2_dimensions(0.5, 0.5, 0.5);

				phyz::RigidBody* r1 = box(&bodies, &p, mthz::Vec3(0, end_y, 0), box1_dimensions);
				phyz::RigidBody* r2 = box(&bodies, &p, mthz::Vec3(0, end_y + box1_dimensions.y, 0), box2_dimensions);

				p.addHingeConstraint(r1, r2, mthz::Vec3(0, box1_dimensions.y / 2.0, 0), mthz::Vec3(0, -box2_dimensions.y / 2.0, 0), mthz::Vec3(0, 1, 0), mthz::Vec3(0, 1, 0));

			}

			//car
			if (rndr::getKeyPressed(GLFW_KEY_C)) {

				mthz::Vec3 pos(0, 10, 0);
				mthz::Vec3 chasis_dim(3, 0.25, 1.3);
				double wheel_width = 0.15;
				double wheel_radius = 0.75;
				double wheel_spacing = 0.66;

				phyz::Geometry wheel_c = phyz::Geometry::cylinder(mthz::Vec3(0, -wheel_width / 2.0, 0), wheel_radius, wheel_width);
				wheel_c = wheel_c.getRotated(mthz::Quaternion(3.1415926535 / 2.0, mthz::Vec3(1, 0, 0)), mthz::Vec3(0, 0, 0));
				std::vector<Mesh> wheel_m = { fromGeometry(wheel_c) };

				phyz::RigidBody* chasis = box(&bodies, &p, pos, chasis_dim);
				phyz::RigidBody* w1 = p.createRigidBody(wheel_c, false, chasis->getPos() + mthz::Vec3(chasis_dim.x * wheel_spacing / 2.0, 0, chasis_dim.z / 2.0));
				phyz::RigidBody* w2 = p.createRigidBody(wheel_c, false, chasis->getPos() + mthz::Vec3(-chasis_dim.x * wheel_spacing / 2.0, 0, chasis_dim.z / 2.0));
				phyz::RigidBody* w3 = p.createRigidBody(wheel_c, false, chasis->getPos() + mthz::Vec3(chasis_dim.x * wheel_spacing / 2.0, 0, -chasis_dim.z / 2.0));
				phyz::RigidBody* w4 = p.createRigidBody(wheel_c, false, chasis->getPos() + mthz::Vec3(-chasis_dim.x * wheel_spacing / 2.0, 0, -chasis_dim.z / 2.0));

				p.addHingeConstraint(chasis, w1, mthz::Vec3(chasis_dim.x * wheel_spacing / 2.0, 0, chasis_dim.z / 2.0), mthz::Vec3(0, 0, 0), mthz::Vec3(0, 0, 1), mthz::Vec3(0, 0, 1));
				p.addHingeConstraint(chasis, w2, mthz::Vec3(-chasis_dim.x * wheel_spacing / 2.0, 0, chasis_dim.z / 2.0), mthz::Vec3(0, 0, 0), mthz::Vec3(0, 0, 1), mthz::Vec3(0, 0, 1));
				p.addHingeConstraint(chasis, w3, mthz::Vec3(chasis_dim.x * wheel_spacing / 2.0, 0, -chasis_dim.z / 2.0), mthz::Vec3(0, 0, 0), mthz::Vec3(0, 0, -1), mthz::Vec3(0, 0, -1));
				p.addHingeConstraint(chasis, w4, mthz::Vec3(-chasis_dim.x * wheel_spacing / 2.0, 0, -chasis_dim.z / 2.0), mthz::Vec3(0, 0, 0), mthz::Vec3(0, 0, -1), mthz::Vec3(0, 0, -1));

				bodies.push_back({ wheel_m, w1 });
				bodies.push_back({ wheel_m, w2 });
				bodies.push_back({ wheel_m, w3 });
				bodies.push_back({ wheel_m, w4 });
			}

			//simple hinge
			if (rndr::getKeyPressed(GLFW_KEY_G)) {
				phyz::Geometry gear_g = phyz::Geometry::gear(mthz::Vec3(0, 0, 0), 0.5, 0.125, 0.1, 16);
				std::vector<Mesh> gear_m = { fromGeometry(gear_g) };

				phyz::RigidBody* r = p.createRigidBody(gear_g, false, mthz::Vec3(0, 10, 0));

				bodies.push_back({ gear_m, r });
			}

			if (rndr::getKeyDown(GLFW_KEY_8)) {
				/*p.setMotorPower(wheel_motor_1, 0.6);
				p.setMotorPower(wheel_motor_2, 0.6);
				p.setMotorPower(wheel_motor_3, -0.6);
				p.setMotorPower(wheel_motor_4, -0.6);*/
			}
			else if (rndr::getKeyDown(GLFW_KEY_7)) {
				/*p.setMotorPower(wheel_motor_1, -0.4);
				p.setMotorPower(wheel_motor_2, -0.4);
				p.setMotorPower(wheel_motor_3, 0.4);
				p.setMotorPower(wheel_motor_4, 0.4);*/
			}
			else {
			/*	p.setMotorPower(wheel_motor_1, 0);
				p.setMotorPower(wheel_motor_2, 0);
				p.setMotorPower(wheel_motor_3, 0);
				p.setMotorPower(wheel_motor_4, 0);*/
			}

			phyz_time += fElapsedTime;
			phyz_time = std::min<double>(phyz_time, 1.0 / 25.0);
			if (phyz_time > 1 / 30.0) {
				printf("n objects: %d\n", p.getNumBodies());
			}
			while (phyz_time > timestep) {
				phyz_time -= timestep;
				p.timeStep();
			}


			rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));

			for (const PhysBod& b : bodies) {
				for (const Mesh& m : b.meshes) {
					shader.setUniformMat4f("u_MVP", rndr::Mat4::proj(0.1, 50.0, 2.0, 2.0, 120.0) * rndr::Mat4::cam_view(pos, orient) * rndr::Mat4::model(b.r->getPos(), b.r->getOrientation()));
					shader.setUniform1i("u_Asleep", b.r->getAsleep());
					rndr::draw(*m.va, *m.ib, shader);
				}
			}
		}

		rndr::terminate();
		return 0;
	}
}