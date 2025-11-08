#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

class ColActionDemo : public DemoScene {
public:
	ColActionDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~ColActionDemo() override {

	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"W, A, S, D", "Move the camera around when in free-look"},
			ControlDescription{"UP, DOWN, LEFT, RIGHT", "Rotate the camera"},
		};
	}

	static mthz::Vec3 randVec(double mag) {
		double theta = frand() * 2 * PI;
		double phi = frand() * PI;

		mthz::Quaternion direction = mthz::Quaternion(theta, mthz::Vec3(0, 1, 0)) * mthz::Quaternion(phi, mthz::Vec3(1, 0, 0));
		return direction.applyRotation(mthz::Vec3(0, mag, 0));
	}

	struct Agent {
		enum Color { Red, Green, Blue };

		rndr::VertexArray* va;
		rndr::IndexBuffer* ib;
		phyz::RigidBody* r;
		Color color;
	};

	Agent genAgent(phyz::ConvexUnionGeometry g, phyz::PhysicsEngine& p, Agent::Color color) {
		Mesh m = fromGeometry(g);
		phyz::RigidBody* r = p.createRigidBody(g);

		return Agent{ new rndr::VertexArray(m.vertices.data(), static_cast<uint32_t>(m.vertices.size()), Vertex::generateLayout()), new rndr::IndexBuffer(m.indices.data(), static_cast<uint32_t>(m.indices.size())), r, color };
	}

	void run() override {

		rndr::init(properties.window_width, properties.window_height, "Collision Action Demo");
		if (properties.n_threads != 0) {
			phyz::PhysicsEngine::enableMultithreading(properties.n_threads);
		}

		phyz::PhysicsEngine p;
		p.setSleepingEnabled(false);

		std::vector<Agent> agents;

		//***********
		//****BOX****
		//***********
		double box_size = 0.2;
		double wall_width = 0.25;

		phyz::Material bouncy{ phyz::CFM{phyz::USE_GLOBAL}, 1.0, 1.0, 0.0, 0.0 };

		phyz::ConvexUnionGeometry wall1 = phyz::ConvexUnionGeometry::box(mthz::Vec3(-box_size / 2.0, -box_size/2.0 - wall_width, -box_size / 2.0), box_size, wall_width, box_size, bouncy);
		phyz::ConvexUnionGeometry wall2 = phyz::ConvexUnionGeometry::box(mthz::Vec3(-box_size / 2.0, box_size/2.0, -box_size / 2.0), box_size, wall_width, box_size, bouncy);
		phyz::ConvexUnionGeometry wall3 = phyz::ConvexUnionGeometry::box(mthz::Vec3(-box_size / 2.0 - wall_width, -box_size/2.0, -box_size / 2.0), wall_width, box_size, box_size, bouncy);
		phyz::ConvexUnionGeometry wall4 = phyz::ConvexUnionGeometry::box(mthz::Vec3(box_size / 2.0, -box_size/2.0, -box_size / 2.0), wall_width, box_size, box_size, bouncy);
		phyz::ConvexUnionGeometry wall5 = phyz::ConvexUnionGeometry::box(mthz::Vec3(-box_size / 2.0, -box_size/2.0, -box_size / 2.0 - wall_width), box_size, box_size, wall_width, bouncy);
		phyz::ConvexUnionGeometry wall6 = phyz::ConvexUnionGeometry::box(mthz::Vec3(-box_size / 2.0, -box_size / 2.0, box_size / 2.0), box_size, box_size, wall_width, bouncy);

		p.createRigidBody(wall1, phyz::RigidBody::FIXED);
		p.createRigidBody(wall2, phyz::RigidBody::FIXED);
		p.createRigidBody(wall3, phyz::RigidBody::FIXED);
		p.createRigidBody(wall4, phyz::RigidBody::FIXED);
		p.createRigidBody(wall5, phyz::RigidBody::FIXED);
		p.createRigidBody(wall6, phyz::RigidBody::FIXED);

		double agent_size = 0.35;
		int agent_num = 6;
		phyz::ConvexUnionGeometry agent = phyz::ConvexUnionGeometry::box(mthz::Vec3(), agent_size, agent_size, agent_size, bouncy);

		mthz::Vec3 red_corner_pos(-box_size / 2.0, -box_size / 2, -box_size / 2);
		mthz::Vec3 green_corner_pos(-agent_num * agent_size, -agent_num * agent_size, -agent_num * agent_size);
		mthz::Vec3 blue_corner_pos(box_size / 2.0 - 2 * agent_num * agent_size, box_size / 2.0 - 2 * agent_num * agent_size, box_size / 2.0 - 2 * agent_num * agent_size);

		for (int i = 0; i < agent_num; i++) {
			for (int j = 0; j < agent_num; j++) {
				for (int k = 0; k < agent_num; k++) {
					phyz::ConvexUnionGeometry red_agent = agent.getTranslated(red_corner_pos + mthz::Vec3(i * 2 * agent_size, j * 2 * agent_size, k * 2 * agent_size));
					phyz::ConvexUnionGeometry green_agent = agent.getTranslated(green_corner_pos + mthz::Vec3(i * 2 * agent_size, j * 2 * agent_size, k * 2 * agent_size));
					phyz::ConvexUnionGeometry blue_agent = agent.getTranslated(blue_corner_pos + mthz::Vec3(i * 2 * agent_size, j * 2 * agent_size, k * 2 * agent_size));

					agents.push_back(genAgent(red_agent, p, Agent::Red));
					agents.push_back(genAgent(red_agent, p, Agent::Green));
					agents.push_back(genAgent(red_agent, p, Agent::Blue));
				}
			}
		}

		for (int i = 0; i < agents.size(); i++) {

			agents[i].r->setVel(randVec(2));

			for (int j = i+1; j < agents.size(); j++) {
				p.registerCollisionAction(phyz::CollisionTarget::with(agents[i].r), phyz::CollisionTarget::with(agents[j].r), [&, i, j](phyz::RigidBody* b1, phyz::RigidBody* b2, const std::vector<phyz::Manifold>& manifold) {
					switch (agents[i].color) {
					case Agent::Red:
						switch (agents[j].color) {
						case Agent::Green:
							agents[j].color = Agent::Red;
							break;
						case Agent::Blue:
							agents[i].color = Agent::Blue;
							break;
						}
						break;
					case Agent::Green:
						switch (agents[j].color) {
						case Agent::Red:
							agents[i].color = Agent::Red;
							break;
						case Agent::Blue:
							agents[j].color = Agent::Green;
							break;
						}
						break;
					case Agent::Blue:
						switch (agents[j].color) {
						case Agent::Red:
							agents[j].color = Agent::Blue;
							break;
						case Agent::Green:
							agents[i].color = Agent::Green;
							break;
						}
						break;
					}
				});
			}
		}

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/AgentColor.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Vec3 pos(0, 0, 10);
		mthz::Quaternion orient;
		double mv_speed = 2;
		double rot_speed = 1;

		double phyz_time = 0;
		double timestep = 1 / 90.0;
		p.setStep_time(timestep);
		p.setGravity(mthz::Vec3(0, 0, 0));

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

			if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
				manager->deselectCurrentScene();
				for (Agent a : agents) {
					delete a.va;
					delete a.ib;
				}
				return;
			}


			phyz_time += fElapsedTime;
			phyz_time = std::min<double>(phyz_time, 1.0 / 30.0);
			while (phyz_time > timestep) {
				phyz_time -= timestep;
				p.timeStep();
			}


			rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));

			for (Agent a : agents) {
				mthz::Vec3 cam_pos = pos;
				mthz::Quaternion cam_orient = orient;

				float aspect_ratio = (float)properties.window_height / properties.window_width;
				shader.setUniformMat4f("u_MVP", rndr::Mat4::proj(0.1f, 50.0f, 2.0f, 2.0f * aspect_ratio, 60.0f) * rndr::Mat4::cam_view(cam_pos, cam_orient) * rndr::Mat4::model(a.r->getPos(), a.r->getOrientation()));
				shader.setUniform1i("u_Color", a.color);

				rndr::draw(*a.va, *a.ib, shader);
			}
		}
	}
};