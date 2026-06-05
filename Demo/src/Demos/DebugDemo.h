#pragma once
#include "DemoScene.h"
#include "../Mesh.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"
#include "Common.h"

class DebugDemo : public DemoScene {
public:
	DebugDemo(DemoManager* manager, DemoProperties properties) : DemoScene(manager, properties) {}

	~DebugDemo() override {

	}

	std::vector<ControlDescription> controls() override {
		return {
			ControlDescription{"W, A, S, D", "Move the camera around when in free-look"},
			ControlDescription{"UP, DOWN, LEFT, RIGHT", "Rotate the camera"},
			ControlDescription{"I. K", "Raise, Lower crane arm"},
			ControlDescription{"J, L", "Rotate crane counter-clockwise, clockwise"},
			ControlDescription{"R", "Reset tower"},
			ControlDescription{"ESC", "Return to main menu"},
		};
	}

	static void drawArc(mthz::Vec3 g1, mthz::Vec3 g2, mthz::Vec3 origin, double radius, Mesh* lines_out) {
		mthz::Vec3 cr = g1.cross(g2);
		double angle = acos(g1.dot(g2));
		mthz::Vec3 rot_axis = cr.normalize();

		const double MIN_ANG_STEP = PI / 60;
		int segment_count = int(angle / MIN_ANG_STEP) + 1;
		color gl_c = color{ 0.0f, 1.0f, 0.0f };
		int offset = lines_out->vertices.size();
		for (int i = 0; i < segment_count; i++) {
			double theta1 = i * MIN_ANG_STEP;
			double theta2 = i + 1 == segment_count ? angle : (i + 1) * MIN_ANG_STEP;

			mthz::Vec3 v1 = mthz::Quaternion(theta1, rot_axis).applyRotation(g1 * radius) + origin;
			mthz::Vec3 v2 = mthz::Quaternion(theta2, rot_axis).applyRotation(g1 * radius) + origin;

			lines_out->indices.push_back(offset++); lines_out->indices.push_back(offset++);
			lines_out->vertices.push_back(Vertex{ (float)v1.x, (float)v1.y, (float)v1.z, gl_c.r, gl_c.g, gl_c.b, gl_c.ambient_k, gl_c.diffuse_k, gl_c.specular_k, gl_c.specular_p, -1, 0.0f, 0.0f });
			lines_out->vertices.push_back(Vertex{ (float)v2.x, (float)v2.y, (float)v2.z, gl_c.r, gl_c.g, gl_c.b, gl_c.ambient_k, gl_c.diffuse_k, gl_c.specular_k, gl_c.specular_p, -1, 0.0f, 0.0f });
		}

		//draw lines connecting origin to the edges
		mthz::Vec3 v1 = g1 * radius + origin;
		mthz::Vec3 v2 = g2 * radius + origin;
		gl_c = color{ 1.0f, 1.0f, 1.0f };
		lines_out->indices.push_back(offset++); lines_out->indices.push_back(offset++);
		lines_out->vertices.push_back(Vertex{ (float)origin.x, (float)origin.y, (float)origin.z, gl_c.r, gl_c.g, gl_c.b, gl_c.ambient_k, gl_c.diffuse_k, gl_c.specular_k, gl_c.specular_p, -1, 0.0f, 0.0f });
		lines_out->vertices.push_back(Vertex{ (float)v1.x, (float)v1.y, (float)v1.z, gl_c.r, gl_c.g, gl_c.b, gl_c.ambient_k, gl_c.diffuse_k, gl_c.specular_k, gl_c.specular_p, -1, 0.0f, 0.0f });
		lines_out->indices.push_back(offset++); lines_out->indices.push_back(offset++);
		lines_out->vertices.push_back(Vertex{ (float)origin.x, (float)origin.y, (float)origin.z, gl_c.r, gl_c.g, gl_c.b, gl_c.ambient_k, gl_c.diffuse_k, gl_c.specular_k, gl_c.specular_p, -1, 0.0f, 0.0f });
		lines_out->vertices.push_back(Vertex{ (float)v2.x, (float)v2.y, (float)v2.z, gl_c.r, gl_c.g, gl_c.b, gl_c.ambient_k, gl_c.diffuse_k, gl_c.specular_k, gl_c.specular_p, -1, 0.0f, 0.0f });
	}

	static void createGaussMapVisualization(phyz::StaticMeshGeometry& geom, const phyz::StaticMeshFace& hit_mesh_face, mthz::Vec3 hit_pos, Mesh* lines_out, Mesh* triangles_out) {
		lines_out->indices.clear();
		lines_out->vertices.clear();
		triangles_out->indices.clear();
		triangles_out->vertices.clear();

		double tolerance = 0.02;
		//check if we have selected a vertex
		int selected_vertex_index = -1;
		for (int i = 0; i < 3; i++) {
			double dist = (hit_pos - geom.get_vertex(hit_mesh_face.vertex_indices[i]).p).mag();
			if (dist < tolerance) {
				selected_vertex_index = i;
				break;
			}
		}

		// check if we have selected an edge
		int selected_edge_index = -1;
		for (int i = 0; i < 3; i++) {
			phyz::StaticMeshHalfEdge e = geom.get_half_edge(hit_mesh_face.half_edge_indices[i]);
			mthz::Vec3 diff = hit_pos - geom.get_vertex(e.p1_index).p;
			mthz::Vec3 edge_dir = (geom.get_vertex(e.p2_index).p - geom.get_vertex(e.p1_index).p).normalize();
			double dist = (diff - edge_dir * edge_dir.dot(diff)).mag();
			if (dist < tolerance) {
				selected_edge_index = i;
				break;
			}
		}
		if (selected_vertex_index != -1) {
			//printf("index: %d\n", hit_mesh_face.vertex_indices[selected_vertex_index]);
			phyz::StaticMeshVertex v = geom.get_vertex(hit_mesh_face.vertex_indices[selected_vertex_index]);
			mthz::Vec3 p = v.p;
			float delta = 0.0001f;

			color c = color{ 1.0f, 1.0f, 0.0f };
			*triangles_out = fromGeometry(phyz::ConvexUnionGeometry::sphere(p, 0.005), c);

			//draw gauss arcs
			for (int i = 0; i < v.valid_normal_gauss_map.size(); i++) {
				mthz::Vec3 g1 = v.valid_normal_gauss_map[i];
				mthz::Vec3 g2 = v.valid_normal_gauss_map[(i + 1) % v.valid_normal_gauss_map.size()];
				drawArc(g1, g2, p, 0.25, lines_out);
			}
			//lines_out->indices.push_back(offset++); lines_out->indices.push_back(offset++);
			//lines_out->vertices.push_back(Vertex{ (float)p.x, (float)p.y, (float)p.z, c.r, c.g, c.b, c.ambient_k, c.diffuse_k, c.specular_k, c.specular_p, -1, 0.0f, 0.0f });
			//lines_out->vertices.push_back(Vertex{ (float)p.x + delta, (float)p.y + delta, (float)p.z + delta, c.r, c.g, c.b, c.ambient_k, c.diffuse_k, c.specular_k, c.specular_p, -1, 0.0f, 0.0f });
		}

		else if (selected_edge_index != -1) {
			phyz::StaticMeshHalfEdge e = geom.get_half_edge(hit_mesh_face.half_edge_indices[selected_edge_index]);
			mthz::Vec3 p1 = geom.get_vertex(e.p1_index).p;
			mthz::Vec3 p2 = geom.get_vertex(e.p2_index).p;

			unsigned int offset = lines_out->vertices.size();
			lines_out->indices.push_back(offset++); lines_out->indices.push_back(offset++);
			color c = color{ 1.0f, 1.0f, 0.0f };
			lines_out->vertices.push_back(Vertex{ (float)p1.x, (float)p1.y, (float)p1.z, c.r, c.g, c.b, c.ambient_k, c.diffuse_k, c.specular_k, c.specular_p, -1, 0.0f, 0.0f });
			lines_out->vertices.push_back(Vertex{ (float)p2.x, (float)p2.y, (float)p2.z, c.r, c.g, c.b, c.ambient_k, c.diffuse_k, c.specular_k, c.specular_p, -1, 0.0f, 0.0f });

			//printf("this face index: %d\n", hit_mesh_face.self_index);
			//printf("this_face_norm: %f %f %f\n", hit_mesh_face.normal.x, hit_mesh_face.normal.y, hit_mesh_face.normal.z);
			//printf("twin index: %d\n", e.twin_index);
			if (e.twin_index != -1) {
				phyz::StaticMeshFace twin_face = geom.get_triangle(geom.get_half_edge(e.twin_index).triangle_index);
				//printf("twin_face_norm: %f %f %f\n", twin_face.normal.x, twin_face.normal.y, twin_face.normal.z);
			}

			if (!e.has_gauss_arc) { return; }
			//draw the gauss arc
			drawArc(e.gauss_arc_g1, e.gauss_arc_g2, (p1 + p2) / 2.0, 0.25, lines_out);
		}
		else {
			// highlight the selected triangle
			mthz::Vec3 centroid = (geom.get_vertex(hit_mesh_face.vertex_indices[0]).p + geom.get_vertex(hit_mesh_face.vertex_indices[1]).p + geom.get_vertex(hit_mesh_face.vertex_indices[2]).p) / 3.0;

			lines_out->indices = { 0, 1, 1, 2, 2, 0 };
			
			color c = color{ 1.0f, 1.0f, 0.0f };
			for (int i = 0; i < 3; i++) {
				mthz::Vec3 p = geom.get_vertex(hit_mesh_face.vertex_indices[i]).p;
				lines_out->vertices.push_back(Vertex{ (float)p.x, (float)p.y, (float)p.z, c.r, c.g, c.b, c.ambient_k, c.diffuse_k, c.specular_k, c.specular_p, -1, 0.0f, 0.0f });
			}

			// draw the triangles normal
			double normal_line_length = 0.33;
			mthz::Vec3 normal_tip = centroid + hit_mesh_face.normal * normal_line_length;

			unsigned int offset = lines_out->vertices.size();
			lines_out->indices.push_back(offset++); lines_out->indices.push_back(offset++);
			lines_out->vertices.push_back(Vertex{ (float)centroid.x, (float)centroid.y, (float)centroid.z, c.r, c.g, c.b, c.ambient_k, c.diffuse_k, c.specular_k, c.specular_p, -1, 0.0f, 0.0f });
			lines_out->vertices.push_back(Vertex{ (float)normal_tip.x, (float)normal_tip.y, (float)normal_tip.z, c.r, c.g, c.b, c.ambient_k, c.diffuse_k, c.specular_k, c.specular_p, -1, 0.0f, 0.0f });
		}

		

		// draw the gauss map
		//double gauss_map_radius = 0.25;
		//color gl_c = color{ 0.0f, 1.0f, 1.0f };
		//const std::vector<mthz::Vec3>& gauss_region = hit_mesh_face->gauss_region;
		//if (gauss_region.size() >= 2) {
		//	bool gauss_region_is_arc = gauss_region.size() == 2;
		//	for (int i = 0; i < gauss_region.size(); i++) {
		//		if (gauss_region_is_arc && i + 1 == gauss_region.size()) { break; }
		//		else {
		//			mthz::Vec3 g1 = gauss_region[i];
		//			mthz::Vec3 g2 = gauss_region[(i + 1) % gauss_region.size()];

		//			mthz::Vec3 offset_g1 = centroid + g1 * gauss_map_radius;
		//			mthz::Vec3 offset_g2 = centroid + g2 * gauss_map_radius;

		//			lines_out->indices.push_back(offset++); lines_out->indices.push_back(offset++);
		//			lines_out->vertices.push_back(Vertex{ (float)offset_g1.x, (float)offset_g1.y, (float)offset_g1.z, gl_c.r, gl_c.g, gl_c.b, gl_c.ambient_k, gl_c.diffuse_k, gl_c.specular_k, gl_c.specular_p, -1, 0.0f, 0.0f });
		//			lines_out->vertices.push_back(Vertex{ (float)offset_g2.x, (float)offset_g2.y, (float)offset_g2.z, gl_c.r, gl_c.g, gl_c.b, gl_c.ambient_k, gl_c.diffuse_k, gl_c.specular_k, gl_c.specular_p, -1, 0.0f, 0.0f });
		//		}
		//	}
		//}
	}

	void run() override {
		rndr::init(properties.window_width, properties.window_height, "Wrecking Ball Demo");

		phyz::PhysicsEngine p;
		if (properties.n_threads != 0) {
			p.enableMultithreading(properties.n_threads);
		}

		bool lock_cam = true;

		std::vector<PhysBod> bodies;

		//mthz::Vec3 center = mthz::Vec3(0, -2, 0);
		//int grid_count = 360;
		//double grid_size = 0.5;
		//phyz::Mesh marble_track = phyz::readOBJ("resources/mesh/marble_track.obj", 0.1);
		//phyz::Mesh marble_screw = phyz::readOBJ("resources/mesh/marble_screw.obj", 0.1);
		//phyz::MeshInput marble_track_input = phyz::generateMeshInputFromMesh(marble_track, center);
		//phyz::MeshInput marble_screw_input = phyz::generateMeshInputFromMesh(marble_screw, center);
		//phyz::MeshInput grid = phyz::generateGridMeshInput(grid_count, grid_count, grid_size, center + mthz::Vec3(-grid_count * grid_size / 2.0, 0, -grid_count * grid_size / 2.0), phyz::Material::ice());//phyz::generateRadialMeshInput(center, 8, 100, 1);


		rndr::init(properties.window_width, properties.window_height, "Car Demo");

		//************************
		//*******BASE PLATE*******
		//************************
		double s = 100;
		phyz::ConvexUnionGeometry geom2 = phyz::ConvexUnionGeometry::box(mthz::Vec3(-s / 2, -2, -s / 2), s, 2, s);
		Mesh m2 = fromGeometry(geom2);
		phyz::RigidBody* r2 = p.createRigidBody(geom2, phyz::RigidBody::FIXED);
		phyz::RigidBody::PKey draw_p = r2->trackPoint(mthz::Vec3(0, -2, 0));
		bodies.push_back({ m2, r2 });

		//*****************
		//****OBSTACLES****
		//*****************
		double radius = 3.5;
		mthz::Vec3 block_dim(1, 1, 2);
		//createCircularTower(&p, &bodies, block_dim, radius, 10, mthz::Vec3(0, 0, -22), 40);
		//createRagdoll(&p, &bodies, mthz::Vec3(0, 5, -22), 0.5);

		/*for (mthz::Vec3& v : grid.points) {
			v.y += 0.01 * 2 * (0.5 - frand());
		}
		for (phyz::TriIndices& t : grid.triangle_indices) {
			t.material = phyz::Material::ice();
		}*/

		bool gauss_map_highlighted = false;
		Mesh highlighted_gauss_map_lines;
		Mesh highlighted_gauss_map_triangles;

		phyz::Mesh bunny_mesh = phyz::readOBJ("resources/mesh/bunny.obj", 50.0);
		//phyz::Mesh bunny_mesh = phyz::readOBJ("resources/mesh/weird_cases.obj", 1.0);
		phyz::MeshInput bunny_mesh_input = phyz::generateMeshInputFromMesh(bunny_mesh, mthz::Vec3(20, 0, 0));
		phyz::RigidBody* bunny_mesh_r = p.createRigidBody(bunny_mesh_input);
		bodies.push_back({ fromStaticMeshInput(bunny_mesh_input, color{ 0.8f, 1.0f, 1.0f, 0.5f, 0.5f, 0.63f, 51.2f }), bunny_mesh_r });

		phyz::Mesh stress_test_cases_mesh = phyz::readOBJ("resources/mesh/weird_cases.obj", 1.0);
		phyz::MeshInput stress_test_cases_mesh_input = phyz::generateMeshInputFromMesh(stress_test_cases_mesh, mthz::Vec3(0, 0, 0));
		phyz::RigidBody* stress_test_cases_mesh_r = p.createRigidBody(stress_test_cases_mesh_input);
		bodies.push_back({ fromStaticMeshInput(stress_test_cases_mesh_input, color{ 0.8f, 1.0f, 1.0f, 0.5f, 0.5f, 0.63f, 51.2f }), stress_test_cases_mesh_r });



		phyz::StaticMeshGeometry debug_triangle = phyz::StaticMeshGeometry({ {phyz::StaticMeshVertex{mthz::Vec3(-1.852056,5.8148808,2.679084),0,{{-0.18252588896589894,-0.0495553526478906,0.9819514075967056},{0.25626361951928267,0.21967775768334097,0.9413132529026115},{0.22637476228717268,0.2937103556041531,0.9287026940901528},{-0.17152432073827417,0.41471967390272857,0.89363695059755},{-0.2490493918615823,0.30071020441214585,0.9206235785465097}}},phyz::StaticMeshVertex{mthz::Vec3(-1.7490708,6.118421400000001,2.5579836),0,{}},phyz::StaticMeshVertex{mthz::Vec3(-2.1558894000000004,5.9885969999999995,2.540148),0,{}}} }, { {phyz::StaticMeshHalfEdge{0,0,0,0,0,0,mthz::Vec3(0.9382139308016109,-0.20796725449296471,0.27659400049250626),0,true,mthz::Vec3(-0.1715243207382742,0.41471967390272857,0.8936369505975501),mthz::Vec3(0.22637476228717274,0.29371035560415304,0.9287026940901528)},phyz::StaticMeshHalfEdge{0,0,0,0,0,0,mthz::Vec3(-0.2541370754085847,0.85775448026749,-0.44684628059749976),0,true,mthz::Vec3(-0.1715243207382742,0.41471967390272857,0.8936369505975501),mthz::Vec3(-0.18271394311417108,0.4526506544976601,0.872767437508132)},phyz::StaticMeshHalfEdge{0,0,0,0,0,0,mthz::Vec3(-0.5652760128979365,-0.7843369300699666,0.2554967893548416),0,true,mthz::Vec3(-0.1715243207382742,0.41471967390272857,0.8936369505975501),mthz::Vec3(-0.2490493918615823,0.30071020441214585,0.9206235785465097)}} });
		phyz::RigidBody* that_one_triangle_r = p.createRigidBody(debug_triangle);
		bodies.push_back({ fromStaticMeshGeometry(debug_triangle, color{ 0.8f, 1.0f, 1.0f, 0.5f, 0.5f, 0.63f, 51.2f }), that_one_triangle_r });

		bool something_hovered = false;
		mthz::Vec3 hover_pos;
		Mesh hover_ball = fromGeometry(phyz::ConvexUnionGeometry::sphere(mthz::Vec3(), 0.01), { 1.0, 1.0, 1.0 });

		Mesh contact_ball_mesh = fromGeometry(phyz::ConvexUnionGeometry::merge(phyz::ConvexUnionGeometry::sphere(mthz::Vec3(), 0.03), phyz::ConvexUnionGeometry::cylinder(mthz::Vec3(), 0.02, 0.1)), {1.0, 0, 0});

		bool color_by_manifold = false;
		struct Contact {
			mthz::Vec3 p;
			mthz::Vec3 n;
			color c;
		};
		std::vector<Contact> all_contact_points;

		p.registerCollisionAction(phyz::CollisionTarget::all(), phyz::CollisionTarget::all(), [&](phyz::RigidBody* b1, phyz::RigidBody* b2,
			const std::vector<phyz::Manifold>& manifold) {
				for (const phyz::Manifold& m : manifold) {
					// psuedo random color for the whole manifold
					uint64_t uid_sum = 0;
					color manifold_color;
					if (color_by_manifold) {
						for (phyz::ContactP p : m.points) {
							// generate a psuedo random color from the magicID- should make a clear visualization a contact is preserved by its magicID
							uint64_t uid = std::hash<phyz::MagicID>{}(p.magicID);
							uid_sum += uid;
						}
					}
					uid_sum = std::hash<uint64_t>{}(uid_sum);
					manifold_color = {
						((uid_sum & 0x0000FF) >> 0) / 255.0f,
						((uid_sum & 0x00FF00) >> 8) / 255.0f,
						((uid_sum & 0xFF0000) >> 16) / 255.0f
					};

					mthz::Vec3 avg;

					for (phyz::ContactP p : m.points) {
						avg += p.pos;

						color c;
						if (color_by_manifold) {
							c = manifold_color;
						}
						else {
							// generate a psuedo random color from the magicID- should make a clear visualization a contact is preserved by its magicID
							uint64_t uid = std::hash<phyz::MagicID>{}(p.magicID);
							c = {
								((uid & 0x0000FF) >> 0) / 255.0f,
								((uid & 0x00FF00) >> 8) / 255.0f,
								((uid & 0xFF0000) >> 16) / 255.0f
							};
						}
						all_contact_points.push_back({ p.pos, m.normal, c});
					}

					avg /= static_cast<double>(m.points.size());
					mthz::Vec3 u, w;
					m.normal.getPerpendicularBasis(&u, &w);
					all_contact_points.push_back({ avg, u, color{1.0f, 0.0f, 0.0f} });
					all_contact_points.push_back({ avg, w, color{0.0f, 0.0f, 1.0f} });
				}
			}
		);

		mthz::Vec3 pos(0, 2, 0);

		rndr::BatchArray batch_array(Vertex::generateLayout(), 1024 * 1024);
		rndr::Shader shader("resources/shaders/Basic.shader");
		rndr::Shader line_shader("resources/shaders/LineDraw.shader");
		shader.bind();

		float t = 0;
		float fElapsedTime;

		mthz::Quaternion orient;
		double mv_speed = 6;
		//double rot_speed = 1;

		double phyz_time = 0;
		double timestep = 1 / 60.0;
		p.setPGSIterations(4, 1, 1);
		p.setSubstepCount(8);
		p.setStep_time(timestep);
		p.setGravity(mthz::Vec3(0, -6.0, 0));


		bool object_highlighted = false;
		unsigned int highlighted_object_id = -1;

		bool single_step_mode = false;

		bool paused = true;

		phyz::RigidBody* grabbed_object = nullptr;                 // null -> no object grabbed
		mthz::Vec3 grabbed_object_grabbed_point_local_coordinates; // what point on the objects surface we have grabbed, in local coords of the object
		double grab_distance = -1;                                      // what distance awway from the camera the grabbed point is. while a grab object is held, we keep this constant when moving the camera.
		const double GRAB_PULL_STRENGTH = 1000;

		rndr::lockMouse();
		double mouse_sensitivity = 0.0015;
		rndr::MousePos mouse_position = rndr::getMousePosition();

		int tick_count = 0;
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

			// mouse controlled camera movement
			rndr::MousePos new_mouse = rndr::getMousePosition();
			double mouse_delta_x = new_mouse.x - mouse_position.x;
			double mouse_delta_y = new_mouse.y - mouse_position.y;
			mouse_position = new_mouse;

			orient = orient * mthz::Quaternion(mouse_sensitivity * mouse_delta_y, mthz::Vec3(1, 0, 0));
			orient = mthz::Quaternion(-mouse_sensitivity * mouse_delta_x, mthz::Vec3(0, 1, 0)) * orient;

			/*if (rndr::getKeyDown(GLFW_KEY_UP)) {
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
			}*/

			if (rndr::getKeyPressed(GLFW_KEY_B)) {
				single_step_mode = !single_step_mode;
			}

			if (rndr::getKeyPressed(GLFW_KEY_G)) {
				double block_size = 1.0;
				double block_speed = 2.5;

				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::ConvexUnionGeometry block = phyz::ConvexUnionGeometry::cylinder(pos, 0.3, 3);
				phyz::RigidBody* block_r = p.createRigidBody(block);

				block_r->setVel(camera_dir * block_speed);
				bodies.push_back({ fromGeometry(block), block_r });

			}

			if (rndr::getKeyPressed(GLFW_KEY_K)) {
				double block_size = 1.0;
				double block_speed = 2.5;

				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::ConvexUnionGeometry block = phyz::ConvexUnionGeometry::sphere(pos, 0.36);
				phyz::RigidBody* block_r = p.createRigidBody(block);

				block_r->setVel(camera_dir * block_speed);
				bodies.push_back({ fromGeometry(block), block_r });

			}

			if (rndr::getKeyPressed(GLFW_KEY_H)) {
				double block_size = 1.0;
				double block_speed = 8.5;

				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				phyz::ConvexUnionGeometry block = phyz::ConvexUnionGeometry::box(pos, 1, 1, 1, phyz::Material::ice());
				phyz::RigidBody* block_r = p.createRigidBody(block);

				block_r->setVel(mthz::Vec3(block_speed, 0, 0));

				bodies.push_back({ fromGeometry(block), block_r });
			}

			// highlighting an object and printing its id
			//if (rndr::getMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT)) {
			//	mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
			//	phyz::RayHitInfo hit_info = p.raycastFirstIntersection(pos, camera_dir);

			//	if (hit_info.did_hit) {
			//		object_highlighted = true;
			//		highlighted_object_id = hit_info.hit_object->getID();
			//		printf("selected object id: %u\n", highlighted_object_id);
			//		//hit_info.hit_object->applyImpulse(camera_dir * 1, hit_info.hit_position);
			//	}
			//	else {
			//		object_highlighted = false;
			//	}
			//}

			// dragging a moving object
			//if (rndr::getMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT)) {
			//	mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
			//	phyz::RayHitInfo hit_info = p.raycastFirstIntersection(pos, camera_dir);

			//	if (hit_info.did_hit && hit_info.hit_object->getMovementType() == phyz::RigidBody::DYNAMIC) {
			//		grabbed_object = hit_info.hit_object;
			//		grabbed_object_grabbed_point_local_coordinates = grabbed_object->getWorldPosInLocalCoords(hit_info.hit_position);
			//		grab_distance = hit_info.hit_distance;
			//	}
			//}
			//else if (rndr::getMouseButtonReleased(GLFW_MOUSE_BUTTON_LEFT)) {
			//	grabbed_object = nullptr;
			//}

			//if (rndr::getKeyPressed(GLFW_KEY_F)) {
			//	mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
			//	phyz::RayHitInfo hit_info = p.raycastFirstIntersection(pos, camera_dir);

			//	if (hit_info.did_hit) {
			//		//object_highlighted = true;
			//		//highlighted_object_id = hit_info.hit_object->getID();
			//		//printf("selected object id: %u\n", highlighted_object_id);
			//		hit_info.hit_object->applyImpulse(camera_dir * 1, hit_info.hit_position);
			//	}
			//}

			mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
			phyz::RayHitInfo hit_info = p.raycastFirstIntersection(pos, camera_dir);
			if (hit_info.did_hit) {
				something_hovered = true;
				hover_pos = hit_info.hit_position;
			}

			// draw gauss map of a feature on a static mesh.
			if (rndr::getMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT)) {
				// visualization is only for static mesh.
				if (!hit_info.did_hit || hit_info.hit_object->getMovementType() != phyz::RigidBody::FIXED || hit_info.hit_object->getGeometryType() != phyz::RigidBody::STATIC_MESH) {
					gauss_map_highlighted = false;
				}
				else {
					phyz::RigidBody* r = hit_info.hit_object;
					phyz::StaticMeshGeometry& body_mesh = r->mesh;
					uint32_t hit_face_index = body_mesh.testRayIntersection(pos, camera_dir).hit_triangle_inedex;
					phyz::StaticMeshFace hit_mesh_face = body_mesh.get_triangle(hit_face_index);
					//
					//
					//phyz::StaticMeshFace* hit_mesh_face = nullptr;
					//// copy-pasta from StaticMeshGeometry::testRayIntersection
					//// get the triangle that we hit. (very ugly use of internal methods, but since it's only needed for this debug visualization don't see a need to add a proper interface).
					//std::vector<unsigned int> hit_candidates = body_mesh.aabb_tree.raycastHitCandidates(pos, camera_dir);

					//for (unsigned int i : hit_candidates) {
					//	phyz::StaticMeshFace& tri = body_mesh.triangles[i];
					//	if (abs(tri.normal.dot(camera_dir)) < 0.0000000001) {
					//		continue;
					//	}

					//	//calculate dist where ray intersects the plane the triangle sits on
					//	double t = -(pos - body_mesh.get_vertex(tri.vertex_indices[1]).p).dot(tri.normal) / camera_dir.dot(tri.normal);
					//	if (t == hit_info.hit_distance) {
					//		hit_mesh_face = &tri;
					//		break;
					//	}
					//}

					//assert(hit_mesh_face != nullptr);
					////printf("wow!: <%f %f %f>\n", hit_mesh_face->normal.x, hit_mesh_face->normal.y, hit_mesh_face->normal.z);
					createGaussMapVisualization(body_mesh, hit_mesh_face, hit_info.hit_position, &highlighted_gauss_map_lines, &highlighted_gauss_map_triangles);
					gauss_map_highlighted = true;
				}
			}

			if (rndr::getKeyPressed(GLFW_KEY_P)) {
				paused = !paused;
			}

			if (rndr::getKeyPressed(GLFW_KEY_T)) {
				all_contact_points.clear();
				phyz_time += timestep;
				printf("%d\n", tick_count);
			}

			t += fElapsedTime;

			if (rndr::getKeyPressed(GLFW_KEY_ESCAPE)) {
				manager->deselectCurrentScene();
				return;
			}

			//if (tick_count == 69) {
			//	paused = true;
			//}

			// dragging grabbed object:
			if (grabbed_object != nullptr) {
				mthz::Vec3 camera_dir = orient.applyRotation(mthz::Vec3(0, 0, -1));
				mthz::Vec3 desired_position = pos + grab_distance * camera_dir;

				mthz::Vec3 current_grabbed_position = grabbed_object->getLocalPosInWorldCoords(grabbed_object_grabbed_point_local_coordinates);

				// generate a impulse that is scales linearly with the timestep, the distance between the desired position and current position, and the objects mass.
				mthz::Vec3 pull_force = (desired_position - current_grabbed_position) * GRAB_PULL_STRENGTH * grabbed_object->getMass() * fElapsedTime;
				grabbed_object->applyImpulse(pull_force, current_grabbed_position);
			}

			if (!paused) {
				phyz_time += fElapsedTime;
				phyz_time = std::min<double>(phyz_time, 1.0);
			}
			while (phyz_time > timestep) {
				all_contact_points.clear();
				phyz_time -= timestep;
				p.timeStep();
				tick_count++;
			}

			rndr::clear(rndr::color(0.0f, 0.0f, 0.0f));
			batch_array.flush();

			mthz::Vec3 cam_pos = pos;
			mthz::Quaternion cam_orient = orient;

			mthz::Vec3 pointlight_pos(0.0, 25.0, 0.0);
			mthz::Vec3 trnsfm_light_pos = cam_orient.conjugate().applyRotation(pointlight_pos - cam_pos);

			float aspect_ratio = (float)properties.window_height / properties.window_width;
			rndr::Mat4 proj_mat = rndr::Mat4::proj(0.1f, 500.0f, 2.0f, 2.0f * aspect_ratio, 60.0f);
			shader.bind();
			glEnable(GL_POLYGON_OFFSET_FILL);
			glPolygonOffset(1.0f, 1.0f); // offset to help the highlighted object.
			shader.setUniformMat4f("u_P", proj_mat);
			shader.setUniform3f("u_ambient_light", 0.4f, 0.4f, 0.4f);
			shader.setUniform3f("u_pointlight_pos", static_cast<float>(trnsfm_light_pos.x), static_cast<float>(trnsfm_light_pos.y), static_cast<float>(trnsfm_light_pos.z));
			shader.setUniform3f("u_pointlight_col", 0.6f, 0.6f, 0.6f);
			shader.setUniform1i("u_Asleep", false);

			for (const PhysBod& b : bodies) {

				bool is_highlighted = object_highlighted && b.r->getID() ==  highlighted_object_id;
				color override_color = is_highlighted ? color{ 1.0, 1.0, 0.0 } : color{ 1.0, 0.0, 0.0 };
				bool color_overriden = is_highlighted || b.r->getAsleep();

				Mesh transformed_mesh = getTransformed(b.mesh, b.r->getPos(), b.r->getOrientation(), cam_pos, cam_orient, color_overriden, override_color);

				if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), static_cast<uint32_t>(transformed_mesh.vertices.size()), transformed_mesh.indices);
			}

			if (something_hovered) {
				Mesh transformed_mesh = getTransformed(hover_ball, hover_pos, mthz::Quaternion(), cam_pos, cam_orient);

				if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() <  transformed_mesh.indices.size()) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), static_cast<uint32_t>(transformed_mesh.vertices.size()), transformed_mesh.indices);
			}

			for (Contact c : all_contact_points) {
				mthz::Quaternion rot;
				double d = mthz::Vec3(0, 1, 0).dot(c.n);
				if (d < -0.99999) {
					rot = mthz::Quaternion(PI, mthz::Vec3(0, 0, 1));
				}
				else if (d < 0.99999) {
					mthz::Vec3 axis = mthz::Vec3(0, 1, 0).cross(c.n).normalize();
					double ang = acos(d);
					rot = mthz::Quaternion(ang, axis);
				}

				Mesh transformed_mesh = getTransformed(contact_ball_mesh, c.p, rot, cam_pos, cam_orient, true, c.c);

				if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), static_cast<uint32_t>(transformed_mesh.vertices.size()), transformed_mesh.indices);
			}

			rndr::draw(batch_array, shader);
			batch_array.flush();


			if (gauss_map_highlighted) {
				Mesh transformed_mesh = getTransformed(highlighted_gauss_map_triangles, mthz::Vec3(), mthz::Quaternion(), cam_pos, cam_orient);
				if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), static_cast<uint32_t>(transformed_mesh.vertices.size()), transformed_mesh.indices);
			}

			rndr::draw(batch_array, shader);
			batch_array.flush();

			line_shader.bind();
			line_shader.setUniformMat4f("u_P", proj_mat);
			glDisable(GL_POLYGON_OFFSET_FILL);
			glLineWidth(2.0f);
			

			if (gauss_map_highlighted) {
				Mesh transformed_mesh = getTransformed(highlighted_gauss_map_lines, mthz::Vec3(), mthz::Quaternion(), cam_pos, cam_orient);
				if (batch_array.remainingVertexCapacity() <= transformed_mesh.vertices.size() || batch_array.remainingIndexCapacity() < transformed_mesh.indices.size()) {
					rndr::draw(batch_array, shader);
					batch_array.flush();
				}
				batch_array.push(transformed_mesh.vertices.data(), static_cast<uint32_t>(transformed_mesh.vertices.size()), transformed_mesh.indices);
			}

			rndr::drawLines(batch_array, line_shader);
			batch_array.flush();
		}
	}
};