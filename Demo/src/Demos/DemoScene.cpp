#include "DemoScene.h"
#include "../renderer/Renderer.h"
#include "../../../ConstraintPhysics/src/PhysicsEngine.h"

#include <cstdio>
#include <iostream>
#include <cassert>

static bool is_valid_int(const std::string& s) {
	if (s.empty()) return false;

	for (char c : s) {
		if (c < '0' || c > '9') {
			return false;
		}
	}

	return true;
}

static int read_int(int default_value) {
		std::string in;
		std::getline(std::cin, in);
		if (!is_valid_int(in)) {
			return default_value;
		}
		else {
			return atoi(in.c_str());
		}
}

void DemoManager::selectProperties() {
	properties = DemoProperties{ 960, 960, 6 };
	printf("Window Dimension X (leave blank for default %d): ", properties.window_width);
	properties.window_width = read_int(properties.window_width);
	printf("Window Dimension Y (leave blank for default %d): ", properties.window_height);
	properties.window_height = read_int(properties.window_height);
	printf("Thread pool size (leave blank for default %d, 0 for single threaded): ", properties.n_threads);
	properties.n_threads = read_int(properties.n_threads);
}

void DemoManager::selectDemoScene() {
	assert(current_scene == nullptr);

	printf("\nAvailable Demos:\n");
	printf("=================\n");
	for (int i = 0; i < scene_suite.size(); i++) {
		printf("(%d) %-26s %s\n", i, scene_suite[i].name.c_str(), scene_suite[i].description.c_str());
	}
	printf("=================\n\n");
	int selected_index = -1;
	do {
		printf("Select demo to run by number: ");
		int selected = read_int(-1);
		if (selected < 0 || selected >= scene_suite.size()) {
			printf("Invalid input\n");
		}
		else {
			selected_index = selected;
		}
	} while (selected_index == -1);

	SceneLauncher scene = scene_suite[selected_index];
	current_scene = scene.createInstance(this, properties);
	current_scene_name = scene.name;
}

void DemoManager::displaySceneControls() {
	assert(current_scene != nullptr);
	printf("\n%s Controls:\n", current_scene_name.c_str());
	printf("======================\n");
	for (ControlDescription c : current_scene->controls()) {
		printf("\t%s:\t%s\n", c.input_name.c_str(), c.input_action_description.c_str());
	}
	printf("\nPress ENTER to start\n");
	std::string unused;
	std::getline(std::cin, unused);
}

void DemoManager::runScene() {
	assert(current_scene != nullptr);
	current_scene->run();
}

void DemoManager::deselectCurrentScene() {
	assert(current_scene != nullptr);
	delete current_scene;
	current_scene = nullptr;
}

DemoScene::DemoScene(DemoManager* manager, DemoProperties properties) : manager(manager), properties(properties) {}

DemoScene::~DemoScene() {
	rndr::terminate();
	phyz::PhysicsEngine::disableMultithreading();
}