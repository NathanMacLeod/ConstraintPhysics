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
	properties = DemoProperties{ 1920, 1080, 6 };
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
	current_scene->parameters = current_scene->askParameters();
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

bool DemoScene::caseIndefferentStringEquals(const std::string& s1, const std::string& s2) {
	if (s1.size() != s2.size()) return false;

	for (int i = 0; i < s1.size(); i++) {
		if (toupper(s1[i]) != toupper(s2[i])) return false;
	}

	return true;
}

std::string DemoScene::pickParameterFromOptions(const std::string& prompt, const std::vector<std::string>& options) {
	auto response_valid = [=](std::string userin) -> bool {
		for (std::string opt : options) {
			if (caseIndefferentStringEquals(opt, userin)) return true;
		}
		return false;
	};

	std::string error_message = "Invalid option. Valid options are: ";
	for (int i = 0; i < options.size(); i++) {
		if (i + 1 < options.size()) error_message += std::format("{}, ", options[i]);
		else						error_message += std::format("{}: ", options[i]);
	}

	return askCustomParameterValue(prompt, response_valid, error_message);
}

std::string DemoScene::askCustomParameterValue(const std::string& prompt, std::function<bool(std::string)> response_valid, const std::string& incorrect_response_message) {
	printf("%s", prompt.c_str());

	std::string in;
	while (1) {
		std::getline(std::cin, in);
		if (response_valid(in)) return in;
		printf("%s", incorrect_response_message.c_str());
	}
}