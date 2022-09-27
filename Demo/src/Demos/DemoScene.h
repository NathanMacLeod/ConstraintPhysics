#pragma once

#include <functional>
#include <vector>
#include <string>

struct DemoProperties {
	int window_width;
	int window_height;
	int n_threads;
};

class DemoScene;
class DemoManager;

struct SceneLauncher {
	std::string name;
	std::string description;
	std::function<DemoScene* (DemoManager*, DemoProperties)> createInstance;
};

class DemoManager {
public:
	void registerScene(std::string name, std::string description, std::function<DemoScene* (DemoManager*, DemoProperties)> createInstance) { scene_suite.push_back({name, description, createInstance}); }
	void selectProperties();
	void selectDemoScene();
	void displaySceneControls();
	void runScene();
	void deselectCurrentScene();
private:
	DemoProperties properties;
	std::vector<SceneLauncher> scene_suite;
	DemoScene* current_scene = nullptr;
	std::string current_scene_name;
};

struct ControlDescription {
	std::string input_name;
	std::string input_action_description;
};

class DemoScene {
public:
	DemoScene(DemoManager* manager, DemoProperties properties);
	virtual ~DemoScene();
	virtual void run()=0;

	friend class DemoManager;
protected:
	DemoManager* manager;
	DemoProperties properties;

	virtual std::vector<ControlDescription> controls()=0;
};