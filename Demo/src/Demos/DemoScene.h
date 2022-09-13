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
	std::function<DemoScene* (DemoManager*, DemoProperties)> createInstance;
};

class DemoManager {
public:
	void registerScene(const std::string& name, std::function<DemoScene* (DemoManager*, DemoProperties)> createInstance) { scene_suite.push_back({name, createInstance}); }
	void selectProperties();
	void selectDemoScene();
	void deselectCurrentScene();
private:
	DemoProperties properties;
	std::vector<SceneLauncher> scene_suite;
	DemoScene* current_scene = nullptr;
};

class DemoScene {
public:
	DemoScene(DemoManager* manager, DemoProperties properties);
	virtual ~DemoScene();
	virtual void run()=0;
protected:
	DemoManager* manager;
	DemoProperties properties;
};