#pragma once

#include <functional>
#include <vector>
#include <string>
#include <map>

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
	std::map<std::string, std::string> parameters;

	//helper functions for parameter questions
	static std::string pickParameterFromOptions(const std::string& prompt, const std::vector<std::string>& options);
	static std::string askCustomParameterValue(const std::string& propmt, std::function<bool(std::string)> response_valid, const std::string& incorrect_response_message);
	static bool caseIndefferentStringEquals(const std::string& s1, const std::string& s2);

	virtual std::map<std::string, std::string> askParameters() { return std::map<std::string, std::string>(); };
	virtual std::vector<ControlDescription> controls()=0;
};