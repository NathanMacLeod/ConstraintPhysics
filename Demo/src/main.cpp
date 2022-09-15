#define GLEW_STATIC
#include "renderer/Renderer.h"
#include "renderer/Mat4.h"
#include "Mesh.h"
#include "Demos/CarDemo.h"
#include "Demos/ImageDemo.h"

int main() {

	DemoManager manager;
	manager.registerScene("Car", [](DemoManager* m, DemoProperties p) { return new CarDemo(m, p); });
	manager.registerScene("Image", [](DemoManager* m, DemoProperties p) { return new ImageDemo(m, p); });

	manager.selectProperties();
	manager.selectDemoScene();
	manager.displaySceneControls();
	manager.runScene();
}