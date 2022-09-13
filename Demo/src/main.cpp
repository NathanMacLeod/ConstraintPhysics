#define GLEW_STATIC
#include "renderer/Renderer.h"
#include "renderer/Mat4.h"
#include "Mesh.h"
#include "Demos/CarDemo.h"

int main() {

	DemoManager manager;
	manager.registerScene("Car", [](DemoManager* m, DemoProperties p) { return new CarDemo(m, p); });

	manager.selectProperties();
	manager.selectDemoScene();
}