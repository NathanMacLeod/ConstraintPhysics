#define GLEW_STATIC
#include "renderer/Renderer.h"
#include "renderer/Mat4.h"
#include "Mesh.h"
#include "Demos/CarDemo.h"
#include "Demos/ImageDemo.h"
#include "Demos/WreckingBallDemo.h"
#include "Demos/ColActionDemo.h"
#include "Demos/DzhanibekovDemo.h"
#include "Demos/ForkliftDemo.h"

int main() {

	DemoManager manager;
	manager.registerScene("Rock Paper Scissors", "Red converts green, green converts blue, and blue converts red.", [](DemoManager* m, DemoProperties p) { return new ColActionDemo(m, p); });
	manager.registerScene("Wrecking Ball", "Knock over a tower with a simulated wrecking ball.", [](DemoManager* m, DemoProperties p) { return new WreckingBallDemo(m, p); });
	manager.registerScene("Precomputed Simulation", "Precompute and view a simulation of colored falling blocks, whose final resting position will create an image.", [](DemoManager* m, DemoProperties p) { return new ImageDemo(m, p); });
	manager.registerScene("Simulated Car", "Drive a simulated car, featuring a working steering wheel and differential.", [](DemoManager* m, DemoProperties p) { return new CarDemo(m, p); });
	manager.registerScene("Dzhanibekov Effect", "Demonstration of dzhanibekov effect simulated in engine.", [](DemoManager* m, DemoProperties p) { return new DzhanibekovDemo(m, p); });
	manager.registerScene("Forklift", "Pick up and move pallets of boxes around with a simulated forklift.", [](DemoManager* m, DemoProperties p) { return new ForkliftDemo(m, p); });

	manager.selectProperties();
	while (1) {
		manager.selectDemoScene();
		manager.displaySceneControls();
		manager.runScene();
	}
}