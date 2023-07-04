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
#include "Demos/TrebuchetDemo.h"
#include "Demos/AngularMomentumDemo.h"
#include "Demos/GyroscopeDemo.h"
#include "Demos/DebugDemo.h"
#include "Demos/VideoDemo.h"

int main() {

	DemoManager manager;
	manager.registerScene("Rock Paper Scissors", "Red converts green, green converts blue, and blue converts red.", [](DemoManager* m, DemoProperties p) { return new ColActionDemo(m, p); });
	manager.registerScene("Wrecking Ball", "Knock over a tower with a simulated wrecking ball.", [](DemoManager* m, DemoProperties p) { return new WreckingBallDemo(m, p); });
	manager.registerScene("Precomputed Simulation", "Precompute and view a simulation of colored falling blocks, whose final resting position will create an image.", [](DemoManager* m, DemoProperties p) { return new ImageDemo(m, p); });
	manager.registerScene("Simulated Car", "Drive a simulated car, featuring a working steering wheel and differential.", [](DemoManager* m, DemoProperties p) { return new CarDemo(m, p); });
	manager.registerScene("Dzhanibekov Effect", "Demonstration of dzhanibekov effect simulated in engine.", [](DemoManager* m, DemoProperties p) { return new DzhanibekovDemo(m, p); });
	manager.registerScene("Forklift", "Pick up and move pallets of boxes around with a simulated forklift.", [](DemoManager* m, DemoProperties p) { return new ForkliftDemo(m, p); });
	manager.registerScene("Trebuchet", "Launch projectiles with a simulated floating-arm trebuchet.", [](DemoManager* m, DemoProperties p) { return new TrebuchetDemo(m, p); });
	manager.registerScene("Angular Momentum Demo", "A common demonstration of the conservation of angular momentom, recreated in the engine", [](DemoManager* m, DemoProperties p) { return new AngularMomentumDemo(m, p); });
	manager.registerScene("Gyroscope", "A simulated gyroscope", [](DemoManager* m, DemoProperties p) { return new GyroscopeDemo(m, p); });
	manager.registerScene("Video Demo", ":o", [](DemoManager* m, DemoProperties p) { return new VideoDemo(m, p); });
	manager.registerScene("Debug demo", "for setting up debug scenes", [](DemoManager* m, DemoProperties p) { return new DebugDemo(m, p); });

	manager.selectProperties();
	while (1) {
		manager.selectDemoScene();
		manager.displaySceneControls();
		manager.runScene();
	}
}