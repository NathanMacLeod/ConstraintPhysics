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
#include "Demos/LDLSolverDemos.h"
#include "Demos/NewtonsCradleDemo.h"
#include "Demos/MarbleMachineDemo.h"
#include "Demos/StanfordArmadillo.h"
#include "Demos/RaycastCarDemo.h"

int main() {

	DemoManager manager;
	//manager.registerScene("Rock Paper Scissors", "Red converts green, green converts blue, and blue converts red.", [](DemoManager* m, DemoProperties p) { return new ColActionDemo(m, p); });
	manager.registerScene("Wrecking Ball", "Knock over a tower with a simulated wrecking ball.", [](DemoManager* m, DemoProperties p) { return new WreckingBallDemo(m, p); });
	manager.registerScene("Precomputed Simulation", "Precompute and view a simulation of colored falling blocks, whose final resting position will create an image.", [](DemoManager* m, DemoProperties p) { return new ImageDemo(m, p); });
	manager.registerScene("Simulated Car", "Drive a simulated car, featuring a working steering wheel and differential.", [](DemoManager* m, DemoProperties p) { return new CarDemo(m, p); });
	manager.registerScene("Dzhanibekov Effect", "Demonstration of dzhanibekov effect simulated in engine.", [](DemoManager* m, DemoProperties p) { return new DzhanibekovDemo(m, p); });
	manager.registerScene("Forklift", "Pick up and move pallets of boxes around with a simulated forklift.", [](DemoManager* m, DemoProperties p) { return new ForkliftDemo(m, p); });
	manager.registerScene("LDL-PGS solver demos", "A set of demos demonstrating the advantages of the ldl-pgs solver.", [](DemoManager* m, DemoProperties p) { return new LDLSolverDemos(m, p); });
	manager.registerScene("Trebuchet", "Launch projectiles with a simulated floating-arm trebuchet.", [](DemoManager* m, DemoProperties p) { return new TrebuchetDemo(m, p); });
	manager.registerScene("Angular Momentum Demo", "A common demonstration of the conservation of angular momentom, recreated in the engine", [](DemoManager* m, DemoProperties p) { return new AngularMomentumDemo(m, p); });
	manager.registerScene("Gyroscope", "A simulated gyroscope", [](DemoManager* m, DemoProperties p) { return new GyroscopeDemo(m, p); });
	manager.registerScene("Bad Apple Demo", "Renders bad apple veeerrryyyyy slowly. Requires precomputation before the first run, which takes a very long time.", [](DemoManager* m, DemoProperties p) { return new VideoDemo(m, p); });
	manager.registerScene("Newtons Cradle", "Newtons cradle.", [](DemoManager* m, DemoProperties p) { return new NewtonsCradleDemo(m, p); });
	manager.registerScene("Marble Machine", "A functioning marble machine downloaded from a 3d printing site.", [](DemoManager* m, DemoProperties p) { return new MarbleMachineDemo(m, p); });
	manager.registerScene("Stanford Armadillo", "oh yeah", [](DemoManager* m, DemoProperties p) { return new StanfordArmadillo(m, p); });
	manager.registerScene("Raycast car", "Drive a car that uses raycasts rather than constraint based wheels. Credit to shazammm for the car 3d model: https://skfb.ly/oPPLV, license: https://creativecommons.org/licenses/by/4.0/", [](DemoManager* m, DemoProperties p) { return new HovercraftDemo(m, p); });
	manager.registerScene("Debug demo", "for setting up debug scenes", [](DemoManager* m, DemoProperties p) { return new DebugDemo(m, p); });

	manager.selectProperties();
	while (1) {
		manager.selectDemoScene();
		manager.displaySceneControls();
		manager.runScene();
	}
}