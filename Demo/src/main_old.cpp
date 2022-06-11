#include <string>
#include "../../ConstraintPhysics/src/PhysicsEngine.h"

#include "renderer/Shader.h"
#define GLEW_STATIC
#include <GL/glew.h>

#include <GLFW/glfw3.h>

static unsigned int CompileShader(unsigned int type, const std::string& source) {
	unsigned int id = glCreateShader(type);
	const char* src = source.c_str();
	glShaderSource(id, 1, &src, nullptr);
	glCompileShader(id);

	int result;
	glGetShaderiv(id, GL_COMPILE_STATUS, &result);
	if (result == GL_FALSE) {
		int length;
		glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
		char* message = (char*)alloca(length * sizeof(char));
		glGetShaderInfoLog(id, length, &length, message);
		printf("Failed to compile %s shader\n%s\n", (type == GL_VERTEX_SHADER)? "vertex" : "fragment", message);
		return 0;
	}

	return id;
}

static unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader) {
	unsigned int program = glCreateProgram();
	unsigned int vs = CompileShader(GL_VERTEX_SHADER, vertexShader);
	unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShader);

	glAttachShader(program, vs);
	glAttachShader(program, fs);
	glLinkProgram(program);
	glValidateProgram(program);

	glDeleteShader(vs);
	glDeleteShader(fs);

	return program;
}

int mainz(void)
{
	phyz::PhysicsEngine p;

	GLFWwindow* window;

	/* Initialize the library */
	if (!glfwInit())
		return -1;

	/* Create a windowed mode window and its OpenGL context */
	window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		return -1;
	}

	/* Make the window's context current */
	glfwMakeContextCurrent(window);

	if (glewInit() != GLEW_OK) {
		std::printf("glew init failed!\n");
	}

	std::printf("%s\n", glGetString(GL_VERSION));

	float positions[6] = {
		-0.5f, -0.5f,
		 0.0f,  0.5f,
		 0.5f, -0.5f,
	};

	unsigned int buffer;
	glGenBuffers(1, &buffer);
	glBindBuffer(GL_ARRAY_BUFFER, buffer);
	glBufferData(GL_ARRAY_BUFFER, 6*sizeof(float), positions, GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), 0);
	
	/*std::string vertexShader =
		"#version 330 core\n"
		"\n"
		"layout(location = 0) in vec4 position;"
		"\n"
		"void main() {\n"
		"	gl_Position = position;\n"
		"}\n";

	std::string fragmentShader = 
		"#version 330 core\n"
		"\n"
		"layout(location = 0) out vec4 color;"
		"\n"
		"void main() {\n"
		"	color = vec4(1.0, 0.0, 0.0, 1.0);\n"
		"}\n";

	unsigned int shader = CreateShader(vertexShader, fragmentShader);
	glUseProgram(shader);*/

	rndr::Shader shader("resources/shaders/Basic.shader");
	shader.bind();

	/* Loop until the user closes the window */
	while (!glfwWindowShouldClose(window))
	{
		/* Render here */
		glClear(GL_COLOR_BUFFER_BIT);

		glDrawArrays(GL_TRIANGLES, 0, 3);

		/* Swap front and back buffers */
		glfwSwapBuffers(window);

		/* Poll for and process events */
		glfwPollEvents();
	}

	//glDeleteShader(shader);

	glfwTerminate();
	return 0;
}

/*double randD(double min, double max) {
	return min + (max - min) * std::rand() / RAND_MAX;
}*/

/*int main() {

	//std::vector<ConvexPoly> geometry;

	//double x = 0, y = 0, z = 0;
	//double w = 10, h = 20, d = 30;

	geometry.push_back(getRect(x, y, z, w, h, d));

	Quaternion q1 = Quaternion(randD(0, 2 * 3.14159265358979323846), Vec3(randD(-1, 1), randD(-1, 1), randD(-1, 1)));
	Quaternion q2 = Quaternion(randD(0, 2 * 3.14159265358979323846), Vec3(randD(-1, 1), randD(-1, 1), randD(-1, 1)));
	Quaternion q3 = Quaternion(randD(0, 2 * 3.14159265358979323846), Vec3(randD(-1, 1), randD(-1, 1), randD(-1, 1)));
	Quaternion q = q3 * q2 * q1;

	geometry[0].rotate(q, Vec3(x, y, z) + Vec3(w, h, d) / 2.0);

	RigidBody r = RigidBody(geometry, 1);

	geometry[0].SAT(geometry[0]);

	phyz::PhysicsEngine p;
	p.foo();

	Quaternion q1 = Quaternion(3.141592/4.0, Vec3(0, 0, 1));
	Quaternion q2 = Quaternion(3.141592 / 4.0, Vec3(0, 1, 0));

	ConvexPoly c1 = getRect(0, 0, 0, 2, 2, 2);
	c1.rotate(q1, Vec3(1, 1, 1));
	ConvexPoly c2 = getRect(2, 0, 0, 2, 2, 2);
	c2.rotate(q2, Vec3(3, 1, 1));

	Manifold info = c1.SAT(c2, 5);
	int a = 3 + 5;

}*/
