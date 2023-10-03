#pragma once

#include "Shader.h"
#include "VertexArray.h"
#include "IndexBuffer.h"

#include <GLFW/glfw3.h>
#include <string>

namespace rndr {

	struct color {
		color(float r, float g, float b, float a=1.0f) {
			this->r = r;
			this->g = g;
			this->b = b;
			this->a = a;
		}

		float r, g, b, a;
	};

	struct MousePos {
		double x, y;
	};

	int init(int width, int height, const std::string& name);
	bool getKeyDown(int key);
	bool getKeyPressed(int key);
	int getWindowWidth();
	int getWindowHeight();
	MousePos getMousePosition();
	void clear(color c);
	void draw(const VertexArray& va, const IndexBuffer& ib, const Shader& s);
	void draw(const BatchArray& ba, const Shader& s);
	void lockMouse();
	void setMode2D();
	void setMode3D();
	bool render_loop(float* fElapsedTime_out);
	void terminate();

}