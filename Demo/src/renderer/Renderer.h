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

	int init(int width, int height, const std::string& name);
	int getKey(int key);
	void clear(color c);
	void draw(const VertexArray& va, const IndexBuffer& ib, const Shader& s);
	bool render_loop(float* fElapsedTime_out);
	void terminate();

}