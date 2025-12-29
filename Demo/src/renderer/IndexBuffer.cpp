#include "IndexBuffer.h"
#include <glad/gl.h>
#include <GLFW/glfw3.h>

/**************************************************************************
* Most of the code in this file either comes from or is based
* off of the code form the OpenGL series from TheCherno youtube channel
* https://www.youtube.com/c/TheChernoProject
***************************************************************************/

namespace rndr {

	IndexBuffer::IndexBuffer(const unsigned int* data, unsigned int count) {
		this->count = count;
		glGenBuffers(1, &indexBufferID);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferID);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, count * sizeof(unsigned int), data, GL_STATIC_DRAW);
	}

	IndexBuffer::~IndexBuffer() {
		glDeleteBuffers(1, &indexBufferID);
	}

	void IndexBuffer::bind() const {
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferID);
	}
	void IndexBuffer::unbind() const {
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}


}