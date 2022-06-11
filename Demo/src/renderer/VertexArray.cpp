#include "VertexArray.h"

/**************************************************************************
* Most of the code in this file either comes from or is based
* off of the code form the OpenGL series from TheCherno youtube channel
* https://www.youtube.com/c/TheChernoProject
***************************************************************************/

namespace rndr {

	template<>
	void VertexArrayLayout::Push<float>(unsigned int count) {
		elements.push_back({ GL_FLOAT, count, GL_FALSE });
		stride += count * VertexBufferElement::GetSizeOfType(GL_FLOAT);
	}

	template<>
	void VertexArrayLayout::Push<unsigned int>(unsigned int count) {
		elements.push_back({ GL_UNSIGNED_INT, count, GL_FALSE });
		stride += count * VertexBufferElement::GetSizeOfType(GL_UNSIGNED_INT) * count;
	}

	template<>
	void VertexArrayLayout::Push<unsigned char>(unsigned int count) {
		elements.push_back({ GL_UNSIGNED_BYTE, count, GL_TRUE });
		stride += count * VertexBufferElement::GetSizeOfType(GL_UNSIGNED_BYTE) * count;
	}

	VertexArray::VertexArray(void* data, unsigned int data_size, const VertexArrayLayout& layout) {
		glGenBuffers(1, &vertexBufferID);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBufferID);
		glBufferData(GL_ARRAY_BUFFER, data_size, data, GL_STATIC_DRAW);

		glGenVertexArrays(1, &vertexArrayID);

		bind();
		const auto& elements = layout.getElements();
		unsigned int offset = 0;
		for (unsigned int i = 0; i < elements.size(); i++) {
			const auto& element = elements[i];
			glEnableVertexAttribArray(i);
			glVertexAttribPointer(i, element.count, element.type, element.normalized, layout.GetStride(), (const void*)offset);
			offset += element.count * VertexBufferElement::GetSizeOfType(element.type);
		}
	}

	VertexArray::~VertexArray() {
		glDeleteBuffers(1, &vertexBufferID);
		glDeleteVertexArrays(1, &vertexArrayID);
	}

	void VertexArray::bind() const {
		glBindVertexArray(vertexArrayID);
	}
	void VertexArray::unbind() const {
		glBindVertexArray(0);
	}

}