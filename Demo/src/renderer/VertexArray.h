#pragma once

#include "IndexBuffer.h"

#include <vector>
#include <GL/glew.h>

/**************************************************************************
* Most of the code in this file either comes from or is based
* off of the code form the OpenGL series from TheCherno youtube channel
* https://www.youtube.com/c/TheChernoProject
***************************************************************************/

namespace rndr {

	struct VertexBufferElement {
		unsigned int type;
		unsigned int count;
		unsigned char normalized;

		static unsigned int GetSizeOfType(unsigned int type) {
			switch (type) {
			case GL_FLOAT:         return 4;
			case GL_UNSIGNED_INT:  return 4;
			case GL_UNSIGNED_BYTE: return 1;
			}

			return 0;
		}
	};

	class VertexArrayLayout {
	public:
		VertexArrayLayout() {
			stride = 0;
		}

		template<typename T>
		void Push(unsigned int count) {}

		template<>
		void Push<float>(unsigned int count);

		template<>
		void Push<unsigned int>(unsigned int count);

		template<>
		void Push<unsigned char>(unsigned int count);

		inline const std::vector<VertexBufferElement>& getElements() const { return elements; }
		inline unsigned int GetStride() const { return stride; }
	private:
		std::vector<VertexBufferElement> elements;
		unsigned int stride;
	};

	class VertexArray {
	public:
		VertexArray(void* data, unsigned int data_size, const VertexArrayLayout& layout);
		~VertexArray();

		void bind() const;
		void unbind() const;
	private:
		unsigned int vertexArrayID;
		unsigned int vertexBufferID;
	};


	class BatchArray {
	public:
		BatchArray(const VertexArrayLayout& layout, unsigned int vertex_capacity);
		~BatchArray();

		void push(void* vertex_data, int vertex_count, std::vector<unsigned int> index_data);
		void flush();

		unsigned int remainingCapacity() const;
		void bindVertexArray() const;
		void unbindVertexArray() const;
		IndexBuffer generateIndexBuffer() const;

	private:
		unsigned int vertex_capacity;
		unsigned int n_vertices_allocated;
		unsigned int vertex_size;

		unsigned int vertexArrayID;
		unsigned int vertexBufferID;
		std::vector<unsigned int> index_buffer_data;


	};
}