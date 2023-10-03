#include "VertexArray.h"
#include "Shader.h"
#include <cassert>

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

	BatchArray::BatchArray(const VertexArrayLayout& layout, unsigned int vertex_capacity)
		: vertex_capacity(vertex_capacity), n_indices_allocated(0), n_vertices_allocated(0)
	{
		const int INDEX_BUFFER_SIZE_FACTOR = 6;
		index_capacity = INDEX_BUFFER_SIZE_FACTOR * vertex_capacity;

		vertex_size = layout.GetStride();

		glGenBuffers(1, &vertexBufferID);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBufferID);
		glBufferData(GL_ARRAY_BUFFER, vertex_capacity * vertex_size, nullptr, GL_DYNAMIC_DRAW);

		glGenBuffers(1, &indexBufferID);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferID);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * vertex_capacity * sizeof(unsigned int), nullptr, GL_DYNAMIC_DRAW);

		glGenVertexArrays(1, &vertexArrayID);

		glBindVertexArray(vertexArrayID);
		const auto& elements = layout.getElements();
		unsigned int offset = 0;
		for (unsigned int i = 0; i < elements.size(); i++) {
			const auto& element = elements[i];
			glEnableVertexAttribArray(i);
			glVertexAttribPointer(i, element.count, element.type, element.normalized, layout.GetStride(), (const void*)offset);
			offset += element.count * VertexBufferElement::GetSizeOfType(element.type);
		}
	}

	BatchArray::~BatchArray() {
		glDeleteBuffers(1, &vertexBufferID);
		glDeleteBuffers(1, &indexBufferID);
		glDeleteVertexArrays(1, &vertexArrayID);
	}

	BatchArray::TexturePushStatus BatchArray::attemptPushTexture(Texture t) {
		if (t.isNoTexture()) {
			return { false, -1.0f }; //-1.0f value is to be interpreted as 'no texture' by the shader
		}
		else {
			//check if the same texture has already been pushed
			for (int i = 0; i < assigned_textures.size(); i++) {
				if (assigned_textures[i].getTextureID() == t.getTextureID()) {
					return { false, (float)i };
				}
			}

			//is a new texture, check for capacity
			if (assigned_textures.size() < texture_capcity) {
				assigned_textures.push_back(t);
				return { false, (float)(assigned_textures.size() - 1) };
			}
			else {
				return { true };
			}
		}
	}

	void BatchArray::push(void* vertex_data, int vertex_count, std::vector<unsigned int> index_data) {
		assert(vertex_count + n_vertices_allocated <= vertex_capacity && index_data.size() + n_indices_allocated <= index_capacity);

		std::vector<unsigned int> adjusted_indices(index_data);
		for (int i = 0; i < adjusted_indices.size(); i++) {
			adjusted_indices[i] += n_vertices_allocated;
		}

		bind();

		glBufferSubData(GL_ARRAY_BUFFER, n_vertices_allocated * vertex_size, vertex_count * vertex_size, vertex_data);
		glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, n_indices_allocated * sizeof(unsigned int), adjusted_indices.size() * sizeof(unsigned int), adjusted_indices.data());

		n_vertices_allocated += vertex_count;
		n_indices_allocated += index_data.size();
	}

	void BatchArray::flush() {
		n_vertices_allocated = 0;
		n_indices_allocated = 0;
		assigned_textures.clear();
	}

	unsigned int BatchArray::remainingVertexCapacity() const {
		return vertex_capacity - n_vertices_allocated;
	}

	unsigned int BatchArray::remainingIndexCapacity() const {
		return index_capacity - n_indices_allocated;
	}

	void BatchArray::bind() const {
		for (int i = 0; i < assigned_textures.size(); i++) {
			assigned_textures[i].bindToTextureUnitI(i);
		}

		glBindVertexArray(vertexArrayID);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBufferID);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferID);
	}

	void BatchArray::unbind() const {
		glBindVertexArray(0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}
}