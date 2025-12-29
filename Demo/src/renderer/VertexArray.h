#pragma once

#include "IndexBuffer.h"
#include "Texture.h"

#include <vector>
#include <glad/gl.h>

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
		void Push<int32_t>(unsigned int count);

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
		uint32_t vertexArrayID;
		uint32_t vertexBufferID;
	};

	class Shader;

	class BatchArray {
	public:
		BatchArray(const VertexArrayLayout& layout, unsigned int vertex_capacity);
		~BatchArray();

		struct TexturePushStatus {
			bool failed_no_capacity;
			float assigned_unit;
		};

		TexturePushStatus attemptPushTexture(Texture t);

		void push(void* vertex_data, uint32_t vertex_count, std::vector<uint32_t> index_data);
		void flush();

		uint32_t remainingTextureCapacity() const;
		uint32_t remainingVertexCapacity() const;
		uint32_t remainingIndexCapacity() const;
		inline uint32_t getIndexCount() const { return n_indices_allocated; }
		inline uint32_t getTextureCount() const { return static_cast<uint32_t>(assigned_textures.size()); }

		void bind() const;
		void unbind() const;

	private:
		uint32_t texture_capcity = 32;
		uint32_t vertex_capacity;
		uint32_t index_capacity;
		uint32_t n_vertices_allocated;
		uint32_t n_indices_allocated;
		uint32_t vertex_size;

		uint32_t vertexArrayID;
		uint32_t vertexBufferID;
		uint32_t indexBufferID;

		std::vector<Texture> assigned_textures;
	};

}