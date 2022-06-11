#pragma once

#include <vector>

/**************************************************************************
* Most of the code in this file either comes from or is based
* off of the code form the OpenGL series from TheCherno youtube channel
* https://www.youtube.com/c/TheChernoProject
***************************************************************************/

namespace rndr {

	class IndexBuffer {
	private:
		unsigned int indexBufferID;
		unsigned int count;
	public:
		IndexBuffer(const unsigned int* data, unsigned int count);
		~IndexBuffer();

		void bind() const;
		void unbind() const;

		inline unsigned int getCount() const { return count; }
	};
}