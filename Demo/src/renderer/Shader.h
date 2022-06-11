#pragma once

#include <string>
#include <unordered_map>

#include "Mat4.h"

/**************************************************************************
* Most of the code in this file either comes from or is based
* off of the code form the OpenGL series from TheCherno youtube channel
* https://www.youtube.com/c/TheChernoProject
***************************************************************************/

namespace rndr {

	struct ShaderProgramSource {
		std::string VertexSource;
		std::string FragmentSource;
	};

	class Shader {
	public:
		Shader(const std::string& filePath);
		~Shader();

		void bind() const;
		void unbind() const;

		void setUniform1i(const std::string& name, int i);
		void setUniform4f(const std::string& name, float v0, float v1, float v2, float v3);
		void setUniformMat4f(const std::string& name, const Mat4& m);

	private:
		std::string filePath;
		unsigned int shaderID;
		std::unordered_map<std::string, int> uniformLocationCache;

		int getUniformLocation(const std::string& name);
	};

}