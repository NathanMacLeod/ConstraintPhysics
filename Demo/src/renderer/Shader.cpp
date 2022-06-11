#include "Shader.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

/**************************************************************************
* Most of the code in this file either comes from or is based
* off of the code form the OpenGL series from TheCherno youtube channel
* https://www.youtube.com/c/TheChernoProject
***************************************************************************/

namespace rndr {

	ShaderProgramSource ParseShader(const std::string& filepath);
	unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader);
	unsigned int CompileShader(unsigned int type, const std::string& source);

	Shader::Shader(const std::string& filePath) {
		this->filePath = filePath;
		shaderID = 0;

		ShaderProgramSource source = ParseShader(filePath);
		shaderID = CreateShader(source.VertexSource, source.FragmentSource);
	}

	Shader::~Shader() {
		glDeleteProgram(shaderID);
	}

	void Shader::bind() const {
		glUseProgram(shaderID);
	}
	void Shader::unbind() const {
		glUseProgram(shaderID);
	}

	void Shader::setUniform1i(const std::string& name, int i) {
		glUniform1i(getUniformLocation(name), i);
	}

	void Shader::setUniform4f(const std::string& name, float v0, float v1, float v2, float v3) {
		glUniform4f(getUniformLocation(name), v0, v1, v2, v3);
	}

	void Shader::setUniformMat4f(const std::string& name, const Mat4& m) {
		glUniformMatrix4fv(getUniformLocation(name), 1, true, (const GLfloat*)m.v);
	}

	int Shader::getUniformLocation(const std::string& name) {
		auto i = uniformLocationCache.find(name);
		if (i != uniformLocationCache.end()) {
			return i->second;
		}

		int location = glGetUniformLocation(shaderID, name.c_str());
		if (location == -1) {
			printf("Warning, uniform %s doesn't exist\n", name.c_str());
		}
		uniformLocationCache[name] = location;

		return location;
	}

	ShaderProgramSource ParseShader(const std::string& filepath) {
		std::ifstream stream = std::ifstream(filepath);

		enum ShaderType { NONE = -1, VERTEX = 0, FRAGMENT = 1 };

		std::string line;
		std::stringstream ss[2];
		ShaderType type = ShaderType::NONE;

		while (getline(stream, line)) {
			if (line.find("#shader") != std::string::npos) {
				if (line.find("vertex") != std::string::npos) {
					type = ShaderType::VERTEX;
				}
				else if (line.find("fragment") != std::string::npos) {
					type = ShaderType::FRAGMENT;
				}
			}
			else {
				ss[(int)type] << line << "\n";
			}
		}

		return ShaderProgramSource{ ss[0].str(), ss[1].str() };
	}

	unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader) {
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

	unsigned int CompileShader(unsigned int type, const std::string& source) {
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
			printf("Failed to compile %s shader\n%s\n", (type == GL_VERTEX_SHADER) ? "vertex" : "fragment", message);
			return 0;
		}

		return id;
	}
}