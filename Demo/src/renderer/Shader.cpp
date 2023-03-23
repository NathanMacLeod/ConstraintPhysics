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
	unsigned int CreateShader(const std::string& vertexShader, const std::string& geometryShader, const std::string& fragmentShader);
	unsigned int CompileShader(unsigned int type, const std::string& source);

	Shader::Shader(const std::string& filePath) {
		this->filePath = filePath;
		shaderID = 0;

		ShaderProgramSource source = ParseShader(filePath);
		shaderID = CreateShader(source.VertexSource, source.GeometrySource, source.FragmentSource);
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

	void Shader::setUniform3f(const std::string& name, float v0, float v1, float v2) {
		glUniform3f(getUniformLocation(name), v0, v1, v2);
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

		enum ShaderType { NONE = -1, VERTEX = 0, GEOMETRY = 1, FRAGMENT = 2 };

		std::string line;
		std::stringstream ss[3];
		ShaderType type = ShaderType::NONE;

		while (getline(stream, line)) {
			if (line.find("#shader") != std::string::npos) {
				if (line.find("vertex") != std::string::npos) {
					type = ShaderType::VERTEX;
				}
				else if (line.find("geometry") != std::string::npos) {
					type = ShaderType::GEOMETRY;
				}
				else if (line.find("fragment") != std::string::npos) {
					type = ShaderType::FRAGMENT;
				}
			}
			else {
				ss[(int)type] << line << "\n";
			}
		}

		return ShaderProgramSource{ ss[0].str(), ss[1].str(), ss[2].str() };
	}

	unsigned int CreateShader(const std::string& vertexShader, const std::string& geometryShader, const std::string& fragmentShader) {
		unsigned int program = glCreateProgram();
		unsigned int vs = CompileShader(GL_VERTEX_SHADER, vertexShader);
		unsigned int gs = CompileShader(GL_GEOMETRY_SHADER, geometryShader);
		unsigned int fs = CompileShader(GL_FRAGMENT_SHADER, fragmentShader);

		glAttachShader(program, vs);
		glAttachShader(program, gs);
		glAttachShader(program, fs);
		glLinkProgram(program);
		glValidateProgram(program);

		glDeleteShader(vs);
		glDeleteShader(fs);

		return program;
	}

	std::string getShaderTypeName(unsigned int type) {
		switch (type) {
		case GL_VERTEX_SHADER: return "vertex";
		case GL_GEOMETRY_SHADER: return "geometry";
		case GL_FRAGMENT_SHADER: return "fragment";
		default: return "(type not recognized)";
		}
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
			printf("Failed to compile %s shader\n%s\n", getShaderTypeName(type).c_str(), message);
			return 0;
		}

		return id;
	}
}