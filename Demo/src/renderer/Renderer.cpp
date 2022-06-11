#include "Renderer.h"

#include <chrono>
#include <cstdio>

#include <GL/glew.h>

namespace rndr {

    static std::chrono::time_point<std::chrono::system_clock> t_last;
	static GLFWwindow* window;

    //from https://gist.github.com/liam-middlebrook/c52b069e4be2d87a6d2f
    void APIENTRY GLDebugMessageCallback(GLenum source, GLenum type, GLuint id,
        GLenum severity, GLsizei length,
        const GLchar* msg, const void* data)
    {
        char* _source;
        char* _type;
        char* _severity;

        switch (source) {
        case GL_DEBUG_SOURCE_API:
            _source = (char*)"API";
            break;

        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:
            _source = (char*)"WINDOW SYSTEM";
            break;

        case GL_DEBUG_SOURCE_SHADER_COMPILER:
            _source = (char*)"SHADER COMPILER";
            break;

        case GL_DEBUG_SOURCE_THIRD_PARTY:
            _source = (char*)"THIRD PARTY";
            break;

        case GL_DEBUG_SOURCE_APPLICATION:
            _source = (char*)"APPLICATION";
            break;

        case GL_DEBUG_SOURCE_OTHER:
            _source = (char*)"UNKNOWN";
            break;

        default:
            _source = (char*)"UNKNOWN";
            break;
        }

        switch (type) {
        case GL_DEBUG_TYPE_ERROR:
            _type = (char*)"ERROR";
            break;

        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR:
            _type = (char*)"DEPRECATED BEHAVIOR";
            break;

        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:
            _type = (char*)"UDEFINED BEHAVIOR";
            break;

        case GL_DEBUG_TYPE_PORTABILITY:
            _type = (char*)"PORTABILITY";
            break;

        case GL_DEBUG_TYPE_PERFORMANCE:
            _type = (char*)"PERFORMANCE";
            break;

        case GL_DEBUG_TYPE_OTHER:
            _type = (char*)"OTHER";
            break;

        case GL_DEBUG_TYPE_MARKER:
            _type = (char*)"MARKER";
            break;

        default:
            _type = (char*)"UNKNOWN";
            break;
        }

        switch (severity) {
        case GL_DEBUG_SEVERITY_HIGH:
            _severity = (char*)"HIGH";
            break;

        case GL_DEBUG_SEVERITY_MEDIUM:
            _severity = (char*)"MEDIUM";
            break;

        case GL_DEBUG_SEVERITY_LOW:
            _severity = (char*)"LOW";
            break;

        case GL_DEBUG_SEVERITY_NOTIFICATION:
            _severity = (char*)"NOTIFICATION";
            break;

        default:
            _severity = (char*)"UNKNOWN";
            break;
        }

        printf("%d: %s of %s severity, raised from %s: %s\n",
            id, _type, _severity, _source, msg);
    }

    int getKey(int key) {
        return glfwGetKey(window, key);
    }

	int init(int width, int height, const std::string& name) {
		if (!glfwInit()) {
			std::printf("glew init failed!\n");
			return -1;
		}

        glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);

		window = glfwCreateWindow(width, height, "Hello World", NULL, NULL);
		if (!window)
		{
			glfwTerminate();
			return -1;
		}

		glfwMakeContextCurrent(window);

		if (glewInit() != GLEW_OK) {
			std::printf("glew init failed!\n");
			return -1;
		}

        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(GLDebugMessageCallback, NULL);

        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_CULL_FACE);

        std::printf("%s\n", glGetString(GL_VERSION));

        return 1;
	}

    void clear(color c) {
        glClearColor(c.r, c.g, c.b, c.a);
        glClearDepth(1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void draw(const VertexArray& va, const IndexBuffer& ib, const Shader& s) {
        va.bind();
        ib.bind();
        s.bind();
        glDrawElements(GL_TRIANGLES, ib.getCount(), GL_UNSIGNED_INT, nullptr);
    }

    bool render_loop(float* fElapsedTimeOut) {
        static bool first = true;

        glfwSwapBuffers(window);
        glfwPollEvents();

        auto t_now = std::chrono::system_clock::now();

        if (first) {
            first = false;
            *fElapsedTimeOut = 0;
        }
        else {
            std::chrono::duration<float> t_passed = t_now - t_last;
            *fElapsedTimeOut = t_passed.count();
        }
        t_last = t_now;

        return !glfwWindowShouldClose(window);
    }

    void terminate() {
        glfwTerminate();
    }

    void foo() {
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

}