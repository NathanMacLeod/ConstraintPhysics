#include "Texture.h"
#include <stb_image.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace rndr {

	Texture::Texture(std::string texture_path) 
		: no_texture(false)
	{
		int texture_width, texture_height, num_color_ch;
		stbi_set_flip_vertically_on_load(true);
		unsigned char* image = stbi_load(texture_path.c_str(), &texture_width, &texture_height, &num_color_ch, 4);

		glGenTextures(1, &textureID);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, textureID);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, texture_width, texture_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
		glGenerateMipmap(GL_TEXTURE_2D);

		stbi_image_free(image);
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	void Texture::bindToTextureUnitI(unsigned int i) const {
		glActiveTexture(GL_TEXTURE0 + i);
		glBindTexture(GL_TEXTURE_2D, textureID);
	}

	void Texture::deleteTexture() {
		glDeleteTextures(1, &textureID);
	}

}