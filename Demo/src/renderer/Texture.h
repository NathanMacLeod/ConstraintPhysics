#pragma once
#include <string>

namespace rndr {

	class Texture {
	public:
		Texture() : no_texture(true) {}
		Texture(std::string texture_path);

		bool isNoTexture() const { return textureID == -1; }
		void bindToTextureUnitI(unsigned int i) const;
		int getTextureID() const { return textureID; }
		void deleteTexture();

		static Texture noTexture() { return Texture(); }

	private:
		bool no_texture;
		unsigned int textureID;
	};

}