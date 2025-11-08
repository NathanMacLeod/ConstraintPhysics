#pragma once

#include "../../../Math/src/Quaternion.h"
#include "../../../Math/src/Vec3.h"

namespace rndr {

	class Mat4 {
	public:
		Mat4() {}
		Mat4(const mthz::Mat3& m) {
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					v[i][j] = static_cast<float>(m.v[i][j]);
				}
			}
		}

		float v[4][4] = { {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} };

		static Mat4 iden() {
			Mat4 out;
			out.v[0][0] = 1.0f;
			out.v[1][1] = 1.0f;
			out.v[2][2] = 1.0f;
			out.v[3][3] = 1.0f;
			return out;
		}

		static Mat4 ortho(float w, float h, float d) {
			Mat4 out;
			out.v[0][0] = 2.0f / w;
			out.v[1][1] = 2.0f / h;
			out.v[2][2] = 2.0f / d;
			out.v[3][3] = 1.0;
			return out;
		}

		static Mat4 translation(float x, float y, float z) {
			Mat4 tr;
			tr.v[0][0] = 1.0f; tr.v[0][3] = x;
			tr.v[1][1] = 1.0f; tr.v[1][3] = y;
			tr.v[2][2] = 1.0f; tr.v[2][3] = z;
			tr.v[3][3] = 1.0f;
			return tr;
		}

		static Mat4 cam_view(mthz::Vec3 cam_pos, mthz::Quaternion cam_orient) {
			Mat4 tr = translation(static_cast<float>(-cam_pos.x), static_cast<float>(-cam_pos.y), static_cast<float>(-cam_pos.z));
			Mat4 rot = cam_orient.conjugate().getRotMatrix();
			rot.v[3][3] = 1.0f;
			return rot * tr;
		}

		static Mat4 model(mthz::Vec3 pos, mthz::Quaternion orientation) {
			Mat4 tr = translation(static_cast<float>(pos.x), static_cast<float>(pos.y), static_cast<float>(pos.z));
			Mat4 rot = orientation.getRotMatrix();
			rot.v[3][3] = 1.0f;
			return tr * rot;
		}

		static Mat4 proj(float near, float far, float w, float h, float FOV) {
			float f = static_cast<float>(tan(FOV * PI / 360));
			
			Mat4 out;
			out.v[0][0] = h / (f * w);
			out.v[1][1] = 1.0f / f;
			out.v[2][2] = -(far + near) / (far - near); out.v[2][3] = -2.0f * far * near / (far - near);
			out.v[3][2] = -1.0f;

			return out;
		}

		static Mat4 window2DTransform(float window_width, float window_height) {
			Mat4 out;

			out.v[0][0] = 2.0f / window_width; out.v[0][3] = -1.0f;
			out.v[1][1] = 2.0f / window_height; out.v[1][3] = -1.0f;
			out.v[2][2] = 1.0f;
			out.v[3][3] = 1.0f;

			return out;
		}

		Mat4 operator*(const Mat4 m) {
			Mat4 out;
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					for (int k = 0; k < 4; k++) {
						out.v[i][j] += v[i][k] * m.v[k][j];
					}
				}
			}
			return out;
		}

		void operator*=(const Mat4 m) {
			float out[4][4] = { {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} };
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					for (int k = 0; k < 4; k++) {
						out[i][j] += v[i][k] * m.v[k][j];
					}
				}
			}

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					v[i][j] = out[i][j];
				}
			}
		}

		void operator*=(const float d) {
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					v[i][j] *= d;
				}
			}
		}

		void operator/=(const float d) {
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					v[i][j] /= d;
				}
			}
		}
	};
}