/******************************************************************************/
/*!
\file		Matrix3x3.cpp
\author 	Guo Yiming, yiming.guo, 2202613
\par    	email: yiming.guo@digipen.edu
\date   	Mar 18, 2023
\brief		This source file defines the functions for Matrix3x3.

Copyright (C) 2023 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the
prior written consent of DigiPen Institute of Technology is prohibited.
 */
 /******************************************************************************/

#include "Matrix3x3.h"
#include <cmath>

namespace CSD1130 {
#ifdef _MSC_VER
	// Supress warning: nonstandard extension used : nameless struct/union
#pragma warning( disable : 4201 )
#endif

// Member functions
	Matrix3x3::Matrix3x3(const float* pArr) {
		for (int i{}; i < 9; ++i) {
			m[i] = pArr[i];
		}
	}

	Matrix3x3::Matrix3x3(float _00, float _01, float _02,
		float _10, float _11, float _12,
		float _20, float _21, float _22) :
		m00(_00), m01(_01), m02(_02),
		m10(_10), m11(_11), m12(_12),
		m20(_20), m21(_21), m22(_22) {}

	Matrix3x3& Matrix3x3::operator=(const Matrix3x3& rhs) {
		for (int i{}; i < 9; ++i) {
			m[i] = rhs.m[i];
		}
		return *this;
	}

	// Assignment operators
	Matrix3x3& Matrix3x3::operator *= (const Matrix3x3& rhs) {
		Matrix3x3 result;
		for (int i{}; i < 3; i++) {
			for (int j{}; j < 3; j++) {
				float sum = 0.0f;
				for (int k{}; k < 3; k++) {
					sum += m[i * 3 + k] * rhs.m[k * 3 + j];
				}
				result.m[i * 3 + j] = sum;
			}
		}
		*this = result;
		return *this;
	}

	// Non-member functions
	Matrix3x3 operator * (const Matrix3x3& lhs, const Matrix3x3& rhs) {
		Matrix3x3 result;
		for (int i{}; i < 3; ++i) {
			for (int j{}; j < 3; ++j) {
				result.m[i * 3 + j] =
					lhs.m[i * 3 + 0] * rhs.m[0 * 3 + j] +
					lhs.m[i * 3 + 1] * rhs.m[1 * 3 + j] +
					lhs.m[i * 3 + 2] * rhs.m[2 * 3 + j];
			}
		}
		return result;
	}

	Vector2D  operator * (const Matrix3x3& pMtx, const Vector2D& rhs) {
		float x = pMtx.m00 * rhs.x + pMtx.m01 * rhs.y + pMtx.m02;
		float y = pMtx.m10 * rhs.x + pMtx.m11 * rhs.y + pMtx.m12;
		float w = pMtx.m20 * rhs.x + pMtx.m21 * rhs.y + pMtx.m22;
		return Vector2D(x / w, y / w);
	}

	void Mtx33Identity(Matrix3x3& pResult) {
		for (int i{}; i < 3; i++) {
			for (int j{}; j < 3; j++) {
				pResult.m2[i][j] = (i == j) ? 1.0f : 0.0f;
			}
		}
	}

	void Mtx33Translate(Matrix3x3& pResult, float x, float y) {
		Mtx33Identity(pResult);
		pResult.m02 = x;
		pResult.m12 = y;
	}

	void Mtx33Scale(Matrix3x3& pResult, float x, float y) {
		Mtx33Identity(pResult);
		pResult.m00 = x;
		pResult.m11 = y;
	}

	void Mtx33RotRad(Matrix3x3& pResult, float angle) {
		Mtx33Identity(pResult);  // Start with the identity matrix
		pResult.m00 = cosf(angle);  // Set the cos of angle to the first element of the first row
		pResult.m01 = -sinf(angle); // Set the negative sin of angle to the second element of the first row
		pResult.m10 = sinf(angle);  // Set the sin of angle to the first element of the second row
		pResult.m11 = cosf(angle);  // Set the cos of angle to the second element of the second row
	}

	void Mtx33RotDeg(Matrix3x3& pResult, float angle) {
		float radians = angle * 3.14159265358f / 180.0f;
		Mtx33RotRad(pResult, radians);
	}

	void Mtx33Transpose(Matrix3x3& pResult, const Matrix3x3& pMtx) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				pResult.m2[i][j] = pMtx.m2[j][i]; // Swap the rows and columns
			}
		}
	}

	void Mtx33Inverse(Matrix3x3* pResult, float* determinant, const Matrix3x3& pMtx) {
		float a = pMtx.m00, b = pMtx.m01, c = pMtx.m02,
			d = pMtx.m10, e = pMtx.m11, f = pMtx.m12,
			g = pMtx.m20, h = pMtx.m21, i = pMtx.m22;

		float det = a * (e * i - f * h) -
			b * (d * i - f * g) +
			c * (d * h - e * g);

		if (det == 0) {
			// The matrix is singular, cannot be inverted
			pResult = NULL;
			return;
		}

		*determinant = det;

		float invDet = 1.0f / det;

		// Calculate the inverse matrix
		pResult->m00 = (e * i - f * h) * invDet;
		pResult->m01 = -(b * i - c * h) * invDet;
		pResult->m02 = (b * f - c * e) * invDet;
		pResult->m10 = -(d * i - f * g) * invDet;
		pResult->m11 = (a * i - c * g) * invDet;
		pResult->m12 = -(a * f - c * d) * invDet;
		pResult->m20 = (d * h - e * g) * invDet;
		pResult->m21 = -(a * h - b * g) * invDet;
		pResult->m22 = (a * e - b * d) * invDet;
	}
}