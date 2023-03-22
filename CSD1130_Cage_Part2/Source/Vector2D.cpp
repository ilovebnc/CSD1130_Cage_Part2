/******************************************************************************/
/*!
\file		Vector2D.cpp
\author 	Guo Yiming, yiming.guo, 2202613
\par    	email: yiming.guo@digipen.edu
\date   	Mar 18, 2023
\brief		This source file defines the functions for Vector2D.

Copyright (C) 2023 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the
prior written consent of DigiPen Institute of Technology is prohibited.
 */
 /******************************************************************************/

#include "Vector2D.h"
#include <math.h>

namespace CSD1130 {
	// Constructors
	Vector2D::Vector2D(float _x, float _y) : x(_x), y(_y) {}

	// Assignment operators
	Vector2D& Vector2D::operator += (const Vector2D& rhs) {
		x += rhs.x;
		y += rhs.y;
		return *this;
	}

	Vector2D& Vector2D::operator -= (const Vector2D& rhs) {
		x -= rhs.x;
		y -= rhs.y;
		return *this;
	}

	Vector2D& Vector2D::operator *= (float rhs) {
		x *= rhs;
		y *= rhs;
		return *this;
	}

	Vector2D& Vector2D::operator /= (float rhs) {
		x /= rhs;
		y /= rhs;
		return *this;
	}

	// Unary operators
	Vector2D Vector2D::operator -() const {
		Vector2D tmp(-x, -y);
		return tmp;
	}

// Binary operators
	Vector2D operator + (const Vector2D& lhs, const Vector2D& rhs) {
		Vector2D tmp = lhs;
		tmp += rhs;
		return tmp;
	}
	Vector2D operator - (const Vector2D& lhs, const Vector2D& rhs) {
		Vector2D tmp = lhs;
		tmp -= rhs;
		return tmp;
	}
	Vector2D operator * (const Vector2D& lhs, float rhs) {
		Vector2D tmp = lhs;
		tmp *= rhs;
		return tmp;
	}
	Vector2D operator * (float lhs, const Vector2D& rhs) {
		Vector2D tmp = rhs;
		tmp *= lhs;
		return tmp;
	}
	Vector2D operator / (const Vector2D& lhs, float rhs) {
		Vector2D tmp = lhs;
		tmp /= rhs;
		return tmp;
	}


	void Vector2DNormalize(Vector2D& pResult, const Vector2D& pVec0) {
		pResult = pVec0 / Vector2DLength(pVec0);
	}

	float Vector2DLength(const Vector2D& pVec0) {
		return (sqrtf(pVec0.x * pVec0.x + pVec0.y * pVec0.y));
	}

	float Vector2DSquareLength(const Vector2D& pVec0) {
		return (pVec0.x * pVec0.x + pVec0.y * pVec0.y);
	}
	
	float Vector2DDistance(const Vector2D& pVec0, const Vector2D& pVec1) {
		return (sqrtf(powf(pVec1.x - pVec0.x, 2.0f) + powf(pVec1.y - pVec0.y, 2.0f)));
	}

	float Vector2DSquareDistance(const Vector2D& pVec0, const Vector2D& pVec1) {
		return (powf(pVec1.x - pVec0.x, 2.0f) + powf(pVec1.y - pVec0.y, 2.0f));
	}

	float Vector2DDotProduct(const Vector2D& pVec0, const Vector2D& pVec1) {
		return (pVec0.x * pVec1.x + pVec0.y * pVec1.y);
	}

	float Vector2DCrossProductMag(const Vector2D& pVec0, const Vector2D& pVec1) {
		return (pVec0.x * pVec1.y - pVec0.y * pVec1.x);
	}
}