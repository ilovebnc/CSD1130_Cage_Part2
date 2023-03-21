/******************************************************************************/
/*!
\file		Collision.cpp
\author 	Guo Yiming, yiming.guo, 2202613
\par    	email: yiming.guo@digipen.edu
\date   	Mar 18, 2023
\brief

Copyright (C) 2023 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the
prior written consent of DigiPen Institute of Technology is prohibited.
 */
/******************************************************************************/

#include "main.h"

/******************************************************************************/
/*!
 */
/******************************************************************************/
void BuildLineSegment(LineSegment &lineSegment,
	const CSD1130::Vec2& p0,
	const CSD1130::Vec2& p1)
{
	// Set the endpoints of the line segment
	lineSegment.m_pt0 = p0;
	lineSegment.m_pt1 = p1;

	// Calculate the direction of the line segment
	CSD1130::Vec2 dir = p1 - p0;

	// Calculate the outward-facing normal of the line segment
	// Set the normal of the line segment
	CSD1130::Vector2DNormalize(lineSegment.m_normal, { dir.y, -dir.x });
}

/******************************************************************************/
/*!
 */
/******************************************************************************/
int CollisionIntersection_CircleLineSegment(const Circle &circle,
	const CSD1130::Vec2 &ptEnd,
	const LineSegment &lineSeg,
	CSD1130::Vec2 &interPt,
	CSD1130::Vec2 &normalAtCollision,
	float &interTime,
	bool & checkLineEdges)
{
	// Calculate Velocity vector V and its outward normal M
	CSD1130::Vec2 V = ptEnd - circle.m_center;
	CSD1130::Vec2 M(V.y, -V.x);

	// Calculate N.Bs, N.P0, N.Bs & N.V
	float NBs = CSD1130::Vector2DDotProduct(lineSeg.m_normal, circle.m_center);
	float NP0 = CSD1130::Vector2DDotProduct(lineSeg.m_normal, lineSeg.m_pt0);
	float NV = CSD1130::Vector2DDotProduct(lineSeg.m_normal, V);

	// Bs is starting from the inside half plane, and away from LNS by at least R
	// Here we consider we have an imaginary line LNS1, distant by -R (opposite N direction)
	if (NBs - NP0 <= -circle.m_radius) {
		// Check if the velocity vector V is within the end points of LNS1
		// M is the outward normal to velocity. Compute P0' and P1'
		// To simulate LNS1 line edge points

		// Calculate P0' and P1'
		CSD1130::Vec2 P0prime = lineSeg.m_pt0 - circle.m_radius * lineSeg.m_normal;
		CSD1130::Vec2 P1prime = lineSeg.m_pt1 - circle.m_radius * lineSeg.m_normal;

		// Calculate BsP0' and BsP1'
		CSD1130::Vec2 BsP0prime = P0prime - circle.m_center;
		CSD1130::Vec2 BsP1prime = P1prime - circle.m_center;

		// Calculate M.BsP0'& M.BsP1'
		float MBsP0prime = CSD1130::Vector2DDotProduct(M, BsP0prime);
		float MBsP1prime = CSD1130::Vector2DDotProduct(M, BsP1prime);

		if (MBsP0prime * MBsP1prime < 0) {
			interTime = (NP0 - NBs - circle.m_radius) / (NV); // We are sure N.V != 0
			if (0 <= interTime && interTime <= 1) {
				interPt = circle.m_center + V * interTime;
				normalAtCollision = -lineSeg.m_normal; // Normal at reflection is -N
				return 1;
			}
		}
		else if (checkLineEdges)
			return CheckMovingCircleToLineEdge(false, circle, ptEnd, lineSeg, interPt, normalAtCollision, interTime);
	}

	// Bs is the starting from the outside half plane, and away from LNS by at least R
	// Here we consider we have an imaginary line LNS2 distant by +R (Same N direction)
	else if (NBs - NP0 >= circle.m_radius) {
		// Check if the velocity vector V is within the end points of LNS2
		// M is the outward normal to Velocity V. Compute P0' and P1'

		// Calculate P0' and P1'
		CSD1130::Vec2 P0prime = lineSeg.m_pt0 + circle.m_radius * lineSeg.m_normal;
		CSD1130::Vec2 P1prime = lineSeg.m_pt1 + circle.m_radius * lineSeg.m_normal;

		// Calculate BsP0' and BsP1'
		CSD1130::Vec2 BsP0prime = P0prime - circle.m_center;
		CSD1130::Vec2 BsP1prime = P1prime - circle.m_center;

		// Calculate M.BsP0'& M.BsP1'
		float MBsP0prime = CSD1130::Vector2DDotProduct(M, BsP0prime);
		float MBsP1prime = CSD1130::Vector2DDotProduct(M, BsP1prime);

		if (MBsP0prime * MBsP1prime < 0) {
			interTime = (NP0 - NBs + circle.m_radius) / (NV); // We are sure N.V != 0
			if (0 <= interTime && interTime <= 1) {
				interPt = circle.m_center + V * interTime;
				normalAtCollision = lineSeg.m_normal; // Normal of reflection is N
				return 1;
			}
		}
		else if (checkLineEdges)
			return CheckMovingCircleToLineEdge(false, circle, ptEnd, lineSeg, interPt, normalAtCollision, interTime);
	}

	else if (checkLineEdges) // The circle's starting position Bs, is between both lines LNS1 and LNS2.
		CheckMovingCircleToLineEdge(true, circle, ptEnd, lineSeg, interPt, normalAtCollision, interTime);

	return 0;
}

/******************************************************************************/
/*!
*/
/******************************************************************************/
int CheckMovingCircleToLineEdge(bool withinBothLines,
	const Circle &circle,
	const CSD1130::Vec2 &ptEnd,
	const LineSegment &lineSeg,
	CSD1130::Vec2 &interPt,
	CSD1130::Vec2 &normalAtCollision,
	float &interTime)
{
	/*UNREFERENCED_PARAMETER(withinBothLines);
	UNREFERENCED_PARAMETER(circle);
	UNREFERENCED_PARAMETER(ptEnd);
	UNREFERENCED_PARAMETER(lineSeg);
	UNREFERENCED_PARAMETER(interPt);
	UNREFERENCED_PARAMETER(normalAtCollision);
	UNREFERENCED_PARAMETER(interTime);*/

	// Bs = circle.center
	CSD1130::Vec2 BsP0 = lineSeg.m_pt0 - circle.m_center;
	CSD1130::Vec2 BsP1 = lineSeg.m_pt1 - circle.m_center;
	CSD1130::Vec2 P0P1 = lineSeg.m_pt1 - lineSeg.m_pt0;
	float BsP0P0P1 = CSD1130::Vector2DDotProduct(BsP0, P0P1);

	// Calculate Velocity vector V and its outward normal M
	CSD1130::Vec2 V = ptEnd - circle.m_center;
	CSD1130::Vec2 M(V.y, -V.x);
	//CSD1130::Vector2DNormalize(M, CSD1130::Vector2D(V.y, -V.x));

	// used for calculation later
	float m, s, dist0, dist1;

	if (withinBothLines) { // When it's true, is to say that Bs is starting from between both imaginary lines
		// Check which edge may collide first?
		if (BsP0P0P1 > 0) { // P0 side
			if ((m = CSD1130::Vector2DDotProduct(BsP0, V)) > 0) { // Otherwise no collision
				// Reaching here means the circle movement is facing P0

				// M is normalized outward normal of V
				dist0 = CSD1130::Vector2DDotProduct(BsP0, M); // Same as P0.M - Bs.M (shortest distance from P0 to V)
				if (abs(dist0) > circle.m_radius)
					return 0;
				
				// Reaching here means the circle movement is going towards P0
				// The next line assumes the circle at collision time with P0
				s = sqrt(circle.m_radius * circle.m_radius - dist0 * dist0);
				interTime = (m - s) / CSD1130::Vector2DLength(V);
				if (interTime <= 1) {
					interPt = circle.m_center + V * interTime;

					// Normal of reflection is P0Bi normalized
					CSD1130::Vec2 P0Bi = interPt - lineSeg.m_pt0;
					CSD1130::Vector2DNormalize(normalAtCollision, P0Bi);
					return 1;
				}
			}
		}

		else { //(BsP1.P0P1 < 0) //P1 side
			if ((m = CSD1130::Vector2DDotProduct(BsP1, V)) > 0) { // Otherwise no collision
				// Reaching here means the circle movement is facing P1
				// M is normalized outward normal of V
				dist1 = CSD1130::Vector2DDotProduct(BsP1, M); // Same as P1.M - Bs.M
				if (abs(dist1) > circle.m_radius)
					return 0;

				// Reaching here means the cirlce movement is going towards P1
				// The next line assumes the circle at collision time with P1
				s = sqrt(circle.m_radius * circle.m_radius - dist1 * dist1);
				interTime = (m - s) / CSD1130::Vector2DLength(V);
				if (interTime <= 1) {
					interPt = circle.m_center + V * interTime;

					// Normal of reflection is P1Bi normalized
					CSD1130::Vec2 P1Bi = interPt - lineSeg.m_pt1;
					CSD1130::Vector2DNormalize(normalAtCollision, P1Bi);
					return 1;
				}
			}
		}
	}

	else { // else of: if (withinBothLines)
		// Check which line edge, P0 or P1, is closer to the velocity vector V?
		bool P0Side = false;
		dist0 = CSD1130::Vector2DDotProduct(BsP0, M); // Same as P0.M - Bs.M (M is normalized outward normal of V)
		dist1 = CSD1130::Vector2DDotProduct(BsP1, M); // Same as P1.M - Bs.M

		float dist0_abs = abs(dist0);
		float dist1_abs = abs(dist1);
		if ((dist0_abs > circle.m_radius) && (dist1_abs > circle.m_radius))
			return 0;

		else if ((dist0_abs <= circle.m_radius) && (dist1_abs <= circle.m_radius)) {
			float m0 = CSD1130::Vector2DDotProduct(BsP0, V);
			float m1 = CSD1130::Vector2DDotProduct(BsP1, V);
			float m0_abs = abs(m0);
			float m1_abs = abs(m1);
			P0Side = (m0_abs < m1_abs);
		}
		else if (dist0_abs <= circle.m_radius)
			P0Side = true;
		else
			P0Side = false;

		if (P0Side) { // circle is closer to P0
			if ((m = CSD1130::Vector2DDotProduct(BsP0, V)) < 0)
				return 0; // moving away
			else {
				// Reaching here means the circle movement is going towards P0
				// The next line assumes the circle at collision time with P0
				s = sqrt(circle.m_radius * circle.m_radius - dist0 * dist0);
				interTime = (m - s) / CSD1130::Vector2DLength(V);
				if (interTime <= 1) {
					interPt = circle.m_center + V * interTime;

					// Normal of reflection is P0Bi normalized
					CSD1130::Vec2 P0Bi = interPt - lineSeg.m_pt0;
					CSD1130::Vector2DNormalize(normalAtCollision, P0Bi);
					return 1;
				}
			}
		}

		else { // circle is closer to P1
			if ((m = CSD1130::Vector2DDotProduct(BsP1, V)) < 0)
				return 0; // moving away
			else {
				// Reaching here means the circle movement is going towards P1
				// The next line assumes the circle at collision time with P1
				s = sqrt(circle.m_radius * circle.m_radius - dist1 * dist1);
				interTime = (m - s) / CSD1130::Vector2DLength(V);
				if (interTime <= 1) {
					interPt = circle.m_center + V * interTime;

					// Normal of reflection is P1Bi normalized
					CSD1130::Vec2 P1Bi = interPt - lineSeg.m_pt1;
					CSD1130::Vector2DNormalize(normalAtCollision, P1Bi);
					return 1;
				}
			}
		}
	}
}





/******************************************************************************/
/*!
 */
/******************************************************************************/
void CollisionResponse_CircleLineSegment(const CSD1130::Vec2 &ptInter,
	const CSD1130::Vec2 &normal,
	CSD1130::Vec2 &ptEnd,
	CSD1130::Vec2 &reflected)
{
	// Calculate penetration vector
	CSD1130::Vec2 penetration = ptEnd - ptInter;

	// Calculate reflection
	CSD1130::Vec2 r = penetration - 2.0f * (CSD1130::Vector2DDotProduct(penetration, normal) * normal);
	ptEnd = ptInter + r;

	// Normalize reflection
	CSD1130::Vector2DNormalize(reflected, r);
}



