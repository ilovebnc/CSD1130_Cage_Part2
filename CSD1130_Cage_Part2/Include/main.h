/******************************************************************************/
/*!
\file		main.h
\author 	Guo Yiming, yiming.guo, 2202613
\par    	email: yiming.guo@digipen.edu
\date   	Mar 18, 2023
\brief		Globals and headers for game entry point

Copyright (C) 2023 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the
prior written consent of DigiPen Institute of Technology is prohibited.
 */
 /******************************************************************************/


#ifndef CSD1130_MAIN_H_
#define CSD1130_MAIN_H_

//------------------------------------
// Globals

extern float	g_dt;
extern double	g_appTime;

// ---------------------------------------------------------------------------
// includes

#include "AEEngine.h"
#include "Math.h"
#include "Vector2D.h"
#include "Matrix3x3.h"

#include <iostream>
#include <fstream>
#include <string>

#include "GameStateMgr.h"
#include "GameState_Cage.h"
#include "Collision.h"


extern s8	fontId;


#endif // CSD1130_MAIN_H_










