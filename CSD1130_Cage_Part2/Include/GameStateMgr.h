/******************************************************************************/
/*!
\file		GameStateMgr.h
\author 	Guo Yiming, yiming.guo, 2202613
\par    	email: yiming.guo@digipen.edu
\date   	Mar 18, 2023
\brief		Defines global variables gGameStateInit, gGameStateCurr,
			gGameStatePrev and gGameStateNext.
			
			Defines global function pointers to GameStateLoad, GameStateInit,
			GameStateUpdate, GameStateDraw, GameStateFree, GameStateUnload.

			Declares GameStateMgrInit and GameStateMgrUpdate functions.

Copyright (C) 2023 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the
prior written consent of DigiPen Institute of Technology is prohibited.
 */
 /******************************************************************************/

#ifndef CSD1130_GAME_STATE_MGR_H_
#define CSD1130_GAME_STATE_MGR_H_

// ---------------------------------------------------------------------------

#include "AEEngine.h"

// ---------------------------------------------------------------------------
// include the list of game states

#include "GameStateList.h"

// ---------------------------------------------------------------------------
// externs

extern GS_STATE gGameStateInit;
extern GS_STATE gGameStateCurr;
extern GS_STATE gGameStatePrev;
extern GS_STATE gGameStateNext;

// ---------------------------------------------------------------------------

extern void (*GameStateLoad)();
extern void (*GameStateInit)();
extern void (*GameStateUpdate)();
extern void (*GameStateDraw)();
extern void (*GameStateFree)();
extern void (*GameStateUnload)();

// ---------------------------------------------------------------------------
// Function prototypes

// call this at the beginning and AFTER all game states are added to the manager
void GameStateMgrInit(GS_STATE gameStateInit);

// update is used to set the function pointers
void GameStateMgrUpdate();

// ---------------------------------------------------------------------------

#endif // CSD1130_GAME_STATE_MGR_H_