/******************************************************************************/
/*!
\file		GameState_Cage.h
\author 	Guo Yiming, yiming.guo, 2202613
\par    	email: yiming.guo@digipen.edu
\date   	Mar 18, 2023
\brief		Declares Load, Init, Update, Draw, Free and Unload functions for
			Cage Game State.

Copyright (C) 2023 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the
prior written consent of DigiPen Institute of Technology is prohibited.
 */
 /******************************************************************************/

#ifndef CSD1130_GAME_STATE_PLAY_H_
#define CSD1130_GAME_STATE_PLAY_H_


// ---------------------------------------------------------------------------

void GameStateCageLoad(void);
void GameStateCageInit(void);
void GameStateCageUpdate(void);
void GameStateCageDraw(void);
void GameStateCageFree(void);
void GameStateCageUnload(void);

// ---------------------------------------------------------------------------

#endif // CSD1130_GAME_STATE_PLAY_H_