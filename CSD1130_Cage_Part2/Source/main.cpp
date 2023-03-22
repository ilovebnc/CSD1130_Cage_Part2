/******************************************************************************/
/*!
\file		main.cpp
\author 	Guo Yiming, yiming.guo, 2202613
\par    	email: yiming.guo@digipen.edu
\date   	Mar 18, 2023
\brief		Game entry point			

Copyright (C) 2023 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the
prior written consent of DigiPen Institute of Technology is prohibited.
 */
 /******************************************************************************/

#include "main.h"
#include <memory>


// ---------------------------------------------------------------------------
// Globals
float	 g_dt = 0.01667f;
double	 g_appTime;

s8	fontId = 0;

/******************************************************************************/
/*!
	Starting point of the application
*/
/******************************************************************************/
int WINAPI WinMain(_In_ HINSTANCE instanceH, _In_opt_ HINSTANCE prevInstanceH, _In_ LPSTR command_line, _In_ int show)
{
	// Enable run-time memory check for debug builds.
	#if defined(DEBUG) | defined(_DEBUG)
		_CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
	#endif

	UNREFERENCED_PARAMETER(prevInstanceH);
	UNREFERENCED_PARAMETER(command_line);
	// Initialize the system
	AESysInit(instanceH, show, 1024, 768, 1, 60, false, nullptr);

	//Fonts Assets
	fontId = AEGfxCreateFont("..\\Bin\\Resources\\Fonts\\Arial Italic.ttf", 28);

	GameStateMgrInit(GS_STATE::GS_CAGE);

	while(gGameStateCurr != GS_STATE::GS_QUIT)
	{
		// reset the system modules
		AESysReset();

		// If not restarting, load the gamestate
		if(gGameStateCurr != GS_STATE::GS_RESTART)
		{
			GameStateMgrUpdate();
			GameStateLoad();
		}
		else
			gGameStateNext = gGameStateCurr = gGameStatePrev;

		// Initialize the gamestate
		GameStateInit();

		while(gGameStateCurr == gGameStateNext)
		{
			AESysFrameStart();

			AEInputUpdate();

			
			GameStateUpdate();

			GameStateDraw();
			

			AESysFrameEnd();

			// check if forcing the application to quit
			if ((AESysDoesWindowExist() == false) || AEInputCheckTriggered(AEVK_ESCAPE))
				gGameStateNext = GS_STATE::GS_QUIT;

			g_dt = (f32)AEFrameRateControllerGetFrameTime();

			//hack
			if (g_dt > 0.01667f)
				g_dt = 0.01667f;
			
			g_appTime += g_dt;
		}
		
		GameStateFree();

		if(gGameStateNext != GS_STATE::GS_RESTART)
			GameStateUnload();

		gGameStatePrev = gGameStateCurr;
		gGameStateCurr = gGameStateNext;
	}

	AEGfxDestroyFont(fontId);

	// free the system
	AESysExit();
}