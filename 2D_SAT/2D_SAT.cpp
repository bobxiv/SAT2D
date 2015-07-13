#include "SFML//Graphics.hpp"
#include "SFML//System.hpp"
#include "SFML//Window.hpp"
#include "SFML//Config.hpp"

#include "Game.h"


int WinMain(HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     char*     lpCmdLine,
                     int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	Game MyGame;

	return MyGame.Go();
}
