#pragma once

//#include "SFML//Graphics.hpp"
//#include "SFML//System.hpp"
//#include "SFML//Window.hpp"
//#include "SFML//Config.hpp"

#include "Polygon.h"

#include "RigidBody.h"
#include "Collider.h"
#include "QuadTree.h"

class Game
{
private:
	
	SATGame::Polygon m_Polygon1;
	SATGame::Polygon m_Polygon2;

	//::Obj2D::Polygon m_novoPoly;
	std::vector<::Obj2D::Polygon*> m_novoPolys;

	sf::RenderWindow m_MainWnd;

	const sf::Input& m_Input;
	
	sf::Event        m_Msg;

	Obj2D::QuadTree  m_QT;

	bool m_Draging1;
	bool m_Draging2;
	std::vector<bool> m_Draging;

	void DrawGame();

	void UpdateGame();

public:
	Game();

	~Game();

	int Go();

};