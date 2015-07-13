#include "Game.h"

#include <vector>


Game::Game(): m_MainWnd(sf::VideoMode(800,600,32), "SAT"), m_Input(m_MainWnd.GetInput()), m_Draging1(false), m_Draging2(false),
m_Polygon1(sf::Color::Blue), m_Polygon2(sf::Color::Red), m_QT(1, 0, 0, 800, 600)
{
	//m_MainWnd.SetFramerateLimit(9);

	Obj2D::ForceDesc gravity;
	Obj2D::ForceDesc gravityHard;
	gravity.Force.x = 0.0f;
	gravityHard.Force.x = 0.0f;
	//gravity.Force.y = 9.80f*50.0f;
	gravity.Force.y = 0.25f;
	gravityHard.Force.y = 0.5f;
	//gravity.Force.y = 0.7f;
	
	for(int i=0; i < 2 ; ++i)
	{
		::Obj2D::Polygon* pPoly = new ::Obj2D::Polygon();
		pPoly->AddVertex(sf::Vector2<float>(-50,-50));

		pPoly->AddVertex(sf::Vector2<float>(50,-50));

		pPoly->AddVertex(sf::Vector2<float>(50,50));

		if( i==0 )
			pPoly->AddVertex(sf::Vector2<float>(-50,50));

		m_Draging.push_back(false);

		pPoly->SetWorldLimit(0,0,800,600);

		if( i==0 )
			pPoly->m_Forces.push_back(gravityHard);
		else
			pPoly->m_Forces.push_back(gravity);
		//pPoly->SetPosition( sf::Vector2<float>( rand()%400 , rand()%300 ));
		pPoly->SetPosition( sf::Vector2<float>( (rand()%700)+50 , (rand()%500)+50) );
		m_novoPolys.push_back(pPoly);

		m_QT.AddElement(pPoly);
	}

	int count = m_QT.GetElementCount();
	
}

Game::~Game()
{}


void Game::UpdateGame()
{
	for(int i=0; i < m_novoPolys.size() ; ++i)
		m_novoPolys[i]->Update(m_MainWnd.GetFrameTime()*1000);//time in miliseconds

	std::vector<Obj2D::SpatialNode*> nodes;
	for(int i=0; i < m_novoPolys.size() ; ++i)
		nodes.push_back(m_novoPolys[i]);
	//std::copy(m_novoPolys.begin(), m_novoPolys.end(), nodes.begin());
	m_QT.Construct(nodes);

	::Obj2D::Collider col;
	//col.AllVsAllCollide(m_novoPolys);
	col.NearVsNearCollide(m_novoPolys, m_QT);

	std::vector<SATGame::Polygon*> aux;
	aux.push_back(&m_Polygon2);
	m_Polygon1.CheckCollision(aux);

	aux.clear();
	aux.push_back(&m_Polygon1);
	m_Polygon2.CheckCollision(aux);
}

void Game::DrawGame()
{
	m_QT.DebugRender(m_MainWnd);

	for(int i=0; i < m_novoPolys.size() ; ++i)
		m_novoPolys[i]->Draw(m_MainWnd);

	m_Polygon1.Draw(m_MainWnd);

	m_Polygon2.Draw(m_MainWnd);
}

int Game::Go()
{
	while( m_MainWnd.IsOpened() )
	{
		while( m_MainWnd.GetEvent(m_Msg) )
		{
			
			if( m_Msg.Type == sf::Event::MouseMoved && m_Draging1 )
				m_Polygon1.SetPosition( sf::Vector2<float>(m_Msg.MouseMove.X , m_Msg.MouseMove.Y) );

			if( m_Msg.Type == sf::Event::MouseMoved && m_Draging2 )
				m_Polygon2.SetPosition( sf::Vector2<float>(m_Msg.MouseMove.X , m_Msg.MouseMove.Y) );

			for(int i=0; i < m_novoPolys.size() ; ++i)
				if( m_Msg.Type == sf::Event::MouseMoved && m_Draging[i] )
					m_novoPolys[i]->SetPosition( sf::Vector2<float>(m_Msg.MouseMove.X , m_Msg.MouseMove.Y) );

			if( m_Msg.Type == sf::Event::MouseButtonPressed && (m_Msg.MouseButton.Button == 1) )
			{
				if( m_Polygon1.IsInside( sf::Vector2<float>(m_Input.GetMouseX(), m_Input.GetMouseY()) ) )
				{
					m_Draging1 = true;
					m_Polygon1.SetCenter( sf::Vector2<float>(m_Input.GetMouseX(), m_Input.GetMouseY()) );
				}

				if( m_Polygon2.IsInside( sf::Vector2<float>(m_Input.GetMouseX(), m_Input.GetMouseY()) ) )
				{
					m_Draging2 = true;
					m_Polygon2.SetCenter( sf::Vector2<float>(m_Input.GetMouseX(), m_Input.GetMouseY()) );
				}

				Obj2D::Collider col;
				for(int i=0; i < m_novoPolys.size() ; ++i)
					if( col.TestPointVsPolygon( sf::Vector2<float>(m_Input.GetMouseX(), m_Input.GetMouseY()), *m_novoPolys[i] ) )
						m_Draging[i] = true;

			}

			if( m_Msg.Type == sf::Event::MouseButtonReleased && (m_Msg.MouseButton.Button == 1) )
			{
				m_Draging1 = false;
				m_Draging2 = false;

				for(int i=0; i < m_novoPolys.size() ; ++i)
					m_Draging[i] = false;
			}

			if( m_Msg.Type == sf::Event::MouseButtonReleased && (m_Msg.MouseButton.Button == 0) && !m_Input.IsKeyDown(sf::Key::LShift) )
				m_Polygon1.AddPoint( sf::Vector2<float>(m_Input.GetMouseX(),m_Input.GetMouseY()) , sf::Color::Red );
			

			if( m_Msg.Type == sf::Event::MouseButtonReleased && (m_Msg.MouseButton.Button == 0) && m_Input.IsKeyDown(sf::Key::LShift) )
				m_Polygon2.AddPoint( sf::Vector2<float>(m_Input.GetMouseX(),m_Input.GetMouseY()) , sf::Color::Blue );

			if( m_Msg.Type == sf::Event::Closed )
				m_MainWnd.Close();
		}

		UpdateGame();

		m_MainWnd.Clear();
		
		DrawGame();

		m_MainWnd.Display();
	}

	return 0;
}