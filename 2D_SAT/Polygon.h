#pragma once

#include "SFML//Graphics.hpp"
#include "SFML//System.hpp"
#include "SFML//Window.hpp"
#include "SFML//Config.hpp"

#include <vector>

#include "Math.h"

namespace SATGame
{

	class Polygon
	{
	private:

		sf::Shape m_PolygonShape;

		std::vector<sf::Vector2<float>> m_Points;

		//First element Middle Point of segment, Second element displacement from middle(ie normal)
		std::vector<std::pair<sf::Vector2<float>, sf::Vector2<float>>> m_NormalsVector;

		sf::Vector2<float> m_Center;

		sf::Vector3<float> m_z;

		sf::Color m_Color;

		static bool SAT_Polygon(const Polygon& A, const Polygon& B);

		void _DrawArrow(sf::RenderWindow& RW, sf::Vector2<float>& Vec1, sf::Vector2<float>& Vec2, sf::Color color, float scale);

		sf::String m_Text;

	public:

		Polygon();

		Polygon(sf::Color color);

		void CheckCollision(std::vector<Polygon*> possibleColliding);

		void ResolveCollision(const sf::Vector2<float>& Normal);

		bool IsInside(const sf::Vector2<float>& Point);

		void Draw(sf::RenderWindow& Wnd);

		void AddPoint(sf::Vector2<float>& Point, sf::Color Color );

		void SetPosition(sf::Vector2<float>& Position);

		void SetCenter(sf::Vector2<float>& Center);

	};

}