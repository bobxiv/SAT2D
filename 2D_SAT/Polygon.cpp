#include "Polygon.h"

namespace SATGame
{

Polygon::Polygon(): m_z(0.0f, 0.0f, 1.0f), m_Color(sf::Color::White)
{
	m_Text.SetPosition(0,0);
}

Polygon::Polygon(sf::Color color): m_z(0.0f, 0.0f, 1.0f), m_Color(color)
{
	m_Text.SetPosition(0,0);
}

void Polygon::CheckCollision(std::vector<Polygon*> possibleColliding)
{

	for(int i=0; i < possibleColliding.size() ; ++i)
		if( SAT_Polygon(*this , *possibleColliding[i]) )
			ResolveCollision(m_PolygonShape.GetCenter() - possibleColliding[i]->m_PolygonShape.GetCenter());
		else
			m_Text.SetText("Not Colliding");

}

void Polygon::ResolveCollision(const sf::Vector2<float>& Normal)
{
	m_Text.SetText("Colliding");
}

bool Polygon::IsInside(const sf::Vector2<float>& Point)
{
	if( m_Points.size() < 3 )//if 0, 1 or 2 We don't consider that we can have a point inside
		return false;

	for(int i=0; i < (m_Points.size()-1) ; ++i)
	{
		//sf::Vector2<float> line = m_PolygonShape.GetPointPosition(i+1)-m_PolygonShape.GetPointPosition(i);
		sf::Vector2<float> B = m_Points[i+1];
		sf::Vector2<float> A = m_Points[i];
		A.y *= -1.0f;
		B.y *= -1.0f;
		sf::Vector2<float> line = B-A;

		//sf::Vector3<float> res = Cross( sf::Vector3<float>(line.x,line.y,0.0f), sf::Vector3<float>(Point.x,-Point.y,0.0f));
		sf::Vector3<float> res = Cross( sf::Vector3<float>(line.x,line.y,0.0f), sf::Vector3<float>(Point.x-A.x,-Point.y-A.y,0.0f));
		
		if( res.z > 0 )
			return false;//Definitely not inside
	}

	return true;//Inside
}

void Polygon::Draw(sf::RenderWindow& Wnd)
{
	sf::Shape poly;
	poly.SetColor(m_Color);
	for(int i=0; i < m_Points.size() ; ++i)
		poly.AddPoint(m_Points[i]);
	//Wnd.Draw(m_PolygonShape);
	Wnd.Draw(poly);

	for(int i=0; i < m_NormalsVector.size() ; ++i)
		_DrawArrow(Wnd, m_NormalsVector[i].first, m_NormalsVector[i].second, sf::Color(255,255,255,128), 30.0f);

	Wnd.Draw(m_Text);
}

void Polygon::_DrawArrow(sf::RenderWindow& RW, sf::Vector2<float>& Vec1, sf::Vector2<float>& Vec2, sf::Color color, float scale)
{
	sf::Shape DirectionVector = sf::Shape::Line(Vec1, Vec1+Vec2*scale, 2, color, 1.0f, sf::Color::Yellow);
	RW.Draw(DirectionVector);

	float length = scale;
	
	sf::Vector3<float> aux = Cross( sf::Vector3<float>(Vec2.x,Vec2.y,0), sf::Vector3<float>(0,0,1) );
	sf::Vector2<float> left( aux.x, aux.y);
	sf::Vector2<float> invVec = Vec2*(-1.0f);
	sf::Vector2<float> diag = (left+invVec)/Norm(left+invVec);

	sf::Shape ArrowLeft = sf::Shape::Line(Vec1+Vec2*scale, Vec1+Vec2*scale+diag*(length/6), 2, color, 1.0f, sf::Color::Yellow);
	RW.Draw(ArrowLeft);

	aux = Cross( sf::Vector3<float>(0,0,1) , sf::Vector3<float>(Vec2.x,Vec2.y,0) );
	sf::Vector2<float> right( aux.x, aux.y);
	diag = (right+invVec)/Norm(right+invVec);

	sf::Shape ArrowRight = sf::Shape::Line(Vec1+Vec2*scale, Vec1+Vec2*scale+diag*(length/6), 2, color, 1.0f, sf::Color::Yellow);
	RW.Draw(ArrowRight);
}

void Polygon::AddPoint(sf::Vector2<float>& Point, sf::Color Color )
{
	//m_PolygonShape.AddPoint(Point.x,Point.y, Color);
	//Add Point
	m_Points.push_back(Point);

	//Update Center
	m_Center.x = 0.0f; m_Center.y = 0.0f;
	for(int i=0; i < m_Points.size() ; ++i)
		m_Center += m_Points[i];
	m_Center /= (float)m_Points.size();

	m_NormalsVector.clear();
	//Update Normals
	for(int i=0; i < m_Points.size() ; ++i)
	{
		sf::Vector2<float> line = m_Points[(i+1)%m_Points.size()]-m_Points[i];
		sf::Vector3<float> n_aux = Cross( sf::Vector3<float>(line.x, line.y, 0) , m_z );
		sf::Vector2<float> n(n_aux.x, n_aux.y);
		n /= Norm( n );

		sf::Vector2<float> middle = m_Points[(i+1)%m_Points.size()]+m_Points[i];
		middle /= 2.0f;
		m_NormalsVector.push_back( std::pair<sf::Vector2<float>, sf::Vector2<float>>(middle , n));
	}

}

void Polygon::SetPosition(sf::Vector2<float>& Position)
{
	m_PolygonShape.SetPosition(Position);

	sf::Vector2<float> displacement = Position-m_Center;
	//Displace Points
	for( int i=0; i < m_Points.size() ; ++i )
		m_Points[i] += displacement;

	//Displace Normals
	for( int i=0; i < m_NormalsVector.size() ; ++i )
		m_NormalsVector[i].first += displacement;

	//Set new center
	m_Center = Position;

}

void Polygon::SetCenter(sf::Vector2<float>& Center)
{
	//m_PolygonShape.SetCenter( m_PolygonShape.TransformToLocal(Center) );
	m_Center = Center;
}

bool Polygon::SAT_Polygon(const Polygon& A, const Polygon& B)
{
	if( A.m_Points.size() == 0 || B.m_Points.size() == 0 )
		return false;

	sf::Vector3<float> z(0,0,1);

	sf::Vector2<float> C_A;
	sf::Vector2<float> C_B;

	#pragma region Center Calculation

		for(int i=0; i < A.m_Points.size() ; ++i)
		{
			C_A += A.m_Points[i];
		}
		C_A *= 1/(float)A.m_Points.size();

		for(int i=0; i < B.m_Points.size() ; ++i)
		{
			C_B += B.m_Points[i];
		}
		C_B *= 1/(float)B.m_Points.size();

	#pragma endregion

	//Test Faces F: there are 2*(F_1+F_2) tests
	for(int i=0; i < A.m_Points.size() ; ++i)
	{
		sf::Vector2<float> line = A.m_Points[(i+1)%A.m_Points.size()]-A.m_Points[i];
		
		sf::Vector3<float> n_aux = Cross( sf::Vector3<float>(line.x, line.y, 0) , z );
		sf::Vector2<float> n(n_aux.x, n_aux.y);
		n /= Norm( n );

		float r = 0;
		for(int j=0; j < B.m_Points.size() ; ++j)
		{
			sf::Vector2<float> line = B.m_Points[(j+1)%B.m_Points.size()]-B.m_Points[j];
			r += abs(Dot(line, n));
		}

		for(int j=0; j < A.m_Points.size() ; ++j)
		{
			sf::Vector2<float> line = A.m_Points[(j+1)%A.m_Points.size()]-A.m_Points[j];
			r += abs(Dot(line, n));
		}

		r /= 4.0f;

/*
		float projC_A = Dot( C_A, n);

		float projC_B = Dot( C_B, n);

		if( abs(projC_A - projC_B) > r )
			return false;//Not Colliding
*/
		float projC_Dif = Dot( (C_A-C_B), n);

		if( abs(projC_Dif) > r )
			return false;//Not Colliding
	}


	//Calculos para caras de B
	for(int i=0; i < B.m_Points.size() ; ++i)
	{
		sf::Vector2<float> line = B.m_Points[(i+1)%B.m_Points.size()]-B.m_Points[i];
		
		sf::Vector3<float> n_aux = Cross( sf::Vector3<float>(line.x, line.y, 0) , z );
		sf::Vector2<float> n(n_aux.x, n_aux.y);
		n /= Norm( n );

		float r = 0;
		for(int j=0; j < B.m_Points.size() ; ++j)
		{
			sf::Vector2<float> line = B.m_Points[(j+1)%B.m_Points.size()]-B.m_Points[j];
			r += abs(Dot(line, n));
		}

		for(int j=0; j < A.m_Points.size() ; ++j)
		{
			sf::Vector2<float> line = A.m_Points[(j+1)%A.m_Points.size()]-A.m_Points[j];
			r += abs(Dot(line, n));
		}

		r /= 4.0f;

/*
		float projC_A = Dot( C_A, n);

		float projC_B = Dot( C_B, n);

		if( abs(projC_A - projC_B) > r )
			return false;//Not Colliding
*/
		float projC_Dif = Dot( (C_A-C_B), n);

		if( abs(projC_Dif) > r )
			return false;//Not Colliding
	}



	return true;//Colliding
}

}