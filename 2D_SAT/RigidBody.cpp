#include "RigidBody.h"

#include "Collider.h"

#include "Math.h"

sf::Vector3<float> AxisZ(0.0f, 0.0f, 1.0f);

namespace Obj2D
{

	/////////////////////////////////////////////////////////////////////////////////
	///////////////////////     SpatialNode Implementation      /////////////////////
	/////////////////////////////////////////////////////////////////////////////////

	SpatialNode::SpatialNode(): m_Position(0.0f,0.0f)
	{}

	SpatialNode::~SpatialNode()
	{}

	sf::Vector2<float> SpatialNode::GetPosition() const
	{
		return m_Position;
	}

	void SpatialNode::SetPosition(const sf::Vector2<float> &newPosition)
	{
		m_Position = newPosition;
	}

	void SpatialNode::Move(const sf::Vector2<float> &movement)
	{
		m_Position += movement;
	}

	/////////////////////////////////////////////////////////////////////////////////
	///////////////////////      RigidBody Implementation       /////////////////////
	/////////////////////////////////////////////////////////////////////////////////

	RigidBody::RigidBody(): m_MassDensity(1.0f), m_Mass(1.0f), m_RotationalInertia(1.0f), m_Rotation(0.0f), m_Velocity(0.0f,0.0f),
	m_RotationVelocity(0.0f), m_Restitution(1.0f)
	{
		//OnCollision = RigidBody::OnCollisionImpulseRotation;
		OnCollision = RigidBody::OnCollisionImpulseNoRotation;
		//OnCollision = RigidBody::OnCollisionMine;
	}

	float RigidBody::GetMassDensity() const
	{
		return m_MassDensity;
	}

	void RigidBody::_ReCalcMass()
	{
		m_Mass = GetArea()*m_MassDensity;
	}

	float RigidBody::GetMass() const
	{
		return m_Mass;
	}

	float RigidBody::GetRotationalInertia() const
	{
		return m_RotationalInertia;
	}

	float RigidBody::GetRotation() const
	{
		return m_Rotation;
	}

	sf::Vector2<float> RigidBody::GetVelocity() const
	{
		return m_Velocity;
	}

	float RigidBody::GetRotationVelocity() const
	{
		return m_RotationVelocity;
	}

	void RigidBody::Update(float elapsedTime)
	{
		sf::Vector2<float> totForce;
		for(int i=0; i < m_Forces.size() ;++i)
			totForce += m_Forces[i].Force;

		sf::Vector3<float> Torque;
		for(int i=0; i < m_Forces.size() ;++i)
			Torque += Cross( sf::Vector3<float>(m_Forces[i].Force.x, m_Forces[i].Force.y, 0.0f), sf::Vector3<float>(m_Forces[i].AppliedPoint.x, m_Forces[i].AppliedPoint.y, 0.0f) );
		float TorqueAcel = Norm(Torque);
		if( Torque.z < 0.0f )
			TorqueAcel *= (-1);


		if( Norm(m_Velocity) != 0.0f )
			totForce -= m_Velocity/Norm(m_Velocity)*0.1f;//Linear Friction
			//totForce -= m_Velocity/Norm(m_Velocity)*Dot(m_Velocity,m_Velocity)*this->GetMass();//Linear Friction
		if( m_RotationVelocity != 0.0f )
			TorqueAcel -= m_RotationVelocity/m_RotationVelocity*(-1.0f);//Rotational Friction


		m_Velocity += (totForce/this->GetMass())*elapsedTime;//Linear Force

		m_Position += m_Velocity*elapsedTime;//Linear Movemente

		m_RotationVelocity += (TorqueAcel/this->GetRotationalInertia())*elapsedTime;//Linear Force

		m_Rotation += m_RotationVelocity*elapsedTime;//Rotational Movemente

		//The transformation matrix
		m_Transform.SetFromTransformations(m_Center, m_Position, m_Rotation, sf::Vector2<float>(1.0f,1.0f));
	}

	void RigidBody::SetVelocity(const sf::Vector2<float>& newVelocity)
	{
		m_Velocity = newVelocity;
	}

	void RigidBody::SetRestitution(float newRestitution)
	{
		m_Restitution = newRestitution;
	}

	void RigidBody::OnCollisionImpulseNoRotation(RigidBody* our, RigidBody* their, const CollisionInfo& info)
	{
		#ifdef _DEBUG
		if( info.CollisionSurface.Type == CollisionContact::Point )
		{
			//Draw the Colision Point
			RigidBody::TemporalDebugShape shapeColPoint(3.0f,sf::Shape::Circle(info.CollisionSurface.Point1.x, info.CollisionSurface.Point1.y, 5.0f, sf::Color::Red));
			our->m_DebugShapesForFrame.push_back( shapeColPoint );
			RigidBody::TemporalDebugShape shapeNormal(3.0f,sf::Shape::Line(info.CollisionSurface.Point1.x, info.CollisionSurface.Point1.y, info.CollisionSurface.Point1.x+info.Normal.x*25.0f, info.CollisionSurface.Point1.y+info.Normal.y*25.0f, 5.0f, sf::Color::Magenta));
			our->m_DebugShapesForFrame.push_back( shapeNormal );
		}else
		{
			//Draw the Colision Edge
			RigidBody::TemporalDebugShape shapeColLine(3.0f,sf::Shape::Line(info.CollisionSurface.Point1.x, info.CollisionSurface.Point1.y, info.CollisionSurface.Point2.x, info.CollisionSurface.Point2.y, 5.0f, sf::Color::Red));
			our->m_DebugShapesForFrame.push_back( shapeColLine );
			RigidBody::TemporalDebugShape shapeNormal(3.0f,sf::Shape::Line((info.CollisionSurface.Point1.x+info.CollisionSurface.Point2.x)/2.0f, (info.CollisionSurface.Point1.y+info.CollisionSurface.Point2.y)/2.0f, (info.CollisionSurface.Point1.x+info.CollisionSurface.Point2.x)/2.0f+info.Normal.x*25.0f, (info.CollisionSurface.Point1.y+info.CollisionSurface.Point2.y)/2.0f+info.Normal.y*25.0f, 5.0f, sf::Color::Magenta));
			our->m_DebugShapesForFrame.push_back( shapeNormal );
		}
		#endif
		//Newton’s Law of Restitution for Instantaneous Collisions with No Friction
		//With Linear Impulse
		//Deduced from:
		//				1) Definition of Restitution:
		//				   (v_a' - v_b') * n = -e *(v_a - v_b) * n
		//
		//				   v_ap = v_a
		//											- v_ap means velocity of point P of object
		//											  A, this gets only linear velocity(in this model)
		//
		//				2) Third Newton Law:
		//				   v_a' = v_a + (j/M_a) * n
		//				   v_b' = v_b - (j/M_b) * n
		//
		//											- ' is after collision
		//											- n is normal of a

		sf::Vector2<float> v_AB = our->m_Velocity - their->m_Velocity;

		float jNum = -(1+our->m_Restitution)*Dot(v_AB,info.Normal);
		float jDen = Dot(info.Normal,info.Normal)*(1.0f/our->m_Mass+1.0f/their->m_Mass);

		float j = jNum/jDen;

		our->m_Velocity = our->m_Velocity + (j/our->m_Mass)* info.Normal;
		//our->m_Velocity = (j/our->m_Mass)* info.Normal;
	}

	void RigidBody::OnCollisionImpulseRotation(RigidBody* our, RigidBody* their, const CollisionInfo& info)
	{
		#ifdef _DEBUG
		if( info.CollisionSurface.Type == CollisionContact::Point )
		{
			//Draw the Colision Point
			RigidBody::TemporalDebugShape shapeColPoint(3.0f,sf::Shape::Circle(info.CollisionSurface.Point1.x, info.CollisionSurface.Point1.y, 5.0f, sf::Color::Red));
			our->m_DebugShapesForFrame.push_back( shapeColPoint );
			RigidBody::TemporalDebugShape shapeNormal(3.0f,sf::Shape::Line(info.CollisionSurface.Point1.x, info.CollisionSurface.Point1.y, info.CollisionSurface.Point1.x+info.Normal.x*25.0f, info.CollisionSurface.Point1.y+info.Normal.y*25.0f, 5.0f, sf::Color::Magenta));
			our->m_DebugShapesForFrame.push_back( shapeNormal );
		}else
		{
			//Draw the Colision Edge
			RigidBody::TemporalDebugShape shapeColLine(3.0f,sf::Shape::Line(info.CollisionSurface.Point1.x, info.CollisionSurface.Point1.y, info.CollisionSurface.Point2.x, info.CollisionSurface.Point2.y, 5.0f, sf::Color::Red));
			our->m_DebugShapesForFrame.push_back( shapeColLine );
			RigidBody::TemporalDebugShape shapeNormal(3.0f,sf::Shape::Line((info.CollisionSurface.Point1.x+info.CollisionSurface.Point2.x)/2.0f, (info.CollisionSurface.Point1.y+info.CollisionSurface.Point2.y)/2.0f, (info.CollisionSurface.Point1.x+info.CollisionSurface.Point2.x)/2.0f+info.Normal.x*25.0f, (info.CollisionSurface.Point1.y+info.CollisionSurface.Point2.y)/2.0f+info.Normal.y*25.0f, 5.0f, sf::Color::Magenta));
			our->m_DebugShapesForFrame.push_back( shapeNormal );
		}
		#endif
		//Newton’s Law of Restitution for Instantaneous Collisions with No Friction
		//With Linear and Angular Impulse
		//Deduced from:
		//				1) Definition of Restitution:
		//				   (v_ap' - v_bp') * n = -e *(v_ap - v_bp) * n
		//
		//				   v_ap = v_a + w_a * Perp(r_a)
		//											- v_ap means velocity of point P of object
		//											  A, this gets linear and angular velocities
		//
		//				2) Third Newton Law:
		//				   v_a' = v_a + (j/M_a) * n
		//				   v_b' = v_b - (j/M_b) * n
		//				   w_a' = w_a + (j/I_a) * Perp(r_a) * n
		//				   w_b' = w_b - (j/I_b) * Perp(r_b) * n
		//
		//											- ' is after collision
		//											- n is normal of a
		//											- Perp is an operator that gets the 
		//											  perpendicular of a vector

		//Gets only 1 representative collision point from the collision surface
		sf::Vector2<float> ColisionPoint;
		if( info.CollisionSurface.Type == CollisionContact::Point )
			ColisionPoint = info.CollisionSurface.Point1;
		else
			ColisionPoint = (info.CollisionSurface.Point1 + info.CollisionSurface.Point2)/2.0f;

		sf::Vector2<float> v_AB = our->m_Velocity - their->m_Velocity;

		//sf::Vector2<float> r_AP = our->m_Position   - ColisionPoint;
		//sf::Vector2<float> r_BP = their->m_Position - ColisionPoint;
		sf::Vector2<float> r_APaux = ColisionPoint - our->m_Position;
		sf::Vector2<float> r_BPaux = ColisionPoint - their->m_Position;
		sf::Vector2<float> r_AP(r_APaux.y, -r_APaux.x);
		sf::Vector2<float> r_BP(r_BPaux.y, -r_BPaux.x);

		float jNum = -(1+our->m_Restitution)*Dot(v_AB,info.Normal);
		float aux1 = Dot(r_AP,info.Normal);
		aux1 *= aux1;
		float aux2 = Dot(r_BP,info.Normal);
		aux2 *= aux2;
		float jDen = Dot(info.Normal,info.Normal)*(1.0f/our->m_Mass+1.0f/their->m_Mass) + aux1/our->m_RotationalInertia + aux2/their->m_RotationalInertia;

		float j = jNum/jDen;

		//our->m_Velocity = our->m_Velocity + (j/our->m_Mass)* normal;
		our->m_Velocity = (j/our->m_Mass)* info.Normal;

		//our->m_RotationVelocity = our->m_RotationVelocity + (j/our->m_RotationalInertia)*Dot(r_AP,normal);
		our->m_RotationVelocity = (j/our->m_RotationalInertia)*Dot(r_AP,info.Normal);
	}

	void RigidBody::OnCollisionMine(RigidBody* our, RigidBody* their, const CollisionInfo& info)
	{
		#ifdef _DEBUG
		if( info.CollisionSurface.Type == CollisionContact::Point )
		{
			//Draw the Colision Point
			RigidBody::TemporalDebugShape shapeColPoint(3.0f,sf::Shape::Circle(info.CollisionSurface.Point1.x, info.CollisionSurface.Point1.y, 5.0f, sf::Color::Red));
			our->m_DebugShapesForFrame.push_back( shapeColPoint );
			RigidBody::TemporalDebugShape shapeNormal(3.0f,sf::Shape::Line(info.CollisionSurface.Point1.x, info.CollisionSurface.Point1.y, info.CollisionSurface.Point1.x+info.Normal.x*25.0f, info.CollisionSurface.Point1.y+info.Normal.y*25.0f, 5.0f, sf::Color::Magenta));
			our->m_DebugShapesForFrame.push_back( shapeNormal );
		}else
		{
			//Draw the Colision Edge
			RigidBody::TemporalDebugShape shapeColLine(3.0f,sf::Shape::Line(info.CollisionSurface.Point1.x, info.CollisionSurface.Point1.y, info.CollisionSurface.Point2.x, info.CollisionSurface.Point2.y, 5.0f, sf::Color::Red));
			our->m_DebugShapesForFrame.push_back( shapeColLine );
			RigidBody::TemporalDebugShape shapeNormal(3.0f,sf::Shape::Line((info.CollisionSurface.Point1.x+info.CollisionSurface.Point2.x)/2.0f, (info.CollisionSurface.Point1.y+info.CollisionSurface.Point2.y)/2.0f, (info.CollisionSurface.Point1.x+info.CollisionSurface.Point2.x)/2.0f+info.Normal.x*25.0f, (info.CollisionSurface.Point1.y+info.CollisionSurface.Point2.y)/2.0f+info.Normal.y*25.0f, 5.0f, sf::Color::Magenta));
			our->m_DebugShapesForFrame.push_back( shapeNormal );
		}
		#endif
		//Idealized Reflexion on Normal of Linear Velocity and Angular Rotation
		//
		//				1) Linear Velocity: Reflected from N
		//					v = v - n * (n * v * 2)
		//
		//				1) Angular Velocity: By definition of angular velocity
		//					w = (r_ap x v)/||r_ap||^2

		//Gets only 1 representative collision point from the collision surface
		sf::Vector2<float> ColisionPoint;
		if( info.CollisionSurface.Type == CollisionContact::Point )
			ColisionPoint = info.CollisionSurface.Point1;
		else
			ColisionPoint = (info.CollisionSurface.Point1 + info.CollisionSurface.Point2)/2.0f;

		our->m_Velocity = our->m_Velocity - info.Normal * (Dot(info.Normal, our->m_Velocity)) * 2.0f;

		//float dummy;
		//Collider col;
		//while( col.TestPolygonVsPolygon(*dynamic_cast<Polygon*>(our),*dynamic_cast<Polygon*>(their),dummy) )
		//	our->m_Position += repel*0.01f;

		sf::Vector2<float> aux = ColisionPoint-our->GetPosition();
		sf::Vector3<float> rotor = Cross( sf::Vector3<float>(aux.x,aux.y,0.0f), sf::Vector3<float>(our->m_Velocity.x,our->m_Velocity.y,0.0f)) / Dot(aux,aux);
		
		our->m_RotationVelocity = rotor.z;
	}

	void RigidBody::_DebugDraw(sf::RenderWindow& Wnd)
	{
		//Draw Velocity
		_DrawArrow(Wnd, m_Position, m_Velocity, sf::Color(255,255,255,128), 30.0f);

		sf::Vector2<float> totForce;
		for(int i=0; i < m_Forces.size() ; ++i)
			totForce += m_Forces[i].Force;
		//Draw Aceleration
		_DrawArrow(Wnd, m_Position, totForce/GetMass(), sf::Color(0,0,255,128), 30.0f);

		//Draw Center of mass
		Wnd.Draw( sf::Shape::Circle(m_CenterOfMass.x+m_Position.x, m_CenterOfMass.y+m_Position.y, 5.0f, sf::Color::Green) );
		//for(int i=0; i < m_NormalsVector.size() ; ++i)
		//	_DrawArrow(Wnd, m_Normals[i].first, m_NormalsVector[i].second, sf::Color(255,255,255,128), 30.0f);

		std::list<TemporalDebugShape>::iterator it = m_DebugShapesForFrame.begin();
		while( it != m_DebugShapesForFrame.end() )
		{
			if( it->ElapsedTime > 0 )
			{
				it->ElapsedTime -= Wnd.GetFrameTime();
				Wnd.Draw( it->Figure );
				it++;
			}else
				it = m_DebugShapesForFrame.erase(it);
		}
	}

	void RigidBody::_DrawArrow(sf::RenderWindow& RW, sf::Vector2<float>& Vec1, sf::Vector2<float>& Vec2, sf::Color color, float scale)
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

	/////////////////////////////////////////////////////////////////////////////////
	///////////////////////        Broad Implementation         /////////////////////
	/////////////////////////////////////////////////////////////////////////////////

	Broad::Broad()
	{
		m_BoundingBox.m_MinX = 0.0f;
		m_BoundingBox.m_MinY = 0.0f;
		m_BoundingBox.m_MaxX = 0.0f;
		m_BoundingBox.m_MaxY = 0.0f;
	}


	void Broad::_CreateBoundingBox(const std::vector< sf::Vector2<float> > & Vertices)
	{
		for(int i=0; i < Vertices.size() ; ++i)
		{
			(Vertices[i].x < m_BoundingBox.m_MinX)? (m_BoundingBox.m_MinX = Vertices[i].x) : 0;

			(Vertices[i].y < m_BoundingBox.m_MinY)? (m_BoundingBox.m_MinY = Vertices[i].y) : 0;

			(Vertices[i].x > m_BoundingBox.m_MaxX)? (m_BoundingBox.m_MaxX = Vertices[i].x) : 0;

			(Vertices[i].y > m_BoundingBox.m_MaxY)? (m_BoundingBox.m_MaxY = Vertices[i].y) : 0;
		}
	}

	/////////////////////////////////////////////////////////////////////////////////
	///////////////////////       Polygon Implementation        /////////////////////
	/////////////////////////////////////////////////////////////////////////////////


	Polygon::Polygon(): m_pWorldLimit(NULL)
	{
		m_Vertices.reserve(4);
	}

	Polygon::~Polygon()
	{
		if( m_pWorldLimit )
			delete m_pWorldLimit;

		m_Vertices.clear();
	}

	void Polygon::_ReCalcRotationalInertia()
	{
		m_RotationalInertia = 0.0f;
		for(int i=0; i < m_Vertices.size() ; ++i)
		{
			float r = Norm( m_CenterOfMass - m_Vertices[i] );
			m_RotationalInertia += 1.0f * r * r;
		}
	}

	void Polygon::_RemakeCenterOfMass()
	{
		m_CenterOfMass.x = 0.0f;
		m_CenterOfMass.y = 0.0f;
		for(int i=0; i < m_Vertices.size() ; ++i)
			m_CenterOfMass += m_Vertices[i];
		m_CenterOfMass *= 1.0f/m_Vertices.size();
	}

	void Polygon::_RemakeCenter()
	{
		m_Center.x = 0.0f;
		m_Center.y = 0.0f;
		for(int i=0; i < m_Vertices.size() ; ++i)
			m_Center += m_Vertices[i];
		m_CenterOfMass *= 1.0f/m_Vertices.size();
	}

	void Polygon::_AddEdge()
	{
		if( m_Vertices.size() == 3 )
		{
			m_Edges.push_back(m_Vertices[1] - m_Vertices[0]);
			m_Edges.push_back(m_Vertices[2] - m_Vertices[1]);
			m_Edges.push_back(m_Vertices[0] - m_Vertices[2]);
		}else
		{
			m_Edges.back() = m_Vertices[m_Vertices.size()-1] - m_Vertices[m_Vertices.size()-2];
			m_Edges.push_back( m_Vertices[0] - m_Vertices[m_Vertices.size()-1] );
		}
	}

	void Polygon::_AddNormal()
	{
		if( m_Vertices.size() == 3 )
		{
			sf::Vector2<float>& Edge0 = m_Edges[0];
			sf::Vector3<float> aux(Edge0.x, Edge0.y, 0.0f);
			aux = Cross<float>(aux, AxisZ );
			m_Normals.push_back( sf::Vector2<float>(aux.x, aux.y) );
			m_Normals.back() /= Norm(m_Normals.back());

			sf::Vector2<float>& Edge1 = m_Edges[1];
			aux.x = Edge1.x;
			aux.y = Edge1.y;
			aux.z = 0.0f;
			aux = Cross<float>(aux, AxisZ );
			m_Normals.push_back( sf::Vector2<float>(aux.x, aux.y) );
			m_Normals.back() /= Norm(m_Normals.back());

			sf::Vector2<float>& Edge2 = m_Edges[2];
			aux.x = Edge2.x;
			aux.y = Edge2.y;
			aux.z = 0.0f;
			aux = Cross<float>(aux, AxisZ );
			m_Normals.push_back( sf::Vector2<float>(aux.x, aux.y) );
			m_Normals.back() /= Norm(m_Normals.back());
		}else
		{
			sf::Vector2<float>& curEdgeFirst = m_Edges[m_Edges.size()-2];
			sf::Vector3<float> aux(curEdgeFirst.x, curEdgeFirst.y, 0.0f);
			aux = Cross<float>(aux, AxisZ );
			m_Normals.back() = sf::Vector2<float>(aux.x, aux.y);
			m_Normals.back() /= Norm(m_Normals.back());


			sf::Vector2<float>& curEdgeSecond = m_Edges[m_Edges.size()-1];
			aux.x = curEdgeSecond.x;
			aux.y = curEdgeSecond.y;
			aux.z = 0.0f;
			aux = Cross<float>(aux, AxisZ );
			m_Normals.push_back( sf::Vector2<float>(aux.x, aux.y) );
			m_Normals.back() /= Norm(m_Normals.back());
		}
	}

	void Polygon::Draw(sf::RenderWindow& Wnd)
	{
		sf::Shape poly;
		//poly.SetColor(m_Color);
		for(int i=0; i < m_Vertices.size() ; ++i)
			poly.AddPoint( m_Transform.Transform(m_Vertices[i]) );
			//poly.AddPoint(m_Vertices[i]+m_Position);
		Wnd.Draw(poly);

		this->_DebugDraw(Wnd);
	}

	float Polygon::GetArea() const
	{
		float areaTot = 0.0f;
		for(int i=0; i < m_Edges.size() ; ++i)
		{
			sf::Vector3<float> normal = Cross( sf::Vector3<float>(m_Edges[i].x, m_Edges[i].y, 0.0f), sf::Vector3<float>(m_Vertices[(i+1)%m_Vertices.size()].x,m_Vertices[(i+1)%m_Vertices.size()].y,0.0f)-sf::Vector3<float>(m_CenterOfMass.x,m_CenterOfMass.y,0.0f) );
			areaTot += Norm(normal)/2.0f;
		}
		return areaTot;
	}

	int Polygon::GetVertexCount() const
	{
		return m_Vertices.size();
	}

	sf::Vector2<float> Polygon::GetVertex(int i) const
	{
		return m_Vertices[i];
	}

	void Polygon::AddVertex(sf::Vector2<float> newVertex)
	{
		m_Vertices.push_back(newVertex);

		_RemakeCenterOfMass();

		_RemakeCenter();

		if( m_Vertices.size() > 2 )//Information for collision
		{
			_AddEdge();

			_AddNormal();
		}

		_ReCalcMass();

		_ReCalcRotationalInertia();

		_CreateBoundingBox(m_Vertices);
	}

	void Polygon::SetWorldLimit(float minX, float minY, float maxX, float maxY)
	{
		m_pWorldLimit = new WorldLimit(minX,minY,maxX,maxY);
	}

	bool Polygon::IsOutSideWorldLimit()
	{
		return m_pWorldLimit->IsOutSide(this, m_Position);
	}

	/////////////////////////////////////////////////////////////////////////////////
	///////////////////////        Circle Implementation        /////////////////////
	/////////////////////////////////////////////////////////////////////////////////

	Circle::Circle(): m_Radius(1.0f)
	{
		_ReCalcMass();
		_ReCalcRotationalInertia();
	}

	Circle::~Circle()
	{}

	void Circle::Draw(sf::RenderWindow& Wnd)
	{
		sf::Shape myShape = sf::Shape::Circle(m_Center.x, m_Center.y, m_Radius, sf::Color::Red);
		Wnd.Draw(myShape);
	}

	float Circle::GetArea() const
	{
		return (3.1415f*m_Radius*m_Radius);
	}

	float Circle::GetRadius() const
	{
		return m_Radius;
	}

	void Circle::SetRadius(float newradius)
	{
		m_Radius = newradius;

		_ReCalcMass();
		_ReCalcRotationalInertia();
	}

	void Circle::SetWorldLimit(float minX, float minY, float maxX, float maxY)
	{
		m_pWorldLimit = new WorldLimit(minX,minY,maxX,maxY);
	}

	void Circle::_ReCalcRotationalInertia()
	{
		m_RotationalInertia = 1.0f;//Supposed balanced
	}

	/////////////////////////////////////////////////////////////////////////////////
	///////////////////////      WorldLimit Implementation      /////////////////////
	/////////////////////////////////////////////////////////////////////////////////

	WorldLimit::WorldLimit(float minX, float minY, float maxX, float maxY)
	{
		m_Limits.m_MinX = minX;
		m_Limits.m_MinY = minY;
		m_Limits.m_MaxX = maxX;
		m_Limits.m_MaxY = maxY;
	}

	bool WorldLimit::IsOutSide(Broad* obj, const sf::Vector2<float>& position) const
	{

		if( (obj->m_BoundingBox.m_MinX+position.x) < m_Limits.m_MinX) return true;

		if( (obj->m_BoundingBox.m_MinY+position.y) < m_Limits.m_MinY) return true;

		if( (obj->m_BoundingBox.m_MaxX+position.x) > m_Limits.m_MaxX) return true;

		if( (obj->m_BoundingBox.m_MaxY+position.y) > m_Limits.m_MaxY) return true;

		return false;
	}

}