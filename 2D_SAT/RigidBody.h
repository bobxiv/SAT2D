#pragma once

#include "SFML//Graphics.hpp"
#include "SFML//System.hpp"
#include "SFML//Window.hpp"
#include "SFML//Config.hpp"

#include <vector>
#include <list>
//#include <string>

namespace Obj2D
{

	struct ForceDesc
	{
		ForceDesc(): Name(""), Force(0.0f,0.0f), AppliedPoint(0.0f,0.0f)
		{}

		ForceDesc(std::string name, sf::Vector2<float>& force, sf::Vector2<float>& appliedPoint = sf::Vector2<float>(0.0f,0.0f)): Name(name), Force(force), AppliedPoint(appliedPoint)
		{}

		std::string Name;
		sf::Vector2<float>	Force;
		sf::Vector2<float>	AppliedPoint;
	};

	struct CollisionContact
	{
		CollisionContact(): Type(Point), Point1(0.0f,0.0f), Point2(0.0f,0.0f)
		{}

		sf::Vector2<float> Point1;
		sf::Vector2<float> Point2;
		enum { Point, Edge} Type;
	};

	class Collider;

	class WorldLimit;

	struct Node;

	struct BoundingBox
	{
		BoundingBox(): m_MinX(-0.5f), m_MinY(0.5f), m_MaxX(0.5f), m_MaxY(-0.5f)
		{}
		BoundingBox(float minX, float minY, float maxX, float maxY): m_MinX(minX), m_MinY(minY), m_MaxX(maxX), m_MaxY(maxY)
		{}

		float m_MinX;
		float m_MinY;
		float m_MaxX;
		float m_MaxY;
	} ;

	class SpatialNode
	{
	protected:
		sf::Vector2<float>	m_Position;

	public:

		SpatialNode();
		virtual ~SpatialNode();

		BoundingBox			m_BoundingBoxSpatial;//Talvez no es lo mejor esto aca!!!!!!!!

		virtual void		Draw(sf::RenderWindow& Wnd) = 0;

		sf::Vector2<float>	GetPosition() const; 
		void				SetPosition(const sf::Vector2<float> &newVertex);
		void				Move(const sf::Vector2<float> &movement);
	};

	class RigidBody: public SpatialNode
	{
	protected:
		float				m_MassDensity;
		sf::Vector2<float>	m_CenterOfMass;
		sf::Vector2<float>	m_Center;
		float				m_Rotation;
		sf::Vector2<float>	m_Velocity;
		float				m_RotationVelocity;

		float				m_Mass;
		float				m_RotationalInertia;

		float				m_Restitution;

		sf::Matrix3			m_Transform;

		void				_ReCalcMass();
		virtual void		_ReCalcRotationalInertia()       = 0;

		void				_DebugDraw(sf::RenderWindow& Wnd);
		void				_DrawArrow(sf::RenderWindow& RW, sf::Vector2<float>& Vec1, sf::Vector2<float>& Vec2, sf::Color color, float scale);
	public:

		struct TemporalDebugShape
		{
			TemporalDebugShape(float elapsedTime, sf::Shape figure): ElapsedTime(elapsedTime), Figure(figure)
			{}

			sf::Shape  Figure;
			float      ElapsedTime;
		};

		std::list< TemporalDebugShape > m_DebugShapesForFrame;

		friend class Collider;

		struct CollisionInfo
		{
			CollisionInfo(): Penetration(0.0f), firstTime(-1.0f), lastTime(-1.0f), Normal(0.0f,0.0f)
			{}

			CollisionInfo(float penetration): Penetration(penetration), firstTime(-1.0f), lastTime(-1.0f),Normal(0.0f,0.0f)
			{}

			CollisionInfo(float penetration, float _firstTime, float _lastTime, CollisionContact collisionSurface, sf::Vector2<float> normal): Penetration(penetration), firstTime(_firstTime), lastTime(_lastTime), CollisionSurface(collisionSurface), Normal(normal)
			{}

			float Penetration;
			float firstTime;
			float lastTime;
			CollisionContact CollisionSurface;
			sf::Vector2<float> Normal;
		};

		RigidBody();

		std::vector<ForceDesc>	m_Forces;

		virtual void		Update(float elapsedTime);
		float				GetMassDensity() const;
		float				GetMass() const;
		float				GetRotationalInertia() const;
		virtual float		GetArea() const                   = 0;
		float				GetRotation() const;
		sf::Vector2<float>	GetVelocity() const;
		float				GetRotationVelocity() const;
		void				SetVelocity(const sf::Vector2<float>& newVelocity);
		void				SetRestitution(float newRestitution);

		void				(*OnCollision)(RigidBody* our, RigidBody* their, const CollisionInfo& info);


		friend void			OnCollisionImpulseNoRotation(RigidBody* our, RigidBody* their, const CollisionInfo& info);
		friend void			OnCollisionImpulseRotation(RigidBody* our, RigidBody* their, const CollisionInfo& info);
		friend void			OnCollisionMine(RigidBody* our, RigidBody* their, const CollisionInfo& info);

		static void			OnCollisionImpulseNoRotation(RigidBody* our, RigidBody* their, const CollisionInfo& info);
		static void			OnCollisionImpulseRotation(RigidBody* our, RigidBody* their, const CollisionInfo& info);
		static void			OnCollisionMine(RigidBody* our, RigidBody* their, const CollisionInfo& info);

	};

	//The general Broad form of objects
	class Broad
	{
	public:
		Broad();

		friend class WorldLimit;

		friend class Collider;

		friend class Node;

	//protected:
	public:

		struct {

			float m_MinX;
			float m_MinY;
			float m_MaxX;
			float m_MaxY;

		} m_BoundingBox;

	public:
		void _CreateBoundingBox(const std::vector< sf::Vector2<float> > & Vertices);

	};

	class Polygon: public RigidBody, public Broad
	{
	protected:

		std::vector<sf::Vector2<float>>	m_Vertices;

		sf::Vector2<float>				m_Center;

		std::vector<sf::Vector2<float>>	m_Edges;

		std::vector<sf::Vector2<float>>	m_Normals;

		WorldLimit*						m_pWorldLimit;

		void							_ReCalcRotationalInertia();

		void							_RemakeCenterOfMass();

		void							_RemakeCenter();

		void							_AddEdge();

		void							_AddNormal();

	public:

		friend class Collider;

		Polygon();
		virtual ~Polygon();

		void				Draw(sf::RenderWindow& Wnd);
		float				GetArea()const;
		int					GetVertexCount() const;
		sf::Vector2<float>	GetVertex(int i) const;
		void				AddVertex(sf::Vector2<float> newVertex);
		void				SetWorldLimit(float minX, float minY, float maxX, float maxY);
		bool				IsOutSideWorldLimit();

	};

	class Circle: public RigidBody, public Broad
	{
	protected:

		float				m_Radius;

		WorldLimit*			m_pWorldLimit;

		void				_ReCalcRotationalInertia();

	public:

		friend class Collider;

		Circle();
		~Circle();

		void				Draw(sf::RenderWindow& Wnd);
		float				GetArea() const;
		float				GetRadius()const;
		void				SetRadius(float newradius);
		void				SetWorldLimit(float minX, float minY, float maxX, float maxY);
	};

	class WorldLimit
	{
	private:
		struct {

			float m_MinX;
			float m_MinY;
			float m_MaxX;
			float m_MaxY;

		} m_Limits;
	public:

		friend class Collider;

		WorldLimit(float minX, float minY, float maxX, float maxY);

		bool IsOutSide(Broad* obj, const sf::Vector2<float>& position) const;
		
	};

}