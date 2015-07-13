#pragma once

#include "RigidBody.h"

namespace Obj2D
	{

	class QuadTree;

	class Collider
	{
	protected:

	public:

		enum ColisionType{ Vertex, Edge };
		
		struct CollisionInfo
		{
			CollisionInfo(): Type(Vertex), pPoint1(NULL), pPoint2(NULL)
			{}

			sf::Vector2<float>* pPoint1;
			sf::Vector2<float>* pPoint2;
			enum ColisionType{ Vertex, Edge} Type;
		};

		enum Side{Left, Right};

		struct CollisionItem
		{
			CollisionItem(Polygon* a, Polygon* b, CollisionInfo info): A(a), B(b), Info(info)
			{}

			Polygon* A;
			Polygon* B;
			CollisionInfo Info;
		};

		void AllVsAllCollide(std::vector<Polygon*> Polygons);

		void NearVsNearCollide(std::vector<Polygon*> Polygons, ::Obj2D::QuadTree &QT);

		bool TestPointVsAABB(const sf::Vector2<float>& A, const Broad& B);

		bool TestPointVsPolygon(const sf::Vector2<float>& A, const Polygon& B);

		//Test if 2 Axis Align Bounding Boxes collide
		//Implemented using SAT test
		//Parameter:
		//- A   first AABB
		//- B   second AABB
		//Return:
		//- true if collide, else false
		bool TestAABBVsAABB(const Broad& A, const Broad& B) const;

		bool TestAABBVsAABB(const BoundingBox& A, const BoundingBox& B) const;

		//Test if 2 Circles collide
		//Implemented using SAT test. Uses vector through Cicles centers.
		//Parameter:
		//- A   first Circle
		//- B   second Circle
		//Return:
		//- true if collide, else false
		bool TestCircleVsCirle(const Circle& A, const Circle& B, float& outPenetration) const;

		//Test if a Circles and a Polygon collide
		//Implemented using SAT test. Uses Polygon normals and Circle_Center -> PolygonVertexs vectors.
		//Parameter:
		//- A   the Circle
		//- B   the Polygon
		//Return:
		//- true if collide, else false
		bool TestCircleVsPolygon(const Circle& A, const Polygon& B, float& outPenetration) const;

		//Test if 2 Polygon collide
		//Implemented using SAT test
		//Parameter:
		//- A                    first Polygon
		//- B                    second Polygon
		//- outPenetration (OUT) the amount of penetration
		//Return:
		//- true if collide, else false
		bool TestPolygonVsPolygon(const Polygon& A, const Polygon& B, float& outPenetration) const;

		//Find if 2 Polygon collide and information of the colision
		//Implemented using SAT modified
		//Parameter:
		//- A                   first Polygon
		//- B                   second Polygon
		//- firstTime     (OUT) the first time of colision in range [0,inf]
		//- lastTime      (OUT) the last  time of colision in range [0,inf]
		//- ColisionPoint (OUT) the point of colision(if a line simplifies to point)
		//Return:
		//- true if collide, else false
		bool FindPolygonVsPolygon(Polygon& A, Polygon& B, float& firstTime, float& lastTime, CollisionContact &CollisionSurface, sf::Vector2<float> &ColisionNormalOfA, float TMax) const;
	};

}