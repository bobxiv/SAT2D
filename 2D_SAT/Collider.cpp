#include "Collider.h"

#include "Math.h"

#include "limits"

#include "QuadTree.h"

namespace Obj2D
{

	/////////////////////////////////////////////////////////////////////////////////
	///////////////////////       Collider Implementation       /////////////////////
	/////////////////////////////////////////////////////////////////////////////////

	void Collider::AllVsAllCollide(std::vector<Polygon*> Polygons)
	{
		float Penetration;
		for(int i=0; i < Polygons.size() ; ++i)
			for(int j=0; j < Polygons.size() ; ++j)
				if( i != j )
					if( i < j )
						if( TestPolygonVsPolygon(*Polygons[i], *Polygons[j], Penetration) )
						{
							//sf::Vector2<float> zero(0.0f,0.0f);
							//Polygons[i]->SetVelocity(zero);
							Polygons[i]->OnCollision(Polygons[i], Polygons[j], Penetration);

							//Polygons[j]->SetVelocity(zero);
							Polygons[j]->OnCollision(Polygons[j], Polygons[i], Penetration);
						}
		for(int i=0; i < Polygons.size() ; ++i)
			if( Polygons[i]->IsOutSideWorldLimit() )
			{/*
				sf::Vector2<float> center(400.0f,300.0f);
				sf::Vector2<float> pos = Polygons[i]->GetPosition();
				::Obj2D::ForceDesc wallRepel;
				wallRepel.Force = center-pos;
				sf::Vector2<float> zero(0.0f,0.0f);
				Polygons[i]->SetVelocity(zero);
				Polygons[i]->m_Forces.clear();
				Polygons[i]->m_Forces.push_back(wallRepel);
*/
				if( (Polygons[i]->m_BoundingBox.m_MinX+Polygons[i]->m_Position.x) < Polygons[i]->m_pWorldLimit->m_Limits.m_MinX)//left
				{
					sf::Vector2<float> N(1.0f,0.0f);

					Polygons[i]->m_Velocity = Polygons[i]->m_Velocity - N * (Dot(N, Polygons[i]->m_Velocity)) * 2.0f;
					//Polygons[i]->m_Velocity *= -1.0f;
				}


				if( (Polygons[i]->m_BoundingBox.m_MinY+Polygons[i]->m_Position.y) < Polygons[i]->m_pWorldLimit->m_Limits.m_MinY)//top
				{
					sf::Vector2<float> N(0.0f,1.0f);

					Polygons[i]->m_Velocity = Polygons[i]->m_Velocity - N * (Dot(N, Polygons[i]->m_Velocity)) * 2.0f;
					//Polygons[i]->m_Velocity *= -1.0f;
				}

				if( (Polygons[i]->m_BoundingBox.m_MaxX+Polygons[i]->m_Position.x) > Polygons[i]->m_pWorldLimit->m_Limits.m_MaxX)//right
				{
					sf::Vector2<float> N(-1.0f,0.0f);

					Polygons[i]->m_Velocity = Polygons[i]->m_Velocity - N * (Dot(N, Polygons[i]->m_Velocity)) * 2.0f;
					//Polygons[i]->m_Velocity *= -1.0f;
				}

				if( (Polygons[i]->m_BoundingBox.m_MaxY+Polygons[i]->m_Position.y) > Polygons[i]->m_pWorldLimit->m_Limits.m_MaxY)//bottom
				{
					sf::Vector2<float> N(0.0f,-1.0f);

					Polygons[i]->m_Velocity = Polygons[i]->m_Velocity - N * (Dot(N, Polygons[i]->m_Velocity)) * 2.0f;
					//Polygons[i]->m_Velocity *= -1.0f;
				}

			}
	}

	void Collider::NearVsNearCollide(std::vector<Polygon*> Polygons, QuadTree& QT)
	{
		float Penetration;
		std::vector<SpatialNode*> NearArray;

		for(int i=0; i < Polygons.size() ; ++i)//ACA ESTA TERRIBLEMENTE PARCHEADO, FUE UN PARCHE MAL!!!
		{
			Polygons[i]->m_BoundingBoxSpatial.m_MinX = Polygons[i]->m_BoundingBox.m_MinX+Polygons[i]->GetPosition().x;
			Polygons[i]->m_BoundingBoxSpatial.m_MinY = Polygons[i]->m_BoundingBox.m_MinY+Polygons[i]->GetPosition().y;
			Polygons[i]->m_BoundingBoxSpatial.m_MaxX = Polygons[i]->m_BoundingBox.m_MaxX+Polygons[i]->GetPosition().x;
			Polygons[i]->m_BoundingBoxSpatial.m_MaxY = Polygons[i]->m_BoundingBox.m_MaxY+Polygons[i]->GetPosition().y;
		}

		//std::vector< CollisionItem > CollisionList;
		for(int i=0; i < Polygons.size() ; ++i)
		{
			NearArray = QT.QueryNearElements(Polygons[i]);

			for(int j=0; j < NearArray.size() ; ++j)
			{
				if( Polygons[i] != dynamic_cast<Polygon*>(NearArray[j]) )
				{
					float firstTime = -1.0f;
					float lastTime  = -1.0f;
					CollisionContact CollisionSurface;
					sf::Vector2<float> ColisionNormalOfA;
					if( FindPolygonVsPolygon(*Polygons[i], *dynamic_cast<Polygon*>(NearArray[j]), firstTime, lastTime, CollisionSurface, ColisionNormalOfA, 50.0f) )
					{
						//CollisionList.push_back( CollisionItem(*Polygons[i], *dynamic_cast<Polygon*>(NearArray[j]), 
						Polygons[i]->OnCollision(Polygons[i], dynamic_cast<RigidBody*>(NearArray[j]), Obj2D::RigidBody::CollisionInfo(0.0f,firstTime,lastTime,CollisionSurface,ColisionNormalOfA));
						Polygons[i]->OnCollision(dynamic_cast<RigidBody*>(NearArray[j]), Polygons[i], Obj2D::RigidBody::CollisionInfo(0.0f,firstTime,lastTime,CollisionSurface,-ColisionNormalOfA));
					}
				}
					/*if( TestPolygonVsPolygon(*Polygons[i], *dynamic_cast<Polygon*>(NearArray[j]), Penetration) )
						Polygons[i]->OnCollision(Polygons[i], dynamic_cast<RigidBody*>(NearArray[j]), Penetration);*/
			}
		}
		
		for(int i=0; i < Polygons.size() ; ++i)
			if( Polygons[i]->IsOutSideWorldLimit() )
			{
				//sf::Vector2<float> center(400.0f,300.0f);
				//sf::Vector2<float> pos = Polygons[i]->GetPosition();
				//::Obj2D::ForceDesc wallRepel;
				//wallRepel.Force = center-pos;
				//sf::Vector2<float> zero(0.0f,0.0f);
				//Polygons[i]->SetVelocity(zero);
				//Polygons[i]->m_Forces.clear();
				//Polygons[i]->m_Forces.push_back(wallRepel);

				if( (Polygons[i]->m_BoundingBox.m_MinX+Polygons[i]->m_Position.x) < Polygons[i]->m_pWorldLimit->m_Limits.m_MinX)//left
				{
					sf::Vector2<float> N(1.0f,0.0f);

					Polygons[i]->m_Velocity = Polygons[i]->m_Velocity - N * (Dot(N, Polygons[i]->m_Velocity)) * 2.0f;
					//Polygons[i]->m_Velocity *= -1.0f;
				}


				if( (Polygons[i]->m_BoundingBox.m_MinY+Polygons[i]->m_Position.y) < Polygons[i]->m_pWorldLimit->m_Limits.m_MinY)//top
				{
					sf::Vector2<float> N(0.0f,1.0f);

					Polygons[i]->m_Velocity = Polygons[i]->m_Velocity - N * (Dot(N, Polygons[i]->m_Velocity)) * 2.0f;
					//Polygons[i]->m_Velocity *= -1.0f;
				}

				if( (Polygons[i]->m_BoundingBox.m_MaxX+Polygons[i]->m_Position.x) > Polygons[i]->m_pWorldLimit->m_Limits.m_MaxX)//right
				{
					sf::Vector2<float> N(-1.0f,0.0f);

					Polygons[i]->m_Velocity = Polygons[i]->m_Velocity - N * (Dot(N, Polygons[i]->m_Velocity)) * 2.0f;
					//Polygons[i]->m_Velocity *= -1.0f;
				}

				if( (Polygons[i]->m_BoundingBox.m_MaxY+Polygons[i]->m_Position.y) > Polygons[i]->m_pWorldLimit->m_Limits.m_MaxY)//bottom
				{
					sf::Vector2<float> N(0.0f,-1.0f);

					//Arbitrarily fix interpenetration
					Polygons[i]->Move( N * abs(Polygons[i]->m_pWorldLimit->m_Limits.m_MaxY-(Polygons[i]->m_BoundingBox.m_MaxY+Polygons[i]->m_Position.y)));
					Polygons[i]->m_Velocity = Polygons[i]->m_Velocity - N * (Dot(N, Polygons[i]->m_Velocity)) * 2.0f;
					//Polygons[i]->m_Velocity *= -1.0f;
				}

			}
	}

	bool Collider::TestPointVsAABB(const sf::Vector2<float>& A, const Broad& B)
	{
		//sf::Vector2<float> localA = A-B.m_Position;//Point in B local coordinates
		//if( !(A.x > obj->m_BoundingBox.m_MinX && A.x < obj->m_BoundingBox.m_MaxX) ) return false;

		//if( !(A.y > obj->m_BoundingBox.m_MinY && A.y < obj->m_BoundingBox.m_MaxY) ) return false;

		return true;//Definitely inside
	}

	bool Collider::TestPointVsPolygon(const sf::Vector2<float>& A, const Polygon& B)
	{
		if( B.m_Vertices.size() < 3 )//if 0, 1 or 2 We don't consider that we can have a point inside
			return false; //nothing, another point, a line

		sf::Vector2<float> localA = A-B.m_Position;//Point in B local coordinates
		for(int i=0; i < B.m_Edges.size() ; ++i)
		{
			sf::Vector3<float> res = Cross( sf::Vector3<float>(B.m_Edges[i].x,B.m_Edges[i].y,0.0f), sf::Vector3<float>(localA.x-B.m_Vertices[i].x,localA.y-B.m_Vertices[i].y,0.0f));
			
			if( res.z < 0 )
				return false;//Definitely not inside
		}

		return true;//Inside
	}

	bool Collider::TestAABBVsAABB(const Broad& A, const Broad& B) const
	{
		//X axis not colliding?
		if( A.m_BoundingBox.m_MaxX < B.m_BoundingBox.m_MinX || B.m_BoundingBox.m_MaxX < A.m_BoundingBox.m_MinX )
			return false;

		//Y axis not colliding?
		if( A.m_BoundingBox.m_MaxY < B.m_BoundingBox.m_MinY || B.m_BoundingBox.m_MaxY < A.m_BoundingBox.m_MinY )
			return false;

		return true;//then we are colliding
	}

	bool Collider::TestAABBVsAABB(const BoundingBox& A, const BoundingBox& B) const
	{
		//X axis not colliding?
		if( A.m_MaxX < B.m_MinX || B.m_MaxX < A.m_MinX )
			return false;

		//Y axis not colliding?
		if( A.m_MaxY < B.m_MinY || B.m_MaxY < A.m_MinY )
			return false;

		return true;//then we are colliding
	}

	bool Collider::TestCircleVsCirle(const Circle& A, const Circle& B, float& outPenetration) const
	{

		sf::Vector2<float> diff;
		diff = A.m_Center - B.m_Center;
		float lengthSquare = Dot<float>(diff,diff);
		float radiusSquare = A.m_Radius + B.m_Radius;
		radiusSquare *= radiusSquare;

		if( radiusSquare <= lengthSquare )
		{
			outPenetration = sqrt(lengthSquare-radiusSquare);
			return true;
		}

		return false;
	}

	bool Collider::TestCircleVsPolygon(const Circle& A, const Polygon& B, float& outPenetration) const
	{
		if( B.m_Vertices.size() < 2 )
			return false;

		outPenetration = std::numeric_limits<float>::max();

		sf::Vector2<float> centerDiffAB = A.m_Center - B.m_Center;
		float projC_Dif;

			//Test axis perpendicular to the Polygon Edges F: there are F_B tests

		float r = 0.0f;
		//A axis calcs
		for(int i=0; i < B.m_Normals.size() ; ++i)
		{
			for(int j=0; j < B.m_Edges.size() ; ++j)		
				r += abs(Dot(B.m_Edges[j], B.m_Normals[i]));

			r /= 4.0f;

			r += A.m_Radius;

			projC_Dif = Dot( centerDiffAB, B.m_Normals[i]);

			if( abs(projC_Dif) > r )
				return false;//Not Colliding
			if( (r-abs(projC_Dif)) < outPenetration ) outPenetration = r-abs(projC_Dif);
		}

		sf::Vector2<float> circlePolygonVertexAxis;
			//Test axis that pass through the circe center and each vertex of the polygon
		for(int i=0; i < B.m_Vertices.size() ; ++i)
		{
			circlePolygonVertexAxis = A.m_Center - B.m_Vertices[i];//The separation axis

			for(int j=0; j < B.m_Edges.size() ; ++j)		
				r += abs(Dot(B.m_Edges[j], circlePolygonVertexAxis));

			r /= 4.0f;

			r += A.m_Radius;

			projC_Dif = Dot( centerDiffAB, circlePolygonVertexAxis);

			if( abs(projC_Dif) > r )
				return false;//Not Colliding
			if( (r-abs(projC_Dif)) < outPenetration ) outPenetration = r-abs(projC_Dif);
		}

		return true;//they collide in every posible separation axis then Colliding
	}

	bool Collider::TestPolygonVsPolygon(const Polygon& A, const Polygon& B, float& outPenetration) const
	{
		if( A.m_Vertices.size() < 2 || B.m_Vertices.size() < 2 )
			return false;
		
		outPenetration = std::numeric_limits<float>::max();

		sf::Vector2<float> centerDiffAB = (A.m_Center+A.m_Position) - (B.m_Center+B.m_Position);
		float projC_Dif;

		sf::Matrix3 TS_A, VertexTS_A;//,TS_Anormal;
		TS_A.SetFromTransformations(sf::Vector2<float>(0.0f,0.0f), sf::Vector2<float>(0.0f,0.0f), A.GetRotation(), sf::Vector2<float>(1.0f,1.0f));
		VertexTS_A.SetFromTransformations(sf::Vector2<float>(0.0f,0.0f), A.GetPosition(), A.GetRotation(), sf::Vector2<float>(1.0f,1.0f));
		//TS_Anormal.SetFromTransformations(sf::Vector2<float>(0.0f,0.0f), sf::Vector2<float>(0.0f,0.0f), -A.GetRotation(), sf::Vector2<float>(1.0f,1.0f));
		//TS_A.SetFromTransformations(sf::Vector2<float>(0.0f,0.0f), sf::Vector2<float>(0.0f,0.0f), 0.0f, sf::Vector2<float>(1.0f,1.0f));
		std::vector< sf::Vector2<float> > A_Vertices(A.m_Vertices.size());
		std::vector< sf::Vector2<float> > A_Normals(A.m_Vertices.size());
		std::vector< sf::Vector2<float> > A_Edges(A.m_Vertices.size());
		for(int i=0; i < A.m_Vertices.size() ; ++i)
		{
			A_Vertices[i] = VertexTS_A.Transform(A.m_Vertices[i]*sf::Vector2<float>(1.0f,1.0f));
			A_Normals[i]  = TS_A.Transform(A.m_Normals[i]*sf::Vector2<float>(1.0f,1.0f) );
			A_Edges[i]    = VertexTS_A.Transform(A.m_Edges[i]*sf::Vector2<float>(1.0f,1.0f)   );
		}
		sf::Matrix3 TS_B, VertexTS_B;//, TS_Bnormal;
		TS_B.SetFromTransformations(sf::Vector2<float>(0.0f,0.0f), sf::Vector2<float>(0.0f,0.0f), B.GetRotation(), sf::Vector2<float>(1.0f,1.0f));
		VertexTS_B.SetFromTransformations(sf::Vector2<float>(0.0f,0.0f), B.GetPosition(), B.GetRotation(), sf::Vector2<float>(1.0f,1.0f));
		//TS_Bnormal.SetFromTransformations(sf::Vector2<float>(0.0f,0.0f), sf::Vector2<float>(0.0f,0.0f), -B.GetRotation(), sf::Vector2<float>(1.0f,1.0f));
		//TS_B.SetFromTransformations(sf::Vector2<float>(0.0f,0.0f), sf::Vector2<float>(0.0f,0.0f), 0.0f, sf::Vector2<float>(1.0f,1.0f));
		std::vector< sf::Vector2<float> > B_Vertices(B.m_Vertices.size());
		std::vector< sf::Vector2<float> > B_Normals(B.m_Vertices.size());
		std::vector< sf::Vector2<float> > B_Edges(B.m_Vertices.size());
		for(int i=0; i < B.m_Vertices.size() ; ++i)
		{
			B_Vertices[i] = VertexTS_B.Transform(B.m_Vertices[i]*sf::Vector2<float>(1.0f,1.0f));
			B_Normals[i]  = TS_B.Transform(B.m_Normals[i]*sf::Vector2<float>(1.0f,1.0f) );
			B_Edges[i]    = VertexTS_B.Transform(B.m_Edges[i]*sf::Vector2<float>(1.0f,1.0f)   );
		}

			//Test Faces F: there are 2*(F_1+F_2) tests

		float r = 0.0f;
		//A axis calcs
		for(int i=0; i < A.m_Normals.size() ; ++i)
		{
			for(int j=0; j < B.m_Edges.size() ; ++j)
				r += abs(Dot(B_Edges[j], A_Normals[i]));

			for(int j=0; j < A.m_Edges.size() ; ++j)
				r += abs(Dot(A_Edges[j], A_Normals[i]));

			r /= 4.0f;

			projC_Dif = Dot( centerDiffAB, A_Normals[i]);

			if( abs(projC_Dif) > r )
				return false;//Not Colliding
			if( (r-abs(projC_Dif)) < outPenetration ) outPenetration = r-abs(projC_Dif);
			r = 0.0f;
		}

		r = 0.0f;
		//B axis calcs
		for(int i=0; i < B.m_Normals.size() ; ++i)
		{
			for(int j=0; j < B_Edges.size() ; ++j)
				r += abs(Dot(B_Edges[j], B_Normals[i]));

			for(int j=0; j < A.m_Edges.size() ; ++j)
				r += abs(Dot(A_Edges[j], B_Normals[i]));

			r /= 4.0f;

			projC_Dif = Dot( centerDiffAB, B_Normals[i]);

			if( abs(projC_Dif) > r )
				return false;//Not Colliding
			if( (r-abs(projC_Dif)) < outPenetration ) outPenetration = r-abs(projC_Dif);
			r = 0.0f;
		}

		return true;//they collide in every posible separation axis then Colliding
	}

	bool Collider::FindPolygonVsPolygon(Polygon& A, Polygon& B, float& firstTime, float& lastTime, CollisionContact &CollisionSurface, sf::Vector2<float> &ColisionNormalOfA, float TMax) const
	{
		if( A.m_Vertices.size() < 2 || B.m_Vertices.size() < 2 )
			return false;

		//firstTime = 0.0f;
		firstTime = -std::numeric_limits<float>::max();
		lastTime  = std::numeric_limits<float>::max();

		sf::Matrix3 TS_A;//,TS_Anormal;
		sf::Matrix3 VertexTS_A;
		TS_A.SetFromTransformations(sf::Vector2<float>(0.0f,0.0f), sf::Vector2<float>(0.0f,0.0f), A.GetRotation(), sf::Vector2<float>(1.0f,1.0f));
		VertexTS_A.SetFromTransformations(sf::Vector2<float>(0.0f,0.0f), A.GetPosition(), A.GetRotation(), sf::Vector2<float>(1.0f,1.0f));
		std::vector< sf::Vector2<float> > A_Vertices(A.m_Vertices.size());
		std::vector< sf::Vector2<float> > A_Normals(A.m_Vertices.size());
		std::vector< sf::Vector2<float> > A_Edges(A.m_Vertices.size());
		for(int i=0; i < A.m_Vertices.size() ; ++i)
		{
			A_Vertices[i] = VertexTS_A.Transform(A.m_Vertices[i]*sf::Vector2<float>(1.0f,1.0f));
			A_Normals[i]  = TS_A.Transform(A.m_Normals[i]*sf::Vector2<float>(1.0f,1.0f) );
			A_Edges[i]    = TS_A.Transform(A.m_Edges[i]*sf::Vector2<float>(1.0f,1.0f)   );
		}
		sf::Matrix3 TS_B;//, TS_Bnormal;
		sf::Matrix3 VertexTS_B;//, TS_Bnormal;
		TS_B.SetFromTransformations(sf::Vector2<float>(0.0f,0.0f), sf::Vector2<float>(0.0f,0.0f), B.GetRotation(), sf::Vector2<float>(1.0f,1.0f));
		VertexTS_B.SetFromTransformations(sf::Vector2<float>(0.0f,0.0f), B.GetPosition(), B.GetRotation(), sf::Vector2<float>(1.0f,1.0f));
		std::vector< sf::Vector2<float> > B_Vertices(B.m_Vertices.size());
		std::vector< sf::Vector2<float> > B_Normals(B.m_Vertices.size());
		std::vector< sf::Vector2<float> > B_Edges(B.m_Vertices.size());
		for(int i=0; i < B.m_Vertices.size() ; ++i)
		{
			B_Vertices[i] = VertexTS_B.Transform(B.m_Vertices[i]*sf::Vector2<float>(1.0f,1.0f));
			B_Normals[i]  = TS_B.Transform(B.m_Normals[i]*sf::Vector2<float>(1.0f,1.0f) );
			B_Edges[i]    = TS_B.Transform(B.m_Edges[i]*sf::Vector2<float>(1.0f,1.0f)   );
		}

		sf::Vector2<float> V = B.GetVelocity() - A.GetVelocity();//Process as if A is stationary and B is moving
		float minA, minB, maxA, maxB, aux, T, speed;
		const sf::Vector2<float>* pFirstColisionAxis = NULL;//The first axis in witch the objects effectively collides
															//Also it will be the collision normal
		Side SideOfB;										//Which side B is of object A with respect to the collision normal
		bool isNormalOfA;									//Tells if pFirstColisionAxis is an axis from A
		//float epi = 0.0000000000000000000001f;//std::numeric_limits<float>::min()*10.0f;
		float epi = std::numeric_limits<float>::min()*1000000000000000000000000000.0f;

			//SAT Test Faces F: there are F_A+F_B tests

			//Implemented projecting the Vertices and calculating
			//the maximum and minimum values of them

		#pragma region A axis calcs
		for(int i=0; i < A_Normals.size() ; ++i)
		{
			speed = Dot(V,A_Normals[i]);
			minB = minA = std::numeric_limits<float>::max();
			maxB = maxA = -std::numeric_limits<float>::max();

			for(int j=0; j < B_Vertices.size() ; ++j)
			{
				aux = Dot(B_Vertices[j], A_Normals[i]);
				(aux < minB)? (minB = aux) : 0;
				(aux > maxB)? (maxB = aux) : 0;
			}

			for(int j=0; j < A_Vertices.size() ; ++j)
			{
				aux = Dot(A_Vertices[j], A_Normals[i]);
				(aux < minA)? (minA = aux) : 0;
				(aux > maxA)? (maxA = aux) : 0;
			}

			if( maxB < minA )//B is left of A
			{
				if( speed <= 0.0f )//they go away
					return false;
				//The Time of first collision in this axis
				T = (minA - maxB)/speed;
				if(T > firstTime)
				{
					firstTime = T;
					pFirstColisionAxis = &A_Normals[i];
					SideOfB = Side::Left;
					isNormalOfA = true;
				}
				if( firstTime > TMax )//Collision to far in time
					return false;
				//The Time of last collision in this axis
				T = (maxA - minB)/speed;
				if(T < lastTime)
					lastTime = T;
				if( firstTime > lastTime )//Not collide
					return false;
			}else
			{
				if( maxA < minB )//B is right of A
				{
					if( speed >= 0.0f )//they go away
						return false;
					//The Time of first collision in this axis
					T = (maxA - minB)/speed;
					if(T > firstTime)
					{
						firstTime = T;
						pFirstColisionAxis = &A_Normals[i];
						SideOfB = Side::Right;
						isNormalOfA = true;
					}
					if( firstTime > TMax )//Collision to far in time
						return false;
					//The Time of last collision in this axis
					T = (minA - maxB)/speed;
					if(T < lastTime)
						lastTime = T;
					if( firstTime > lastTime )//Not collide
						return false;
				}else//A and B overlap
				{
					if(speed > 0.0f)
					{
						//The Time of collision USE TO FIX INTERPENETRATION
						//will be negative
						T = (minA - maxB)/speed;
						if(T > firstTime)
						{
							firstTime = T;
							pFirstColisionAxis = &A_Normals[i];
							SideOfB = Side::Left;;//Will be post-fix
							isNormalOfA = true;
						}
						//The Time of last collision in this axis
						T = (maxA - minB)/speed;
						if(T < lastTime)
							lastTime = T;
						if(firstTime > lastTime)//Not collide
							return false;
					}else if(speed < 0.0f)
					{
						//The Time of collision USE TO FIX INTERPENETRATION
						//will be negative
						T = (maxA - minB)/speed;
						if(T > firstTime)
						{
							firstTime = T;
							pFirstColisionAxis = &A_Normals[i];
							SideOfB = Side::Right;//Will be post-fix
							isNormalOfA = true;
						}
						//The Time of last collision in this axis
						T = (minA - maxB)/speed;
						if(T < lastTime)
							lastTime = T;
						if(firstTime > lastTime)//Not collide
							return false;
					}
				}
			}
		}
		#pragma endregion

		#pragma region B axis calcs
		for(int i=0; i < B_Normals.size() ; ++i)
		{
			speed = Dot(V,B_Normals[i]);
			minB = minA = std::numeric_limits<float>::max();
			maxB = maxA = -std::numeric_limits<float>::max();

			for(int j=0; j < B_Vertices.size() ; ++j)
			{
				aux = Dot(B_Vertices[j], B_Normals[i]);
				(aux < minB)? (minB = aux) : 0;
				(aux > maxB)? (maxB = aux) : 0;
			}

			for(int j=0; j < A_Vertices.size() ; ++j)
			{
				aux = Dot(A_Vertices[j], B_Normals[i]);
				(aux < minA)? (minA = aux) : 0;
				(aux > maxA)? (maxA = aux) : 0;
			}

			if( maxB < minA )//B is left of A
			{
				if( speed <= 0.0f )//they go away
					return false;
				//The Time of first collision in this axis
				T = (minA - maxB)/speed;
				if(T > firstTime)
				{
					firstTime = T;
					pFirstColisionAxis = &B_Normals[i];
					SideOfB = Side::Left;
					isNormalOfA = false;
				}
				if( firstTime > TMax )//Collision to far in time
					return false;
				//The Time of last collision in this axis
				T = (maxA - minB)/speed;
				if(T < lastTime)
					lastTime = T;
				if( firstTime > lastTime )//Not collide
					return false;
			}else
			{
				if( maxA < minB )//B is right of A
				{
					if( speed >= 0.0f )//they go away
						return false;
					//The Time of first collision in this axis
					T = (maxA - minB)/speed;
					if(T > firstTime)
					{
						firstTime = T;
						pFirstColisionAxis = &B_Normals[i];
						SideOfB = Side::Right;
						isNormalOfA = false;
					}
					if( firstTime > TMax )//Collision to far in time
						return false;
					//The Time of last collision in this axis
					T = (minA - maxB)/speed;
					if(T < lastTime)
						lastTime = T;
					if( firstTime > lastTime )//Not collide
						return false;
				}else//A and B overlap
				{
					if(speed > 0.0f)
					{
						//The Time of collision USE TO FIX INTERPENETRATION
						//will be negative
						T = (minA - maxB)/speed;
						if(T > firstTime)
						{
							firstTime = T;
							pFirstColisionAxis = &B_Normals[i];
							SideOfB = Side::Left;//Will be post-fix
							isNormalOfA = false;
						}
						//The Time of last collision in this axis
						T = (maxA - minB)/speed;
						if(T < lastTime)
							lastTime = T;
						if(firstTime > lastTime)//Not collide
							return false;
					}else if(speed < 0.0f)
					{
						//The Time of collision USE TO FIX INTERPENETRATION
						//will be negative
						T = (maxA - minB)/speed;
						if(T > firstTime)
						{
							firstTime = T;
							pFirstColisionAxis = &B_Normals[i];
							SideOfB = Side::Right;//Will be post-fix
							isNormalOfA = false;
						}
						//The Time of last collision in this axis
						T = (minA - maxB)/speed;
						if(T < lastTime)
							lastTime = T;
						if(firstTime > lastTime)//Not collide
							return false;
					}
				}
			}
		}
		#pragma endregion

		#pragma Fix Interpenetration
		//bool res = true;
		float Penetration = 0.0f;
		if( firstTime < 0.0f )
		{
			//firstTime -= 10.0f;
			//firstTime *= 1000.0f;
			firstTime -= 0.1f;
			//firstTime -= 200.0f;
			B.Move( B.GetVelocity()*firstTime );
			A.Move( A.GetVelocity()*firstTime );
			//B.Move( V*firstTime );
			firstTime = 0.0f;
			//firstTime = 0.0f;
			Collider col;
			//res = col.TestPolygonVsPolygon(A,B,Penetration);
			float auxfirst;
			float auxlast;
			CollisionContact cont;
			sf::Vector2<float> ColNorm;
			//res = col.FindPolygonVsPolygon(A,B,auxfirst,auxlast,cont,ColNorm,0.0f);
			//res = col.TestPolygonVsPolygon(A,B,Penetration);
			//if( res )
			if( col.FindPolygonVsPolygon(A,B,auxfirst,auxlast,cont,ColNorm,0.0f) )
				float aux = 99.0f*12.0f;
			//float aux1;
			//float aux2;
			//Collider::FindPolygonVsPolygon(A,B,firstTime,
		}
		#pragma endregion

		//With the time of collision(firstTime) and the Axis of Colision(pFirstColisionAxis)
		//we compute the kind of collision:
		//									a) Vertex-Vertex
		//									b) Vertex-Edge		or		Edge-Vertex
		//									c) Edge-Edge
		//Then:
		//	If it is Vertex-Vertex or Edge-Vertex the point of collision
		//	If it is Edge-Edge collision the segment of collision
		#pragma region Harvest Collision Info
		if( pFirstColisionAxis != NULL )
		{
			CollisionInfo InfoB;
			CollisionInfo InfoA;

			speed = Dot(V,*pFirstColisionAxis);
			ColisionNormalOfA = *pFirstColisionAxis;

			minB = minA = std::numeric_limits<float>::max();
			maxB = maxA = -std::numeric_limits<float>::max();
			sf::Vector2<float>* pColisionVNeighbor1 = NULL;
			sf::Vector2<float>* pColisionVNeighbor2 = NULL;
			if( SideOfB == Side::Left )
			{
				for(int j=0; j < B.m_Edges.size() ; ++j)
				{
					aux = Dot(B_Vertices[j], ColisionNormalOfA);
					(aux > maxB)? (maxB = aux, InfoB.pPoint1 = &B_Vertices[j], pColisionVNeighbor1 = &B_Vertices[(j+1)%B_Vertices.size()], pColisionVNeighbor2 = &B_Vertices[(j-1)%B_Vertices.size()]) : 0;
				}
				//aux = maxB;

				aux = Dot(*pColisionVNeighbor1, ColisionNormalOfA);
				if( (maxB - epi) < aux && aux < (maxB + epi) )
				{
					InfoB.Type = CollisionInfo::Edge;
					InfoB.pPoint2 = pColisionVNeighbor1;
				}else
				{
					aux = Dot(*pColisionVNeighbor2, ColisionNormalOfA);
					if( (maxB - epi) < aux && aux < (maxB + epi) )
					{
						InfoB.Type = CollisionInfo::Edge;
						InfoB.pPoint2 = pColisionVNeighbor2;
					}
				}
			}else
			{
				for(int j=0; j < B.m_Edges.size() ; ++j)
				{
					aux = Dot(B_Vertices[j], ColisionNormalOfA);
					(aux < minB)? (minB = aux, InfoB.pPoint1 = &B_Vertices[j], pColisionVNeighbor1 = &B_Vertices[(j+1)%B_Vertices.size()], pColisionVNeighbor2 = &B_Vertices[(j-1)%B_Vertices.size()]) : 0;
				}
				//aux = minB;

				aux = Dot(*pColisionVNeighbor1, ColisionNormalOfA);
				if( (minB - epi) < aux && aux < (minB + epi) )
				{
					InfoB.Type = CollisionInfo::Edge;
					InfoB.pPoint2 = pColisionVNeighbor1;
				}else
				{
					aux = Dot(*pColisionVNeighbor2, ColisionNormalOfA);
					if( (minB - epi) < aux && aux < (minB + epi) )
					{
						InfoB.Type = CollisionInfo::Edge;
						InfoB.pPoint2 = pColisionVNeighbor2;
					}
				}
			}

			pColisionVNeighbor1 = NULL;
			pColisionVNeighbor2 = NULL;
			if( SideOfB == Side::Right )//if B is right then A is left...
			{
				for(int j=0; j < A.m_Edges.size() ; ++j)
				{
					aux = Dot(A_Vertices[j], ColisionNormalOfA);
					(aux > maxA)? (maxA = aux, InfoA.pPoint1 = &A_Vertices[j], pColisionVNeighbor1 = &A_Vertices[(j+1)%A_Vertices.size()], pColisionVNeighbor2 = &A_Vertices[(j-1)%A_Vertices.size()]) : 0;
				}
				//aux = maxB;

				aux = Dot(*pColisionVNeighbor1, ColisionNormalOfA);
				if( (maxA - epi) < aux && aux < (maxA + epi) )
				{
					InfoA.Type = CollisionInfo::Edge;
					InfoA.pPoint2 = pColisionVNeighbor1;
				}else
				{
					aux = Dot(*pColisionVNeighbor2, ColisionNormalOfA);
					if( (maxA - epi) < aux && aux < (maxA + epi) )
					{
						InfoA.Type = CollisionInfo::Edge;
						InfoA.pPoint2 = pColisionVNeighbor2;
					}
				}
			}else//if B is left then A is right
			{
				for(int j=0; j < A.m_Edges.size() ; ++j)
				{
					aux = Dot(A_Vertices[j], ColisionNormalOfA);
					(aux < minA)? (minA = aux, InfoA.pPoint1 = &A_Vertices[j], pColisionVNeighbor1 = &A_Vertices[(j+1)%A_Vertices.size()], pColisionVNeighbor2 = &A_Vertices[(j-1)%A_Vertices.size()]) : 0;
				}
				//aux = minA;

				aux = Dot(*pColisionVNeighbor1, ColisionNormalOfA);
				if( (minA - epi) < aux && aux < (minA + epi) )
				{
					InfoA.Type = CollisionInfo::Edge;
					InfoA.pPoint2 = pColisionVNeighbor1;
				}else
				{
					aux = Dot(*pColisionVNeighbor2, ColisionNormalOfA);
					if( (minA - epi) < aux && aux < (minA + epi) )
					{
						InfoA.Type = CollisionInfo::Edge;
						InfoA.pPoint2 = pColisionVNeighbor2;
					}
				}
			}

			CollisionSurface.Type = CollisionContact::Point;
			if( InfoA.Type == InfoB.Type )
			{
				if( InfoA.Type == CollisionInfo::Vertex )//Vertex-Vertex Colision
					CollisionSurface.Point1 = (*InfoB.pPoint1)+B.GetVelocity()*firstTime;
				else									//Edge-Edge Colision
				{
					CollisionSurface.Type = CollisionContact::Edge;
					//Collision calculated from U as:     U_0 + t*W_0 + u * E_0
					//Where u_min -> ( U collision with V_0 -> stamp(0.0f,1.0f) )
					//Where u_max -> ( U collision with V_1 -> stamp(0.0f,1.0f) )
					sf::Vector2<float> E_0 = *InfoB.pPoint2-*InfoB.pPoint1;
					float E_0NormSquare = Dot(E_0,E_0);
					//u_min = E_0 * [(V_0+t*W_1) - (U_0+t*W_0)]/||E_0||^2
					//u_max = E_0 * [(V_1+t*W_1) - (U_0+t*W_0)]/||E_0||^2
					sf::Vector2<float> aux = firstTime*A.GetVelocity()-*InfoB.pPoint1-firstTime*B.GetVelocity();
					float u_min = Dot(E_0 , (*InfoA.pPoint1 + aux )/E_0NormSquare);
					float u_max = Dot(E_0 , (*InfoA.pPoint2 + aux )/E_0NormSquare);

					(u_min < 0.0f)? u_min=0.0f : 0;
					(u_min > 1.0f)? u_min=1.0f : 0;
					(u_max < 0.0f)? u_max=0.0f : 0;
					(u_max > 1.0f)? u_max=1.0f : 0;

					CollisionSurface.Point1 = *InfoB.pPoint1 + B.GetVelocity()*firstTime + E_0*u_min;
					CollisionSurface.Point2 = *InfoB.pPoint1 + B.GetVelocity()*firstTime + E_0*u_max;
				}
			}else
			{
				if( InfoA.Type == CollisionInfo::Vertex )//Vertex-Edge Colision
					CollisionSurface.Point1 = (*InfoA.pPoint1)+A.GetVelocity()*firstTime;
				else									//Edge-Vertex Colision
					CollisionSurface.Point1 = (*InfoB.pPoint1)+B.GetVelocity()*firstTime;
			}

			//if the collision normal is from B, fix it to be for A
			if(!isNormalOfA)
				ColisionNormalOfA = ColisionNormalOfA * (-1.0f);

		}
		#pragma endregion

		return true;//they collide in every posible separation axis then Colliding
	}

}