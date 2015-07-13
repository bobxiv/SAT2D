#pragma once

#include "SFML//Graphics.hpp"
#include "SFML//System.hpp"
#include "SFML//Window.hpp"
#include "SFML//Config.hpp"

template< class T >
sf::Vector3<T> Cross(sf::Vector3<T> A, sf::Vector3<T> B)
{
	return sf::Vector3<T>( A.y*B.z-A.z*B.y , A.z*B.x-A.x*B.z , A.x*B.y-A.y*B.x );
}

template< class T >
T Dot(sf::Vector2<T> A, sf::Vector2<T> B)
{
	return ( A.x*B.x + A.y*B.y );
}

template< class T >
T Norm(sf::Vector3<T> A)
{
	return sqrt(A.x*A.x+A.y*A.y+A.z*A.z);
}

template< class T >
T Norm(sf::Vector2<T> A)
{
	return sqrt(A.x*A.x+A.y*A.y);
}

////////////////////////////////////////////////////////////
/// Operator * overload ; multiply a vector by a scalar value
///
/// \param V : Vector
/// \param X : Scalar value
///
/// \return V * X
///
////////////////////////////////////////////////////////////
template <typename T>
sf::Vector2<T> operator *(const sf::Vector2<T>& A, const sf::Vector2<T>& B)
{
	return sf::Vector2<float>(A.x*B.x,A.y*B.y);
}