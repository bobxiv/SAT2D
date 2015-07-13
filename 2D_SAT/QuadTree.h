#pragma once

#include "SFML//Graphics.hpp"
#include "SFML//System.hpp"
#include "SFML//Window.hpp"
#include "SFML//Config.hpp"

#include <vector>

#include "RigidBody.h"

namespace Obj2D
{

	struct Node
	{
		Node();

		Node(float minX, float minY, float maxX, float maxY);

		BoundingBox					Volume;

		Node*						pChild_NW;//North-West
		Node*						pChild_NE;//North-East
		Node*						pChild_SW;//South-West
		Node*						pChild_SE;//South-East

		std::vector<SpatialNode*>	ElementsPts;

		bool IsInside(const sf::Vector2<float> position) const;
	};

	class QuadTree
	{
	private:
		//The maximun number of leaf in a node
		int			m_MaxNodeLeaf;

		Node		m_Root;

		void _DebugRender(const Node* node, sf::RenderWindow& Wnd) const;

		void _Add(Node* node, SpatialNode* newElement);

		int _Count(const Node* node) const;

		void _Clear(Node* node);

		std::vector<SpatialNode*> _GetNearElements(const Node* node, const SpatialNode* element) const;

		std::vector<SpatialNode*> _GetNearElements(const Node* node, const sf::Vector2<float>& position) const;

	public:

		enum TransverseType{ Prefix, Innerfix, Postfix };

		QuadTree(int maxNodeLeaf);

		QuadTree(int maxNodeLeaf, float minXWorld, float minYWorld, float maxXWorld, float maxYWorld);

		void AddElement(SpatialNode* newElement);

		void Construct(std::vector<SpatialNode*> Elements);

		void Clear();

		int GetElementCount() const;

		std::vector<SpatialNode*> QueryNearElements(SpatialNode* element) const;

		std::vector<SpatialNode*> QueryNearElements(sf::Vector2<float> position) const;

		std::string GetLispTree(TransverseType type);

		void DebugRender(sf::RenderWindow& Wnd) const;

	};

}