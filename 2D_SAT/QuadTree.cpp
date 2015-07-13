#include "QuadTree.h"
#include "Collider.h"

namespace Obj2D
{

	Node::Node(): pChild_NW(NULL), pChild_NE(NULL), pChild_SW(NULL), pChild_SE(NULL)
	{}

	Node::Node(float minX, float minY, float maxX, float maxY): pChild_NW(NULL), pChild_NE(NULL), pChild_SW(NULL), pChild_SE(NULL),
			Volume(minX,minY,maxX,maxY)
	{}

	bool Node::IsInside(const sf::Vector2<float> position) const
	{
		if( position.x <= Volume.m_MaxX && position.x >= Volume.m_MinX &&
			position.y <= Volume.m_MaxY && position.y >= Volume.m_MinY )
			return true;
		return false;
	}









	QuadTree::QuadTree(int maxNodeLeaf): m_MaxNodeLeaf(maxNodeLeaf)
	{}

	QuadTree::QuadTree(int maxNodeLeaf, float minXWorld, float minYWorld, float maxXWorld, float maxYWorld): m_MaxNodeLeaf(maxNodeLeaf),
			m_Root(minXWorld,minYWorld,maxXWorld,maxYWorld)
	{}

	void QuadTree::_DebugRender(const Node* node, sf::RenderWindow& Wnd) const
	{
		if( node->pChild_NW != NULL )//if 1 is NULL then all are NULL
		{
			_DebugRender(node->pChild_NW, Wnd);
			_DebugRender(node->pChild_NE, Wnd);
			_DebugRender(node->pChild_SW, Wnd);
			_DebugRender(node->pChild_SE, Wnd);
		}

		sf::Shape rectanle = sf::Shape::Rectangle(node->Volume.m_MinX, node->Volume.m_MinY,
			node->Volume.m_MaxX, node->Volume.m_MaxY, sf::Color::Color(255,255,255,0.0f), 3.0f, sf::Color::Yellow);
		Wnd.Draw(rectanle);
	}

	void QuadTree::_Add(Node* node, SpatialNode* newElement)
	{
		if( node->pChild_NW == NULL )//if 1 is NULL then all are NULL
		{
			if( node->ElementsPts.size()+1 > m_MaxNodeLeaf )
			{
				node->pChild_NW = new Node(node->Volume.m_MinX, node->Volume.m_MinY, (node->Volume.m_MinX+node->Volume.m_MaxX)/2.0f, (node->Volume.m_MinY+node->Volume.m_MaxY)/2.0f);
				node->pChild_NE = new Node((node->Volume.m_MinX+node->Volume.m_MaxX)/2.0f, node->Volume.m_MinY, node->Volume.m_MaxX, (node->Volume.m_MinY+node->Volume.m_MaxY)/2.0f);
				node->pChild_SW = new Node(node->Volume.m_MinX, (node->Volume.m_MinY+node->Volume.m_MaxY)/2.0f, (node->Volume.m_MinX+node->Volume.m_MaxX)/2.0f, node->Volume.m_MaxY);
				node->pChild_SE = new Node((node->Volume.m_MinX+node->Volume.m_MaxX)/2.0f, (node->Volume.m_MinY+node->Volume.m_MaxY)/2.0f, node->Volume.m_MaxX, node->Volume.m_MaxY);

				for(int i=0; i < node->ElementsPts.size() ; ++i)
					_Add(node, node->ElementsPts[i]);
				node->ElementsPts.clear();

			}else
			{	
				node->ElementsPts.push_back(newElement);
				return;
			}
		}

		sf::Vector2<float> pos = newElement->GetPosition();
		if( node->pChild_NW->IsInside(pos) )
		{
			_Add(node->pChild_NW, newElement);
		}else if( node->pChild_NE->IsInside(pos) )
		{
			_Add(node->pChild_NE, newElement);
		}else if( node->pChild_SW->IsInside(pos) )
		{
			_Add(node->pChild_SW, newElement);
		}else if( node->pChild_SE->IsInside(pos) )
		{
			_Add(node->pChild_SE, newElement);
		}
	}

	int QuadTree::_Count(const Node* node) const
	{
		if( node->pChild_NW == NULL )//if 1 is NULL then all are NULL
			return node->ElementsPts.size();

		int aux = 0;
		aux += _Count(node->pChild_NW);
		aux += _Count(node->pChild_NE);
		aux += _Count(node->pChild_SW);
		aux += _Count(node->pChild_SE);

		return aux;
	}

	void QuadTree::_Clear(Node* node)
	{
		if( node->pChild_NW == NULL )//if 1 is NULL then all are NULL
		{
			node->ElementsPts.clear();
			return;
		}

		_Clear(node->pChild_NW);
		delete node->pChild_NW;
		node->pChild_NW = NULL;

		_Clear(node->pChild_NE);
		delete node->pChild_NE;
		node->pChild_NE = NULL;

		_Clear(node->pChild_SW);
		delete node->pChild_SW;
		node->pChild_SW = NULL;

		_Clear(node->pChild_SE);
		delete node->pChild_SE;
		node->pChild_SE = NULL;
		return;
	}

	std::vector<SpatialNode*> QuadTree::_GetNearElements(const Node* node, const SpatialNode* element) const
	{
		if( node->pChild_NW == NULL )//if 1 is NULL then all are NULL
			return node->ElementsPts;

		Obj2D::Collider col;
		std::vector<SpatialNode*> NW_Elements;
		std::vector<SpatialNode*> NE_Elements;
		std::vector<SpatialNode*> SW_Elements;
		std::vector<SpatialNode*> SE_Elements;

		//aux is element->m_BoundingBoxSpatial converted to world positions
		//sf::Vector2<float> pos = element->GetPosition();
		//BoundingBox aux(element->m_BoundingBoxSpatial.m_MinX+pos.x, element->m_BoundingBoxSpatial.m_MinY+pos.y,
		//	element->m_BoundingBoxSpatial.m_MaxX+pos.x, element->m_BoundingBoxSpatial.m_MaxY+pos.y);
		
		if( col.TestAABBVsAABB( element->m_BoundingBoxSpatial, node->pChild_NW->Volume ) )
		{
			NW_Elements = _GetNearElements(node->pChild_NW, element);
		}

		if( col.TestAABBVsAABB( element->m_BoundingBoxSpatial, node->pChild_NE->Volume ) )
		{
			NE_Elements = _GetNearElements(node->pChild_NE, element);
		}

		if( col.TestAABBVsAABB( element->m_BoundingBoxSpatial, node->pChild_SW->Volume ) )
		{
			SW_Elements = _GetNearElements(node->pChild_SW, element);
		}

		if( col.TestAABBVsAABB( element->m_BoundingBoxSpatial, node->pChild_SE->Volume ) )
		{
			SE_Elements = _GetNearElements(node->pChild_SE, element);
		}

		//std::copy(NE_Elements.begin(), NE_Elements.end(), NW_Elements.end());
		for(int i=0; i < NE_Elements.size() ; ++i)
			NW_Elements.push_back( NE_Elements[i] );
		//std::copy(SW_Elements.begin(), SW_Elements.end(), NW_Elements.end());
		for(int i=0; i < SW_Elements.size() ; ++i)
			NW_Elements.push_back( SW_Elements[i] );
		//std::copy(SE_Elements.begin(), SE_Elements.end(), NW_Elements.end());
		for(int i=0; i < SE_Elements.size() ; ++i)
			NW_Elements.push_back( SE_Elements[i] );

		return NW_Elements;//All the elements;
	}

	std::vector<SpatialNode*> QuadTree::_GetNearElements(const Node* node, const sf::Vector2<float>& position) const
	{
		if( node->pChild_NW == NULL )//if 1 is NULL then all are NULL
			return node->ElementsPts;

		if( node->pChild_NW->IsInside(position) )
		{
			return _GetNearElements(node->pChild_NW, position);
		}else if( node->pChild_NE->IsInside(position) )
		{
			return _GetNearElements(node->pChild_NE,position);
		}else if( node->pChild_SW->IsInside(position) )
		{
			return _GetNearElements(node->pChild_SW, position);
		}else if( node->pChild_SE->IsInside(position) )
		{
			return _GetNearElements(node->pChild_SE, position);
		}
	}





	void QuadTree::AddElement(SpatialNode* newElement)
	{
		_Add(&m_Root, newElement);
	}

	void QuadTree::Construct(std::vector<SpatialNode*> Elements)
	{
		Clear();
		for(int i=0; i < Elements.size() ; ++i)
			_Add(&m_Root, Elements[i]);
	}

	void QuadTree::Clear()
	{
		_Clear(&m_Root);
	}

	int QuadTree::GetElementCount() const
	{
		return _Count(&m_Root);
	}

	std::vector<SpatialNode*> QuadTree::QueryNearElements(SpatialNode* element) const
	{
		return _GetNearElements(&m_Root, element);
	}

	std::vector<SpatialNode*> QuadTree::QueryNearElements(sf::Vector2<float> position) const
	{
		return _GetNearElements(&m_Root, position);
	}

	std::string QuadTree::GetLispTree(TransverseType type)
	{
		return "Not implemented";
	}

	void QuadTree::DebugRender(sf::RenderWindow& Wnd) const
	{
		_DebugRender(&m_Root, Wnd);
	}

}