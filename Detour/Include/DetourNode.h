//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef DETOURNODE_H
#define DETOURNODE_H

#include "DetourNavMesh.h"
#ifdef ZENGINE_NAVMESH
#include <Common.h>
#include <cstdint>
#include <cstring>
#include <cassert>
#include <cstdlib>
#endif // ZENGINE_NAVMESH

enum dtNodeFlags
{
	DT_NODE_OPEN = 0x01,
	DT_NODE_CLOSED = 0x02,
	DT_NODE_PARENT_DETACHED = 0x04, // parent of the node is not adjacent. Found using raycast.
};

typedef unsigned short dtNodeIndex;
static const dtNodeIndex DT_NULL_IDX = (dtNodeIndex)~0;

static const int DT_NODE_PARENT_BITS = 24;
static const int DT_NODE_STATE_BITS = 2;
struct dtNode
{
	float pos[3];								///< Position of the node.
	float cost;									///< Cost from previous node to current node.
	float total;								///< Cost up to the node.
	unsigned int pidx : DT_NODE_PARENT_BITS;	///< Index to parent node.
	unsigned int state : DT_NODE_STATE_BITS;	///< extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE
	unsigned int flags : 3;						///< Node flags. A combination of dtNodeFlags.
	dtPolyRef id;								///< Polygon ref the node corresponds to.
};

static const int DT_MAX_STATES_PER_NODE = 1 << DT_NODE_STATE_BITS;	// number of extra states per node. See dtNode::state

class dtNodePool
{
public:
	dtNodePool(int maxNodes, int hashSize);
	~dtNodePool();
	void clear();

	// Get a dtNode by ref and extra state information. If there is none then - allocate
	// There can be more than one node for the same polyRef but with different extra state information
	dtNode* getNode(dtPolyRef id, unsigned char state=0);	
	dtNode* findNode(dtPolyRef id, unsigned char state);
	unsigned int findNodes(dtPolyRef id, dtNode** nodes, const int maxNodes);

	inline unsigned int getNodeIdx(const dtNode* node) const
	{
		if (!node) return 0;
		return (unsigned int)(node - m_nodes) + 1;
	}

	inline dtNode* getNodeAtIdx(unsigned int idx)
	{
		if (!idx) return 0;
		return &m_nodes[idx - 1];
	}

	inline const dtNode* getNodeAtIdx(unsigned int idx) const
	{
		if (!idx) return 0;
		return &m_nodes[idx - 1];
	}
	
	inline int getMemUsed() const
	{
		return sizeof(*this) +
			sizeof(dtNode)*m_maxNodes +
			sizeof(dtNodeIndex)*m_maxNodes +
			sizeof(dtNodeIndex)*m_hashSize;
	}
	
	inline int getMaxNodes() const { return m_maxNodes; }
	
	inline int getHashSize() const { return m_hashSize; }
	inline dtNodeIndex getFirst(int bucket) const { return m_first[bucket]; }
	inline dtNodeIndex getNext(int i) const { return m_next[i]; }
	inline int getNodeCount() const { return m_nodeCount; }
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtNodePool(const dtNodePool&);
	dtNodePool& operator=(const dtNodePool&);
	
	dtNode* m_nodes;
	dtNodeIndex* m_first;
	dtNodeIndex* m_next;
	const int m_maxNodes;
	const int m_hashSize;
	int m_nodeCount;
};

class dtNodeQueue
{
public:
	dtNodeQueue(int n);
	~dtNodeQueue();
	
	inline void clear() { m_size = 0; }
	
	inline dtNode* top() { return m_heap[0]; }
	
	inline dtNode* pop()
	{
		dtNode* result = m_heap[0];
		m_size--;
		trickleDown(0, m_heap[m_size]);
		return result;
	}
	
	inline void push(dtNode* node)
	{
		m_size++;
		bubbleUp(m_size-1, node);
	}
	
	inline void modify(dtNode* node)
	{
		for (int i = 0; i < m_size; ++i)
		{
			if (m_heap[i] == node)
			{
				bubbleUp(i, node);
				return;
			}
		}
	}
	
	inline bool empty() const { return m_size == 0; }
	
	inline int getMemUsed() const
	{
		return sizeof(*this) +
		sizeof(dtNode*) * (m_capacity + 1);
	}
	
	inline int getCapacity() const { return m_capacity; }
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtNodeQueue(const dtNodeQueue&);
	dtNodeQueue& operator=(const dtNodeQueue&);

	void bubbleUp(int i, dtNode* node);
	void trickleDown(int i, dtNode* node);
	
	dtNode** m_heap;
	const int m_capacity;
	int m_size;
};

#ifdef ZENGINE_NAVMESH

struct dtNodeJmp
{
	uint32_t transferTypeFromPrevPoly;
	float posJmpFromPrevPoly[3];
	float pos[3];								///< Position of the node (entrance position for poly with id over previous poly)
	float cost;									///< Cost from previous node to current node.
	float total;								///< Cost up to the node.
	unsigned int pidx : DT_NODE_PARENT_BITS;	///< Index to parent node.
	unsigned int state : DT_NODE_STATE_BITS;	///< extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE
	unsigned int flags : 3;						///< Node flags. A combination of dtNodeFlags.
	dtPolyRef id;								///< Polygon ref the node corresponds to.
};

class dtNodePoolJmp: private common::NonCopyable
{
public:
	dtNodePoolJmp(int maxNodes, int hashSize):
		m_nodes(0),
		m_first(0),
		m_next(0),
		m_maxNodes(maxNodes),
		m_hashSize(hashSize),
		m_nodeCount(0)
	{
		assert(dtNextPow2(m_hashSize) == (unsigned int)m_hashSize);
		// pidx is special as 0 means "none" and 1 is the first node. For that reason
		// we have 1 fewer nodes available than the number of values it can contain.
		assert(m_maxNodes > 0 && m_maxNodes <= DT_NULL_IDX && m_maxNodes <= (1 << DT_NODE_PARENT_BITS) - 1);

		m_nodes = (dtNodeJmp*)std::malloc(sizeof(dtNodeJmp) * m_maxNodes);
		m_next = (dtNodeIndex*)std::malloc(sizeof(dtNodeIndex) * m_maxNodes);
		m_first = (dtNodeIndex*)std::malloc(sizeof(dtNodeIndex) * hashSize);

		assert(m_nodes);
		assert(m_next);
		assert(m_first);

		std::memset(m_first, 0xff, sizeof(dtNodeIndex) * m_hashSize);
		std::memset(m_next, 0xff, sizeof(dtNodeIndex) * m_maxNodes);
	}

	~dtNodePoolJmp()
	{
		std::free(m_nodes);
		std::free(m_next);
		std::free(m_first);
	}

	void clear()
	{
		std::memset(m_first, 0xff, sizeof(dtNodeIndex) * m_hashSize);
		m_nodeCount = 0;
	}

	uint32_t dtHashRef(dtPolyRef id) { return -1; }

	// Get a dtNode by ref and extra state information. If there is none then - allocate
	// There can be more than one node for the same polyRef but with different extra state information
	dtNodeJmp* getNode(dtPolyRef id, const uint32_t transferType, const uint8_t state)
	{
		unsigned int bucket = dtHashRef(id) & (m_hashSize - 1);
		dtNodeIndex i = m_first[bucket];
		dtNodeJmp* node = nullptr;
		if (transferType <= NavmeshPolyTransferFlags::WALKING)
		{
			while (i != DT_NULL_IDX)
			{
				if (
					m_nodes[i].id == id &&
					m_nodes[i].transferTypeFromPrevPoly == transferType &&
					m_nodes[i].state == state
				) {
					return &m_nodes[i];
				}
				i = m_next[i];
			}
		}

		if (m_nodeCount >= m_maxNodes)
			return nullptr;

		i = (dtNodeIndex)m_nodeCount;
		m_nodeCount++;

		// Init node
		node = &m_nodes[i];
		node->transferTypeFromPrevPoly = transferType;
		node->pidx = 0;
		node->cost = 0;
		node->total = 0;
		node->id = id;
		node->state = state;
		node->flags = 0;

		m_next[i] = m_first[bucket];
		m_first[bucket] = i;

		return node;
	}

	/*
	dtNodeJmp* findNode(dtPolyRef id, const uint32_t transferType, const uint8_t state)
	{
		unsigned int bucket = dtHashRef(id) & (m_hashSize - 1);
		dtNodeIndex i = m_first[bucket];
		while (i != DT_NULL_IDX)
		{
			if (
				m_nodes[i].id == id &&
				m_nodes[i].transferType == transferType &&
				m_nodes[i].state == state
			) {
				return &m_nodes[i];
			}
			i = m_next[i];
		}
		return 0;
	}
	*/

	inline unsigned int getNodeIdx(const dtNodeJmp* node) const
	{
		if (!node) return 0;
		return (unsigned int)(node - m_nodes) + 1;
	}

	inline dtNodeJmp* getNodeAtIdx(unsigned int idx)
	{
		if (!idx) return 0;
		return &m_nodes[idx - 1];
	}

	inline const dtNodeJmp* getNodeAtIdx(unsigned int idx) const
	{
		if (!idx) return 0;
		return &m_nodes[idx - 1];
	}

	inline int getMemUsed() const
	{
		return sizeof(*this) +
			sizeof(dtNodeJmp) * m_maxNodes +
			sizeof(dtNodeIndex) * m_maxNodes +
			sizeof(dtNodeIndex) * m_hashSize;
	}

	inline int getMaxNodes() const { return m_maxNodes; }

	inline int getHashSize() const { return m_hashSize; }
	inline dtNodeIndex getFirst(int bucket) const { return m_first[bucket]; }
	inline dtNodeIndex getNext(int i) const { return m_next[i]; }
	inline int getNodeCount() const { return m_nodeCount; }

private:
	dtNodeJmp* m_nodes;
	dtNodeIndex* m_first;
	dtNodeIndex* m_next;
	const int m_maxNodes;
	const int m_hashSize;
	int m_nodeCount;
};

class dtNodeQueueJmp: private common::NonCopyable
{
public:
	dtNodeQueueJmp(int n):
		m_heap(0),
		m_capacity(n),
		m_size(0)
	{
		assert(m_capacity > 0);

		m_heap = (dtNodeJmp**)std::malloc(sizeof(dtNodeJmp*) * (m_capacity + 1));
		assert(m_heap);
	}

	~dtNodeQueueJmp()
	{
		std::free(m_heap);
	}

	inline void clear() { m_size = 0; }

	inline dtNodeJmp* top() { return m_heap[0]; }

	inline dtNodeJmp* pop()
	{
		dtNodeJmp* result = m_heap[0];
		m_size--;
		trickleDown(0, m_heap[m_size]);
		return result;
	}

	inline void push(dtNodeJmp* node)
	{
		m_size++;
		bubbleUp(m_size - 1, node);
	}

	inline void modify(dtNodeJmp* node)
	{
		for (int i = 0; i < m_size; ++i)
		{
			if (m_heap[i] == node)
			{
				bubbleUp(i, node);
				return;
			}
		}
	}

	inline bool empty() const { return m_size == 0; }

	inline int getMemUsed() const
	{
		return sizeof(*this) +
			sizeof(dtNode*) * (m_capacity + 1);
	}

	inline int getCapacity() const { return m_capacity; }

private:
	void bubbleUp(int i, dtNodeJmp* node)
	{
		int parent = (i - 1) / 2;
		// note: (index > 0) means there is a parent
		while ((i > 0) && (m_heap[parent]->total > node->total))
		{
			m_heap[i] = m_heap[parent];
			i = parent;
			parent = (i - 1) / 2;
		}
		m_heap[i] = node;
	}

	void trickleDown(int i, dtNodeJmp* node)
	{
		int child = (i * 2) + 1;
		while (child < m_size)
		{
			if (((child + 1) < m_size) &&
				(m_heap[child]->total > m_heap[child + 1]->total))
			{
				child++;
			}
			m_heap[i] = m_heap[child];
			i = child;
			child = (i * 2) + 1;
		}
		bubbleUp(i, node);
	}

private:
	dtNodeJmp** m_heap;
	const int m_capacity;
	int m_size;
};

#endif // ZENGINE_NAVMESH


#endif // DETOURNODE_H
