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

#include "DetourNode.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include "DetourCommon.h"
#include <string.h>

#ifdef DT_POLYREF64
// From Thomas Wang, https://gist.github.com/badboy/6267743
inline unsigned int dtHashRef(dtPolyRef a)
{
	a = (~a) + (a << 18); // a = (a << 18) - a - 1;
	a = a ^ (a >> 31);
	a = a * 21; // a = (a + (a << 2)) + (a << 4);
	a = a ^ (a >> 11);
	a = a + (a << 6);
	a = a ^ (a >> 22);
	return (unsigned int)a;
}
#else
inline unsigned int dtHashRef(dtPolyRef a)
{
	a += ~(a<<15);
	a ^=  (a>>10);
	a +=  (a<<3);
	a ^=  (a>>6);
	a += ~(a<<11);
	a ^=  (a>>16);
	return (unsigned int)a;
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////
dtNodePool::dtNodePool(int maxNodes, int hashSize) :
	m_nodes(0),
	m_first(0),
	m_next(0),
	m_maxNodes(maxNodes),
	m_hashSize(hashSize),
	m_nodeCount(0)
{
	dtAssert(dtNextPow2(m_hashSize) == (unsigned int)m_hashSize);
	// pidx is special as 0 means "none" and 1 is the first node. For that reason
	// we have 1 fewer nodes available than the number of values it can contain.
	dtAssert(m_maxNodes > 0 && m_maxNodes <= DT_NULL_IDX && m_maxNodes <= (1 << DT_NODE_PARENT_BITS) - 1);

	m_nodes = (dtNode*)dtAlloc(sizeof(dtNode)*m_maxNodes, DT_ALLOC_PERM);
	m_next = (dtNodeIndex*)dtAlloc(sizeof(dtNodeIndex)*m_maxNodes, DT_ALLOC_PERM);
	m_first = (dtNodeIndex*)dtAlloc(sizeof(dtNodeIndex)*hashSize, DT_ALLOC_PERM);

	dtAssert(m_nodes);
	dtAssert(m_next);
	dtAssert(m_first);

	memset(m_first, 0xff, sizeof(dtNodeIndex)*m_hashSize);
	memset(m_next, 0xff, sizeof(dtNodeIndex)*m_maxNodes);
}

dtNodePool::~dtNodePool()
{
	dtFree(m_nodes);
	dtFree(m_next);
	dtFree(m_first);
}

void dtNodePool::clear()
{
	memset(m_first, 0xff, sizeof(dtNodeIndex)*m_hashSize);
	m_nodeCount = 0;
}

unsigned int dtNodePool::findNodes(dtPolyRef id, dtNode** nodes, const int maxNodes)
{
	int n = 0;
	unsigned int bucket = dtHashRef(id) & (m_hashSize-1);
	dtNodeIndex i = m_first[bucket];
	while (i != DT_NULL_IDX)
	{
		if (m_nodes[i].id == id)
		{
			if (n >= maxNodes)
				return n;
			nodes[n++] = &m_nodes[i];
		}
		i = m_next[i];
	}

	return n;
}

dtNode* dtNodePool::findNode(dtPolyRef id, unsigned char state)
{
	unsigned int bucket = dtHashRef(id) & (m_hashSize-1);
	dtNodeIndex i = m_first[bucket];
	while (i != DT_NULL_IDX)
	{
		if (m_nodes[i].id == id && m_nodes[i].state == state)
			return &m_nodes[i];
		i = m_next[i];
	}
	return 0;
}

dtNode* dtNodePool::getNode(dtPolyRef id, unsigned char state)
{
	unsigned int bucket = dtHashRef(id) & (m_hashSize-1);
	dtNodeIndex i = m_first[bucket];
	dtNode* node = 0;
	while (i != DT_NULL_IDX)
	{
		if (m_nodes[i].id == id && m_nodes[i].state == state)
			return &m_nodes[i];
		i = m_next[i];
	}
	
	if (m_nodeCount >= m_maxNodes)
		return 0;
	
	i = (dtNodeIndex)m_nodeCount;
	m_nodeCount++;
	
	// Init node
	node = &m_nodes[i];
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


//////////////////////////////////////////////////////////////////////////////////////////
dtNodeQueue::dtNodeQueue(int n) :
	m_heap(0),
	m_capacity(n),
	m_size(0)
{
	dtAssert(m_capacity > 0);
	
	m_heap = (dtNode**)dtAlloc(sizeof(dtNode*)*(m_capacity+1), DT_ALLOC_PERM);
	dtAssert(m_heap);
}

dtNodeQueue::~dtNodeQueue()
{
	dtFree(m_heap);
}

void dtNodeQueue::bubbleUp(int i, dtNode* node)
{
	int parent = (i-1)/2;
	// note: (index > 0) means there is a parent
	while ((i > 0) && (m_heap[parent]->total > node->total))
	{
		m_heap[i] = m_heap[parent];
		i = parent;
		parent = (i-1)/2;
	}
	m_heap[i] = node;
}

void dtNodeQueue::trickleDown(int i, dtNode* node)
{
	int child = (i*2)+1;
	while (child < m_size)
	{
		if (((child+1) < m_size) && 
			(m_heap[child]->total > m_heap[child+1]->total))
		{
			child++;
		}
		m_heap[i] = m_heap[child];
		i = child;
		child = (i*2)+1;
	}
	bubbleUp(i, node);
}

#ifdef ZENGINE_NAVMESH

dtNodePoolJmp::dtNodePoolJmp() :
	m_nodes(0),
	m_first(0),
	m_next(0),
	m_maxNodes(0),
	m_hashSize(0),
	m_nodeCount(0)
{}

bool dtNodePoolJmp::init(int maxNodes, int hashSize)
{
	m_maxNodes = maxNodes;
	m_hashSize = hashSize;
	assert(dtNextPow2(m_hashSize) == (unsigned int)m_hashSize);
	// pidx is special as 0 means "none" and 1 is the first node. For that reason
	// we have 1 fewer nodes available than the number of values it can contain.
	assert(m_maxNodes > 0 && m_maxNodes <= DT_NULL_IDX && m_maxNodes <= (1 << DT_NODE_PARENT_BITS) - 1);

	m_nodes = (dtNodeJmp*)std::malloc(sizeof(dtNodeJmp) * m_maxNodes);
	m_next = (dtNodeIndex*)std::malloc(sizeof(dtNodeIndex) * m_maxNodes);
	m_first = (dtNodeIndex*)std::malloc(sizeof(dtNodeIndex) * hashSize);
	if (!m_nodes || !m_next || !m_first) {
		clear();
		return false;
	}

	std::memset(m_first, 0xff, sizeof(dtNodeIndex) * m_hashSize);
	std::memset(m_next, 0xff, sizeof(dtNodeIndex) * m_maxNodes);
	
	return true;
}

dtNodePoolJmp::~dtNodePoolJmp()
{
	clear();
}

void dtNodePoolJmp::clear()
{
	std::free(m_nodes);
	m_nodes = nullptr;
	std::free(m_next);
	m_next = nullptr;
	std::free(m_first);
	m_first = nullptr;
	m_maxNodes = m_hashSize = m_nodeCount = 0;
}

void dtNodePoolJmp::clearState()
{
	std::memset(m_first, 0xff, sizeof(dtNodeIndex) * m_hashSize);
	m_nodeCount = 0;
}

dtNodeJmp* dtNodePoolJmp::getNode(dtPolyRef id, const uint32_t transferType, const uint8_t state)
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

dtNodeQueueJmp::dtNodeQueueJmp():
	m_heap(0),
	m_capacity(0),
	m_size(0)
{}

bool dtNodeQueueJmp::init(int heapSize)
{
	m_heap = (dtNodeJmp**)std::malloc(sizeof(dtNodeJmp*) * (heapSize + 1));
	if (!m_heap)
		return false;
	m_capacity = heapSize;
	return true;
}

void dtNodeQueueJmp::clear()
{
	std::free(m_heap);
	m_heap = nullptr;
	m_capacity = m_size = 0;
}

dtNodeQueueJmp::~dtNodeQueueJmp()
{
	clear();
}

void dtNodeQueueJmp::bubbleUp(int i, dtNodeJmp* node)
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

void dtNodeQueueJmp::trickleDown(int i, dtNodeJmp* node)
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

#endif // ZENGINE_NAVMESH
