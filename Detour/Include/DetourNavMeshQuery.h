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

#ifndef DETOURNAVMESHQUERY_H
#define DETOURNAVMESHQUERY_H

#include "DetourNavMesh.h"
#include "DetourStatus.h"
#include "DetourCommon.h"

#ifdef ZENGINE_NAVMESH
#include <cstring>
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cassert>
#include <memory>
#include <cmath>
#include <new>
#include <tuple>
#include <type_traits>
#include <utility>
#include "Geometry.h"
#include "Common.h"
#include "Mesh.h"
#endif // ZENGINE_NAVMESH

// Define DT_VIRTUAL_QUERYFILTER if you wish to derive a custom filter from dtQueryFilter.
// On certain platforms indirect or virtual function call is expensive. The default
// setting is to use non-virtual functions, the actual implementations of the functions
// are declared as inline for maximum speed. 

//#define DT_VIRTUAL_QUERYFILTER 1

/// Defines polygon filtering and traversal costs for navigation mesh query operations.
/// @ingroup detour
class dtQueryFilter
{
	float m_areaCost[DT_MAX_AREAS];		///< Cost per area type. (Used by default implementation.)
	unsigned short m_includeFlags;		///< Flags for polygons that can be visited. (Used by default implementation.)
	unsigned short m_excludeFlags;		///< Flags for polygons that should not be visted. (Used by default implementation.)
	
public:
	dtQueryFilter();
	
#ifdef DT_VIRTUAL_QUERYFILTER
	virtual ~dtQueryFilter() { }
#endif
	
	/// Returns true if the polygon can be visited.  (I.e. Is traversable.)
	///  @param[in]		ref		The reference id of the polygon test.
	///  @param[in]		tile	The tile containing the polygon.
	///  @param[in]		poly  The polygon to test.
#ifdef DT_VIRTUAL_QUERYFILTER
	virtual bool passFilter(const dtPolyRef ref,
							const dtMeshTile* tile,
							const dtPoly* poly) const;
#else
	bool passFilter(const dtPolyRef ref,
					const dtMeshTile* tile,
					const dtPoly* poly) const;
#endif

	/// Returns cost to move from the beginning to the end of a line segment
	/// that is fully contained within a polygon.
	///  @param[in]		pa			The start position on the edge of the previous and current polygon. [(x, y, z)]
	///  @param[in]		pb			The end position on the edge of the current and next polygon. [(x, y, z)]
	///  @param[in]		prevRef		The reference id of the previous polygon. [opt]
	///  @param[in]		prevTile	The tile containing the previous polygon. [opt]
	///  @param[in]		prevPoly	The previous polygon. [opt]
	///  @param[in]		curRef		The reference id of the current polygon.
	///  @param[in]		curTile		The tile containing the current polygon.
	///  @param[in]		curPoly		The current polygon.
	///  @param[in]		nextRef		The refernece id of the next polygon. [opt]
	///  @param[in]		nextTile	The tile containing the next polygon. [opt]
	///  @param[in]		nextPoly	The next polygon. [opt]
#ifdef DT_VIRTUAL_QUERYFILTER
	virtual float getCost(const float* pa, const float* pb,
						  const dtPolyRef prevRef, const dtMeshTile* prevTile, const dtPoly* prevPoly,
						  const dtPolyRef curRef, const dtMeshTile* curTile, const dtPoly* curPoly,
						  const dtPolyRef nextRef, const dtMeshTile* nextTile, const dtPoly* nextPoly) const;
#else
	float getCost(const float* pa, const float* pb,
				  const dtPolyRef prevRef, const dtMeshTile* prevTile, const dtPoly* prevPoly,
				  const dtPolyRef curRef, const dtMeshTile* curTile, const dtPoly* curPoly,
				  const dtPolyRef nextRef, const dtMeshTile* nextTile, const dtPoly* nextPoly) const;
#endif

#ifdef ZENGINE_NAVMESH
	static float calcJumpClimbCost(const float* from, const float* to, const float* pathStart, float scale);
	static float calcNonWalkCoeff(const uint16_t transferType);
#endif // ZENGINE_NAVMESH

	/// @name Getters and setters for the default implementation data.
	///@{

	/// Returns the traversal cost of the area.
	///  @param[in]		i		The id of the area.
	/// @returns The traversal cost of the area.
	inline float getAreaCost(const int i) const { return m_areaCost[i]; }

	/// Sets the traversal cost of the area.
	///  @param[in]		i		The id of the area.
	///  @param[in]		cost	The new cost of traversing the area.
	inline void setAreaCost(const int i, const float cost) { m_areaCost[i] = cost; } 

	/// Returns the include flags for the filter.
	/// Any polygons that include one or more of these flags will be
	/// included in the operation.
	inline unsigned short getIncludeFlags() const { return m_includeFlags; }

	/// Sets the include flags for the filter.
	/// @param[in]		flags	The new flags.
	inline void setIncludeFlags(const unsigned short flags) { m_includeFlags = flags; }

	/// Returns the exclude flags for the filter.
	/// Any polygons that include one ore more of these flags will be
	/// excluded from the operation.
	inline unsigned short getExcludeFlags() const { return m_excludeFlags; }

	/// Sets the exclude flags for the filter.
	/// @param[in]		flags		The new flags.
	inline void setExcludeFlags(const unsigned short flags) { m_excludeFlags = flags; }	

	///@}

};

/// Provides information about raycast hit
/// filled by dtNavMeshQuery::raycast
/// @ingroup detour
struct dtRaycastHit
{
	/// The hit parameter. (FLT_MAX if no wall hit.)
	float t; 
	
	/// hitNormal	The normal of the nearest wall hit. [(x, y, z)]
	float hitNormal[3];

	/// The index of the edge on the final polygon where the wall was hit.
	int hitEdgeIndex;
	
	/// Pointer to an array of reference ids of the visited polygons. [opt]
	dtPolyRef* path;
	
	/// The number of visited polygons. [opt]
	int pathCount;

	/// The maximum number of polygons the @p path array can hold.
	int maxPath;

	///  The cost of the path until hit.
	float pathCost;
};

/// Provides custom polygon query behavior.
/// Used by dtNavMeshQuery::queryPolygons.
/// @ingroup detour
class dtPolyQuery
{
public:
	virtual ~dtPolyQuery() { }

	/// Called for each batch of unique polygons touched by the search area in dtNavMeshQuery::queryPolygons.
	/// This can be called multiple times for a single query.
	virtual void process(const dtMeshTile* tile, dtPoly** polys, dtPolyRef* refs, int count) = 0;
};

#ifdef ZENGINE_NAVMESH

template <int MAX_PLANES_SIZE, int FOUND_POLYS_ARR_SIZE>
class dtFindCollidedPolysQuery: public dtPolyQuery, private common::NonCopyable
{
public:
	static const int MAX_PLANES_SIZE_VAL = MAX_PLANES_SIZE;
	static const int FOUND_POLYS_ARR_SIZE_VAL = FOUND_POLYS_ARR_SIZE;

private:
	using ObpType = typename std::conditional<
		MAX_PLANES_SIZE == geometry::Obb::DIRS_SIZE,
		geometry::Obb,
		geometry::YAlignedObp<MAX_PLANES_SIZE>
	>::type;

	template <int I>
	struct Strategy
	{
		template <typename ... Args>
		static bool execute(Args&& ... args) {
			return geometry::intersectionYaobpVsPolygon<ObpType, DT_VERTS_PER_POLYGON>(std::forward<Args>(args)...);
		}
		template <typename T, typename ... Args>
		static void init(T& obb, Args&& ... args) {
			obb.init(std::forward<Args>(args)...);
		}
	};
	template <>
	struct Strategy<geometry::Obb::DIRS_SIZE>
	{
		template <typename ... Args>
		static bool execute(Args&& ... args) {
			return geometry::intersectionObbVsPoly<DT_VERTS_PER_POLYGON>(std::forward<Args>(args)...);
		}
		template <typename T>
		static void init(T& obb, const float* dirs, const int dirsNum, const float* verts, const int vertsNum) {
			assert(dirsNum == geometry::Obb::DIRS_SIZE);
			assert(vertsNum == geometry::Obb::VERTS_SIZE);
			obb.init(dirs, verts);
		}
	};

public:
	template <typename T>
	dtFindCollidedPolysQuery(
		const T* navQuery,
		const float* dirs,
		const int dirsNum,
		const float* verts,
		const int vertsNum,
		const int maxNumFoundPolys
	):
		m_nav(navQuery->getAttachedNavMesh()),
		m_polysNum(),
		m_maxNumFoundPolys(std::min(maxNumFoundPolys, FOUND_POLYS_ARR_SIZE))
	{
		assert(maxNumFoundPolys <= FOUND_POLYS_ARR_SIZE);
		assert(maxNumFoundPolys >= 1);
		Strategy<MAX_PLANES_SIZE>::init(obp, dirs, dirsNum, verts, vertsNum);
	}

	~dtFindCollidedPolysQuery() = default;

	void clear() { m_polysNum = 0; }

	int getPolysNum() const { return m_polysNum; }

	dtPolyRef getPoly(const int idx) const
	{
		assert(idx < m_polysNum);
		return m_polys[idx];
	}

	int getPolys(dtPolyRef* dat, const int datSize) const
	{
		int num = std::min(datSize, m_polysNum);
		std::memcpy(dat, m_polys, num * sizeof(dtPolyRef));
		return num;
	}

	void process(const dtMeshTile* tile, dtPoly** polys, dtPolyRef* refs, int count) override
	{
		dtIgnoreUnused(polys);
		float polyNorm[3];
		float d = FLT_MAX;
		static const float EPS = 1e-4;

		for (int i = 0; i < count; ++i)
		{
			if (m_polysNum >= m_maxNumFoundPolys)
				break;
			const dtPoly* poly = polys[i];
			//m_nav->getTileAndPolyByRefUnsafe(ref, &tile, &poly);

			if (!poly->isAveragePolyInited()) {
				const float* v0 = &tile->verts[poly->verts[0] * 3];
				const float* v1 = &tile->verts[poly->verts[1] * 3];
				const float* v2 = &tile->verts[poly->verts[2] * 3];
				float e1[3], e2[3];
				geometry::vsub(e1, v0, v1);
				geometry::vsub(e1, v2, v1);
				geometry::vcross(polyNorm, e1, e2);
				if (geometry::vlen(polyNorm) < EPS) {
					// degradated, TODO try to use other vertices, if they exist
					continue;
				}
				d = -geometry::vdot(v0, polyNorm);
			}
			else {
				geometry::vcopy(polyNorm, poly->norm);
				d = poly->dist;
			}

			float projVerts[3 * DT_VERTS_PER_POLYGON];
			const int numPolyVerts = poly->vertCount;
			for (int j = 0; j < numPolyVerts; ++j) {
				const float* v = &tile->verts[poly->verts[j] * 3];
				geometry::calcVerticalVertexProjectionOnPlane(v, polyNorm, d, projVerts + j * 3);
			}

			if (Strategy<MAX_PLANES_SIZE>::execute(&obp, polyNorm, projVerts, numPolyVerts)) {
				m_polys[m_polysNum++] = refs[i];
			}
		}
	}

private:
	const dtNavMesh* m_nav;
	ObpType obp;
	int m_polysNum;
	int m_maxNumFoundPolys;
	dtPolyRef m_polys[FOUND_POLYS_ARR_SIZE];
};

struct AgentCharacteristics
{
	bool acrobat;
	bool canJmpFwd;
	bool canJmpDown;
	bool canFallFromLedge;        // npc hasn't animation of jumping
	bool canJmpDownWithDamage;    // aggressive jumping down with damage from big height
	bool canClimb;
	bool jumpingClimbingDisabled; // only walking and running
	float agentHeight;
	float agentLength; // depth
	float agentWidth;
	float maxJmpFwdDistance;
	float maxJmpFwdHeight;
	float stepHeight;
	float maxClimbHeight;
	float maxJmpDownDistance;
	float maxJmpDownHeight;
	float damagePerOneCoordUnit; // for canJmpDownWithDamage
	float maxTotalDamage;        // for canJmpDownWithDamage
	dtQueryFilter filter;
};

struct JumpingRet
{
	JumpingRet() = default;
	void setData(
		uint8_t transferType_,
		dtPolyRef polyRefFrom_,
		dtPolyRef polyRefTo_,
		const float* posFrom_,
		const float* posTo_
	) {
		transferType = transferType_;
		polyRefFrom = polyRefFrom_;
		polyRefTo = polyRefTo_;
		std::memcpy(posFrom, posFrom_, sizeof(posFrom));
		std::memcpy(posTo, posTo_, sizeof(posTo));
	}
	void clear() { std::memset(this, 0, sizeof(JumpingRet)); }

	uint8_t transferType;
	dtPolyRef polyRefFrom;
	float posFrom[3];
	dtPolyRef polyRefTo;
	float posTo[3];
};

template <int I>
class JumpingRetArr: private common::NonCopyable
{
public:
	static const int MAX_SIZE = I;

public:
	JumpingRetArr() = default;
	~JumpingRetArr() = default;

	JumpingRet* allocEntry() { assert(m_num != MAX_SIZE); return &m_arr[m_num++]; }
	JumpingRet* getEntry(const uint32_t idx) { return m_arr + idx; }
	const JumpingRet* getEntry(const uint32_t idx) const { return m_arr + idx; }
	void clear() { m_num = 0; }
	uint32_t getNum() const { return m_num; }

private:
	JumpingRet m_arr[MAX_SIZE];
	uint32_t m_num = 0;
};

using StdJmpArr = JumpingRetArr<MAX_TRANSFERS_PER_JMP_ACTION * JMP_ACTION_SIZE>;

class TotalJmpTransfers
{
public:
	static const int SIZE = DT_VERTS_PER_POLYGON + 1;
	static const int CLIMB_OVERLAPPED_INDEX = SIZE - 1;

public:
	void init() {
		for (int i = 0; i < SIZE; ++i) {
			auto* transfer = m_transfers + i;
			transfer->clear();
		}
	}
	void clear() {
		init();
	}
	StdJmpArr* get(int i) {
		assert(i >= 0 && i < SIZE);
		return &m_transfers[i];
	}
	const StdJmpArr* get(int i) const {
		assert(i >= 0 && i < SIZE);
		return &m_transfers[i];
	}
	bool hasTransfers(int i) const {
		assert(i >= 0 && i < SIZE);
		return m_transfers[i].getNum();
	}
	bool hasTransfers() const {
		for (int i = 0; i < SIZE; ++i) {
			auto* transfer = m_transfers + i;
			if (transfer->getNum())
				return true;
		}
		return false;
	}
	uint32_t getTotalNumTransfers() const {
		uint32_t num = 0;
		for (int i = 0; i < SIZE; ++i) {
			auto* transfer = m_transfers + i;
			num += transfer->getNum();
		}
		return num;
	}

private:
	StdJmpArr m_transfers[SIZE]; // by edge id + last index for overlapped climb
};

struct TransferDataEntry
{
	void setData(uint8_t eIdx, uint8_t tType, dtPolyRef refIdTo, const float* posFrom, const float* posTo)
	{
		edgeId = eIdx;
		transferType = tType;
		std::memcpy(pointFrom, posFrom, sizeof(float) * 3);
		std::memcpy(pointTo, posTo, sizeof(float) * 3);
		polyRefIdTo = refIdTo;
		float diff[3];
		geometry::vsub(diff, pointTo, pointFrom);
		roundedMaxIntGroundDst = (int16_t)std::ceil( std::sqrt(diff[0] * diff[0] + diff[1] * diff[1]) );
		roundedMaxIntHeightDst = (int16_t)std::ceil( std::abs(diff[1]) );
	}

	uint8_t edgeId; // edge index in range 0 - 5
	uint8_t transferType;
	int16_t roundedMaxIntGroundDst; // XZ distance
	int16_t roundedMaxIntHeightDst; // Y distance
	float pointFrom[3];
	//dtPolyRef polyRefIdFrom; // ref id is start poly
	float pointTo[3];
	dtPolyRef polyRefIdTo;
};

template <uint32_t TRANSFER_BLOCK_SZ = DT_VERTS_PER_POLYGON>
class JmpTransferCache final: private common::NonCopyable
{
public:
	using IndexType = uint32_t;
	using ScoreType = uint64_t;

	static const uint32_t TRANSFER_BLOCK_SIZE = TRANSFER_BLOCK_SZ;
	static const IndexType INVALID_BLOCK_IDX = -1;
	static const IndexType INVALID_HEAP_IDX = 0;

private:
	using JmpTransferCheckerType = bool(const AgentCharacteristics* info, const TransferDataEntry* entry);

	struct PolyEntry
	{
		void init()
		{
			numBlocks = 0;
			dataBlockIdx = INVALID_BLOCK_IDX;
		}

		int16_t numBlocks;
		IndexType dataBlockIdx;
	};

	struct TileEntry
	{
		void init()
		{
			std::memset(this, 0, sizeof(TileEntry));
			blocksBunchIdx = INVALID_BLOCK_IDX;
		}

		ScoreType heapScore;
		IndexType heapIdx;

		uint32_t polysSize;
		PolyEntry* polys;
		IndexType blocksBunchIdx;
	};

	template <int SZ = DT_VERTS_PER_POLYGON>
	struct TransfersDataEntry
	{
		static const uint32_t SIZE = SZ;

		IndexType getNextBlockIdx() const { return nextBlockIdx; }
		int getEntriesNum() const { return entriesNum; }
		void setNextBlockIdx(IndexType idx) { nextBlockIdx = idx; }
		void setEntriesNum(int num) { entriesNum = (uint8_t)num; }
		bool hasNextBlock() const { return nextBlockIdx != INVALID_BLOCK_IDX; }
		TransferDataEntry* allocEntry() { return entriesNum != SIZE ? &transfers[entriesNum++] : nullptr; }
		void setPolyArrIdx(const IndexType idx) { polyArrIdx = idx; }
		IndexType getPolyArrIdx() { return polyArrIdx; }
		void setBlocksBunchIdx(const IndexType idx) { blocksBunchIdx = idx; }
		IndexType getBlocksBunchIdx() { return blocksBunchIdx; }
		void clear() { entriesNum = 0; nextBlockIdx = polyArrIdx = blocksBunchIdx = INVALID_BLOCK_IDX; }

		uint8_t entriesNum;
		IndexType nextBlockIdx;
		IndexType polyArrIdx;
		IndexType blocksBunchIdx;
		TransferDataEntry transfers[SIZE];
	};
	using TransfersDataEntryType = TransfersDataEntry<TRANSFER_BLOCK_SIZE>;

public:
	static const uint32_t POOL_BLOCK_BYTES_SIZE = sizeof(TransfersDataEntryType);

private:
	template <typename T>
	class TransfersDataPool final: private common::NonCopyable
	{
	public:
		using Type = T;

	public:
		TransfersDataPool() = default;
		~TransfersDataPool() { clear(); }
		bool init(const uint32_t blocksSize)
		{
			assert(blocksSize);
			m_memManagment = true;
			m_freedBlocks = static_cast<uint32_t*>(std::malloc(sizeof(uint32_t) * blocksSize));
			m_data = static_cast<Type*>(std::malloc(sizeof(Type) * blocksSize));
			if (!m_data || !m_freedBlocks) {
				clear();
				return false;
			}
			m_blocksSize = blocksSize;
			clearState();
			return true;
		}
		static uint32_t calcMemSize(const uint32_t blocksSize)
		{
			return (sizeof(uint32_t) + sizeof(Type)) * blocksSize;
		}
		bool init(void* data, const uint32_t blocksSize)
		{
			assert(blocksSize);
			m_memManagment = false;
			m_freedBlocks = (uint32_t*)data;
			m_data = (Type*)((uint8_t*)data + sizeof(uint32_t) * blocksSize);
			m_blocksSize = blocksSize;
			clearState();
			return true;
		}
		void clearState()
		{
			for (int i = m_blocksSize - 1, j = 0; i >= 0; --i, ++j) {
				m_freedBlocks[i] = j;
			}
			m_freedIndex = m_blocksSize - 1;
		}
		void clear()
		{
			if (m_memManagment) {
				std::free(m_freedBlocks);
				std::free(m_data);
			}
			m_freedBlocks = nullptr;
			m_data = nullptr;
			m_blocksSize = 0;
			m_freedIndex = INVALID_BLOCK_IDX;
		}

		uint32_t allocBlock()
		{
			if (m_freedIndex == INVALID_BLOCK_IDX)
				return INVALID_BLOCK_IDX;
			return m_freedBlocks[m_freedIndex--];
		}
		void freeBlock(uint32_t blockId)
		{
			assert(m_freedIndex != m_blocksSize - 1);
			m_freedBlocks[++m_freedIndex] = blockId;
		}
		bool isEmpty() const
		{
			return m_freedIndex == INVALID_BLOCK_IDX;
		}
		int getNumberFreeBlocks() const
		{
			return m_freedIndex + 1;
		}
		int getSizeBlocks() const
		{
			return m_blocksSize;
		}
		Type* getData() { return m_data; }
		Type* getBlockData(uint32_t blockId)
		{
			return m_data + blockId;
		}

	private:
		bool m_memManagment = true;
		Type* m_data = nullptr;
		uint32_t m_blocksSize = 0;
		uint32_t* m_freedBlocks = nullptr;
		int m_freedIndex = INVALID_BLOCK_IDX;
	};
	using DataPoolType = TransfersDataPool<TransfersDataEntryType>;

	class MinPriorityQueue: private common::NonCopyable // binary heap
	{
	public:
		MinPriorityQueue() = default;
		bool init(const uint32_t size)
		{
			m_heapSize = calcHeapSize(size);
			m_heapNum = 1;
			assert(m_heapSize > size);
			m_memManagment = true;
			if ((!m_heapData = (TileEntry**)std::malloc(sizeof(TileEntry*) * m_heapSize))) {
				clear();
				return false;
			}
			std::memset(m_heapData, 0, sizeof(TileEntry*) * m_heapSize);
			return true;
		}
		static uint32_t calcMemSize(const uint32_t size)
		{
			return calcHeapSize(size) * sizeof(TileEntry*);
		}
		static uint32_t calcHeapSize(const uint32_t size)
		{
			return (uint32_t)(std::pow(2, std::ceil(std::log2(size)))) + 2;
		}
		bool init(void* data, const uint32_t size)
		{
			m_heapSize = size;
			m_heapNum = 1;
			m_memManagment = false;
			m_heapData = (TileEntry**)data;
			std::memset(m_heapData, 0, sizeof(TileEntry*) * m_heapSize);
			return true;
		}
		void clearState() { m_heapNum = 1; }
		void clear()
		{
			if (m_memManagment) {
				std::free(m_heapData);
			}
			m_heapData = nullptr;
			m_heapNum = 0;
			m_heapSize = 0;
		}
		~MinPriorityQueue()
		{
			clear();
		}

		TileEntry* getByIndex(const IndexType i)
		{
			assert(i <= m_heapNum);
			return *(m_heapData + i);
		}

		TileEntry* getMin()
		{
			return *(m_heapData + 1);
		}

		inline bool empty() const { return m_heapNum == 1; }
		inline uint32_t getNum() const { return m_heapNum - 1; }
		inline bool noMemory() const { return m_heapNum == m_heapSize; }

		void setScoreByIndex(const IndexType i, const ScoreType score)
		{
			assert(i < m_heapNum);
			TileEntry* dat = *(m_heapData + i);
			assert(dat->heapScore < score);
			dat->heapScore = score;
			moveDown(i);
		}

		bool append(TileEntry* te)
		{
			if (noMemory())
				return false;
			m_heapData[m_heapNum] = te;
			++m_heapNum;
			moveUp(m_heapNum - 1);
			return true;
		}

		TileEntry* removeMin()
		{
			assert(m_heapNum > 1);
			std::swap(m_heapData[1], m_heapData[m_heapNum]);
			--m_heapNum;
			if (m_heapNum > 1) moveDown(1);
			return m_heapData[m_heapNum + 1];
		}

	private:
		inline IndexType parentIdx(const IndexType i) { return i >> 1; }
		inline IndexType leftChildIdx(const IndexType i) { return i << 1; }
		inline IndexType rightChildIdx(const IndexType i) { return i << 1 | 1; }

		void moveDown(IndexType idx)
		{
			assert(idx < m_heapNum);
			IndexType newIdx = idx;
			TileEntry* dat = *(m_heapData + idx);
			const ScoreType score = dat->heapScore;
			while (newIdx < m_heapNum)
			{
				const IndexType leftIdx = leftChildIdx(newIdx);
				const IndexType rightIdx = rightChildIdx(newIdx);
				if (leftIdx < m_heapNum && (*(m_heapData + leftIdx))->heapScore < score) {
					newIdx = leftIdx;
				}
				else if (rightIdx < m_heapNum && (*(m_heapData + rightIdx))->heapScore < score) {
					newIdx = rightIdx;
				}

				if (newIdx == idx) break;
				std::swap(m_heapData[idx], m_heapData[newIdx]);
				(*(m_heapData + idx))->heapIdx = idx;
				idx = newIdx;
			}
			assert(newIdx == idx);
			dat->heapIdx = newIdx;
		}

		void moveUp(IndexType idx)
		{
			assert(idx + 1 == m_heapNum);
			TileEntry* dat = *(m_heapData + idx);
			const ScoreType score = dat->heapScore;
			IndexType newIdx = parentIdx(idx);
			while (newIdx && (*(m_heapData + newIdx))->heapScore > score)
			{
				std::swap(m_heapData[idx], m_heapData[newIdx]);
				(*(m_heapData + idx))->heapIdx = idx;
				idx = newIdx;
				newIdx = parentIdx(idx);
			}
			assert(idx && idx != newIdx);
			dat->heapIdx = idx;
		}

	private:
		bool m_memManagment = true;
		uint32_t m_heapNum = 0;
		uint32_t m_heapSize = 0;
		TileEntry** m_heapData = nullptr;
	};

public:
	JmpTransferCache() = default;
	~JmpTransferCache() { clear(); }

	static void calcTilesAndPolygons(const dtNavMesh* nav, uint32_t& tilesNum, uint32_t& polysNum)
	{
		tilesNum = nav->getMaxTiles();
		polysNum = 0;
		for (uint32_t i = 0; i < tilesNum; ++i)
		{
			const dtMeshTile* tile = nav->getTile(i);
			if (!tile->header) continue;
			polysNum += tile->header->polyCount;
		}
	}

	bool init(
		const dtNavMesh* nav,
		const uint32_t tilesSize,
		const uint32_t polysSize,
		const uint32_t casheBlocksSize,
		JmpTransferCheckerType* jmpTransferChecker
	) {
		assert(tilesSize && polysSize && casheBlocksSize && jmpTransferChecker);
		m_tilesNum = nav->getMaxTiles();
		if (!m_tilesNum)
			return false;
		m_dataSize = calcMemSize(tilesSize, polysSize, casheBlocksSize);
		m_data = std::malloc(m_dataSize);
		if (!m_data) {
			m_dataSize = 0;
			return false;
		}
		m_tilesSize = tilesSize;
		m_polysSize = polysSize;
		m_nav = nav;
		m_jmpTransferChecker = jmpTransferChecker;

		void* polyDataEnd = clearStateInternal();
		const uint32_t queueMemSize = m_pqTilesUsage.calcMemSize(tilesSize);
		m_pqTilesUsage.init(polyDataEnd, queueMemSize);
		m_pool.init((uint8_t*)polyDataEnd + queueMemSize, casheBlocksSize);

		return true;
	}

	static uint32_t calcMemSize(
		const uint32_t tilesSize, const uint32_t polysSize, const uint32_t casheBlocksSize
	) {
		return
			sizeof(TileEntry) * tilesSize +
			sizeof(PolyEntry) * polysSize +
			MinPriorityQueue::calcMemSize(tilesSize) +
			DataPoolType::calcMemSize(casheBlocksSize);
	}

	bool clearState(const dtNavMesh* nav)
	{
		uint32_t tilesNum = nav->getMaxTiles();
		if (!tilesNum)
			return false;
		m_tilesNum = tilesNum;
		m_nav = nav;

		clearStateInternal();
		m_cacheScore = 0;
		m_pqTilesUsage.clearState();
		m_pool.clearState();

		return true;
	}

	void clear()
	{
		m_nav = nullptr;
		std::free(m_data);
		m_data = nullptr;
		m_dataSize = 0;
		m_tilesSize = 0;
		m_polysSize = 0;
		m_tilesNum = 0;
		m_jmpTransferChecker = nullptr;
		m_cacheScore = 0;
		m_pqTilesUsage.clear();
		m_pool.clear();
	}

	bool getData(const dtPolyRef polyRef, const AgentCharacteristics* info, TotalJmpTransfers* to)
	{
		unsigned int salt, it, ip;
		m_nav->decodePolyId(ref, salt, it, ip);
		return getData(polyRef, it, ip, info, to);
	}

	bool insertData(const dtPolyRef polyRef, const TotalJmpTransfers* from)
	{
		unsigned int salt, it, ip;
		m_nav->decodePolyId(polyRef, salt, it, ip);
		return insertData(it, ip, from);
	}

	bool getData(
		const dtPolyRef polyRef,
		unsigned int it,
		unsigned int ip,
		const AgentCharacteristics* info,
		TotalJmpTransfers* to
	) {
		const TileEntry* te = &m_tiles[it];
		const PolyEntry* pe = &te->polys[ip];
		if (pe->dataBlockIdx == INVALID_BLOCK_IDX)
		{
			return false;
		}
		IndexType curBlockIdx = pe->dataBlockIdx;

		const TransfersDataEntryType* from = nullptr;
		do {
			from = m_pool.getBlockData(curBlockIdx);
			for (uint32_t i = 0, n = from->getEntriesNum(); i < n; ++i)
			{
				const TransferDataEntry* entryFrom = from->transfers + i;
				assert(
					entryFrom->edgeId >= 0 &&
					entryFrom->edgeId <= TotalJmpTransfers::CLIMB_OVERLAPPED_INDEX
				);
				StdJmpArr* jmpArr = to->get(entryFrom->edgeId);
				if (!m_jmpTransferChecker(info, entryFrom)) {
					continue;
				}
				JumpingRet* entryTo = jmpArr->allocEntry();
				entryTo->setData(
					entryFrom->transferType,
					polyRef,
					entryFrom->polyRefIdTo,
					entryFrom->pointFrom,
					entryFrom->pointTo
				);
			}
			curBlockIdx = from->getNextBlockIdx();
		} while (from->hasNextBlock());

		return true;
	}

	bool insertData(unsigned int it, unsigned int ip, const TotalJmpTransfers* from)
	{
		if (!from->hasTransfers())
			return true;

		TileEntry* te = &m_tiles[it];
		PolyEntry* pe = &te->polys[ip];
		assert(pe->numBlocks == 0);
		assert(pe->dataBlockIdx == INVALID_BLOCK_IDX);
		uint32_t numTranfers = from->getTotalNumTransfers();
		uint32_t numBlocks = numTranfers / TransfersDataEntryType::SIZE +
				 (uint32_t)(1 && (numTranfers % TransfersDataEntryType::SIZE));
		assert(numBlocks == std::ceil(numTranfers / TransfersDataEntryType::SIZE));

		const uint32_t numFreeBlocks = m_pool.getNumberFreeBlocks();
		if (numBlocks > numFreeBlocks)
		{
			if (!freeBlocksFromTile(numBlocks)) {
				// insufficient memory
				return false;
			}
		}
		
		IndexType toBlockIdx = m_pool.allocBlock();
		TransfersDataEntryType* toBlock = m_pool.getBlockData(toBlockIdx);
		toBlock->clear();
		uint16_t numAllocedBlocks = 1;
		bool copied = false;
		for (int i = 0; i < TotalJmpTransfers::SIZE; ++i)
		{
			// this check will be in lower cycle 'fromArr->getNum()'
			//if (!from->hasTransfers(i))
			//	continue;
			const StdJmpArr* fromArr = from->get(i);
			for (int j = 0, n = fromArr->getNum(); j < n; ++j)
			{
				const JumpingRet* fromEntry = fromArr->getEntry(j);
				assert(fromEntry->transferType > NavmeshPolyTransferFlags::WALKING);
				TransferDataEntry* toEntry = toBlock->allocEntry();
				if (!toEntry)
				{
					IndexType idx = m_pool.allocBlock();
					++numAllocedBlocks;
					toBlock = m_pool.getBlockData(idx);
					toBlock->clear();
					toBlock->setNextBlockIdx(toBlockIdx);
					toBlockIdx = idx;
					toEntry = toBlock->allocEntry();
				}
				toEntry->setData(
					(uint8_t)i,
					fromEntry->transferType,
					fromEntry->polyRefTo,
					fromEntry->posFrom,
					fromEntry->posTo
				);
				copied = true;
			}
		}

		if (copied) {
			pe->dataBlockIdx = toBlockIdx;
			pe->numBlocks = numAllocedBlocks;
			toBlock->setPolyArrIdx(ip);
			toBlock->setBlocksBunchIdx(te->blocksBunchIdx);
			te->blocksBunchIdx = toBlockIdx;
		}
		else {
			m_pool.freeBlock(toBlockIdx);
		}

		return true;
	}

	bool freeBlocksFromTile(const uint32_t numBlocks)
	{
		static const uint32_t MIN_TILES_IN_QUEUE = 11;
		TileEntry* te = m_pqTilesUsage.getMin();
		uint32_t numFreedBlocks = 0;

		while (numFreedBlocks < numBlocks)
		{
			if (m_pqTilesUsage.getNum() < MIN_TILES_IN_QUEUE)
				break;

			IndexType idx = te->blocksBunchIdx;
			assert(idx != INVALID_BLOCK_IDX);
			TransfersDataEntryType* block = m_pool.getBlockData(idx);
			uint32_t polyArrIdx = block->getPolyArrIdx();
			assert(polyArrIdx != INVALID_BLOCK_IDX);
			PolyEntry* pe = &te->polys[polyArrIdx];
			numFreedBlocks += pe->numBlocks;
			pe->init();
			te->blocksBunchIdx = block->getBlocksBunchIdx();
			if (te->blocksBunchIdx == INVALID_BLOCK_IDX)
			{
				m_pqTilesUsage.removeMin();
				te = m_pqTilesUsage.getMin();
			}

#ifndef NDEBUG
			uint32_t cntFreedBlocks = 0;
#endif
			while(true) {
				m_pool.freeBlock(idx);
#ifndef NDEBUG
				++cntFreedBlocks;
#endif
				if (!block->hasNextBlock())
					break;
				idx = block->getNextBlockIdx();
				block = m_pool.getBlockData(idx);
			}
			assert(cntFreedBlocks == numFreedBlocks);
		}

		return numFreedBlocks >= numBlocks;
	}

	ScoreType getTileScore(const IndexType idx) const { return m_tiles[idx].heapScore; }

	bool isTileActive(const IndexType idx) const { return m_tiles[idx].heapIdx != INVALID_HEAP_IDX; }

	void setTileScore(const IndexType idx, const ScoreType val)
	{
		TileEntry* te = &m_tiles[idx];
		assert(te->heapIdx != INVALID_HEAP_IDX);
		if (te->heapScore == val)
			return;
		m_pqTilesUsage.setScoreByIndex(te->heapIdx, val);
	}

	bool insertTile(const IndexType idx, const ScoreType val)
	{
		TileEntry* te = &m_tiles[idx];
		assert(te->heapIdx == INVALID_HEAP_IDX);
		te->heapScore = val;
		return m_pqTilesUsage.append(te);
	}

	void incrementScore() const { ++m_cacheScore; }
	ScoreType getScore() const { return m_cacheScore; }
	void setScore(const ScoreType val) const { m_cacheScore = val; }
	uint32_t getTilesSize() const { return m_tilesSize; }
	uint32_t getPolysSize() const { return m_polysSize; }

private:
	void* clearStateInternal()
	{
		m_tiles = (TileEntry*)m_data;
		for (uint32_t i = 0; i < m_tilesNum; ++i)
		{
			m_tiles[i].init();
		}
		PolyEntry* polyData = (PolyEntry*)((uint8_t*)m_data + sizeof(TileEntry) * m_tilesNum);
		for (uint32_t i = 0; i < m_tilesNum; ++i)
		{
			const dtMeshTile* tile = m_nav->getTile(i);
			if (!tile->header)
				continue;
			TileEntry* te = &m_tiles[i];
			te->polysSize = tile->header->polyCount;
			if (!te->polysSize)
				continue;
			te->polys = polyData;
			for (uint32_t j = 0; j < te->polysSize; ++j)
			{
				te->polys[j].init();
			}
			polyData += te->polysSize;
		}
		return polyData;
	}

private:
	const dtNavMesh* m_nav = {};
	JmpTransferCheckerType* m_jmpTransferChecker = {};
	uint32_t m_dataSize = {};
	void* m_data = {};
	uint32_t m_tilesSize = {};
	uint32_t m_polysSize = {};
	uint32_t m_tilesNum = {};
	TileEntry* m_tiles = {};
	mutable ScoreType m_cacheScore = {};
	MinPriorityQueue m_pqTilesUsage;
	DataPoolType m_pool;
};

using StdJmpTransferCache = JmpTransferCache<>;

class JmpCollider: private common::NonCopyable
{
public:
	JmpCollider() = default;
	~JmpCollider() = default;

	bool init(const mesh::Grid2dBvh* space) { return 1; }
	void clear() {}
	const mesh::Grid2dBvh* getSpace() const;

	bool detectJmpFwdCollision(
		const float* src,
		const float* dst,
		const AgentCharacteristics* agentInfo
	) const;
	bool detectClimbCollision(
		const float* src,
		const float* dst,
		const AgentCharacteristics* agentInfo
	) const;
	bool detectJmpDownCollision(
		const float* src,
		const float* dst,
		const AgentCharacteristics* agentInfo
	) const;

private:
	const mesh::Grid2dBvh* m_collider = {};
};

struct PolyToPolyTransfer
{
	uint32_t transferTypeToNextPoly;
	dtPolyRef polyRefFrom;
	float posFrom[3];
	float posTo[3];
};

class CalcedPathEntry: private common::NonCopyable
{
public:
	~CalcedPathEntry() = default;

	template <typename ... Args>
	static std::shared_ptr<CalcedPathEntry> makeEntry(Args&&... args)
	{
		return std::shared_ptr<CalcedPathEntry>(new CalcedPathEntry(std::forward<Args>(args)...));
	}

	uint32_t getTransferType() const { return m_data.transferTypeToNextPoly; }
	const float* getPointFrom() const { return m_data.posFrom; }
	const float* getPointTo() const { return m_data.posTo; }
	dtPolyRef getPolyFrom() const { return m_data.polyRefFrom; }
	const std::shared_ptr<CalcedPathEntry>& getNext() const { return m_next; }
	void setNext(const std::shared_ptr<CalcedPathEntry>& val) { m_next = val; }

private:
	CalcedPathEntry(
		const uint32_t transferType,
		const float* pointFrom,
		const float* pointTo,
		dtPolyRef polyFrom
	) {
		m_data.transferTypeToNextPoly = transferType;
		geometry::vcopy(m_data.posFrom, pointFrom);
		if (pointTo)
			geometry::vcopy(m_data.posTo, pointTo);
		m_data.polyRefFrom = polyFrom;
	}

	CalcedPathEntry(
		const uint32_t transferType,
		const float* pointFrom,
		const float* pointTo,
		dtPolyRef polyFrom,
		const std::shared_ptr<CalcedPathEntry>& next
	): CalcedPathEntry(transferType, pointFrom, pointTo, polyFrom)
	{
		m_next = next;
	}

private:
	PolyToPolyTransfer m_data;
	std::shared_ptr<CalcedPathEntry> m_next;
	//uint32_t n_nextEntryIdx;
};

class dtJmpNavMeshQuery: private common::NonCopyable
{
public:
	enum: uint32_t
	{
		CALC_PATH_OK,
		CALC_PATH_ERROR_FIND_START_POLY,
		CALC_PATH_ERROR_FIND_END_POLY,
		CALC_PATH_ERROR_FIND_POLY_CORRIDOR,
		CALC_PATH_ERROR_FIND_STRAIGHT_PATH
	};

	static const int OBP_NUM_DIRS = MAX_PLANES_PER_BOUNDING_POLYHEDRON;
	static const int OBP_NUM_VERTS = (OBP_NUM_DIRS - 1) * 2;

private:
	static const int POLY_ARR_SIZE_FOR_EXTRACTION = 512;
	static const int MIN_RAW_PATH_SIZE = 1024;
	static const int MAX_NODES_DECREASE_COEFF = 4;
	static const int MINIMUM_MAX_NODES_SIZE = 16;

	struct JumpTransfersCommonData: private common::NonCopyable
	{
		void init(
			const dtPolyRef startPolyRef_,
			const dtMeshTile* startTile_,
			const dtPoly* startPoly_
		);
		
		dtPolyRef startPolyRef;
		const dtMeshTile* startTile;
		const dtPoly* startPoly;
		float startPolyCenter[3];
		float startPolyVerts[3 * (DT_VERTS_PER_POLYGON + 1)];
		float startPolyAverageVerts[3 * (DT_VERTS_PER_POLYGON + 1)];
		uint32_t neighboursNum;
		dtPolyRef neighbours[DT_VERTS_PER_POLYGON + 1];
		std::pair<dtPolyRef, unsigned char> edges[DT_VERTS_PER_POLYGON];
		TotalJmpTransfers transfersData;
	};

	struct FinalizationPathData: private common::NonCopyable
	{
		FinalizationPathData() = default;
		~FinalizationPathData();
		bool init(const uint32_t nodesSize);
		void clear();
		
		uint32_t polyPathSize{};
		PolyToPolyTransfer* polyPath{};
		uint32_t rawPolyPathSize{};
		dtPolyRef* rawPolyPath{};

		uint32_t straightPathSize{};
		float* straightPath{};
		uint8_t* straightPathFlags{};
		dtPolyRef* straightPathRefs{};
	};

public:
	dtJmpNavMeshQuery() = default;
	~dtJmpNavMeshQuery();

	bool init(
		const dtNavMesh* nav,
		const mesh::Grid2dBvh* space,
		const uint32_t agentCharsSize,
		const AgentCharacteristics* agentChars,
		const uint32_t tilesMaxSize,
		const uint32_t polysMaxSize,
		const uint32_t casheBlocksSize,
		const uint32_t maxNodes,
		const float polyPickWidth,
		const float polyPickHeight
	);
	void clear();
	bool clearState(const dtNavMesh* nav);

	uint32_t calcPathWithJumps(
		const uint32_t agentIdx, const float* startPos, const float* endPos, const uint32_t flags = 0
	);
	dtStatus findNearestPoly(const float* center, const dtQueryFilter* filter, dtPolyRef* foundRef) const;
	dtStatus queryPolygons(
		const float* center,
		const float* halfExtents,
		const dtQueryFilter* filter,
		dtPolyQuery* query
	) const;
	dtStatus queryPolygonsAabb(
		const float* aabbMin,
		const float* aabbMax,
		const dtQueryFilter* filter,
		dtPolyQuery* query
	) const;

	static bool calcObbDataForJumpingForwardDown(
		const float* edgeV1,
		const float* edgeV2,
		const float* polyCenter,
		const float checkBboxFwdDst,
		const float checkBboxHeight,
		const float shrinkCoeff,
		float* verts,
		float* dirs
	);
	static bool calcObbDataForClimbing(
		const float* edgeV1,
		const float* edgeV2,
		const float* polyCenter,
		const float forwardDistance,
		const float minClimbHeight,
		const float maxClimbHeight,
		float* verts,
		float* dirs
	);
	static void calcObpDataForOverlappedClimbing(
		const dtMeshTile* tile,
		const dtPoly* poly,
		const float minClimbHeight,
		const float maxClimbHeight,
		float* verts,
		float* dirs
	);
	static void calcObpDataForOverlappedClimbing(
		const float* polyVertices,
		const int verticesNum,
		const float* polyNorm,
		const float polyDist,
		const float minClimbHeight,
		const float maxClimbHeight,
		float* verts,
		float* dirs
	);

	const dtNavMesh* getAttachedNavMesh() const;
	void clearLastPath();
	const std::shared_ptr<CalcedPathEntry>& getLastPath() const;
	uint64_t incFindPathWithJumpsCounter() const;
	uint64_t getFindPathWithJumpsCounter() const;
	uint32_t getPoolNodesSize() const;
	uint32_t getTilesSize() const;
	uint32_t getPolysSize() const;

private:
	static bool availableWalkTransfer(const uint8_t from, const uint8_t to);
	static bool availableJmpTransfer(const AgentCharacteristics* info, const TransferDataEntry* entry);

	void fixEndPoint(
		const dtPolyRef endRef, const dtPolyRef lastRef, const float* orig, float* fixed
	) const;
	std::pair<std::shared_ptr<CalcedPathEntry>, std::shared_ptr<CalcedPathEntry>>
		pathEntriesArrToList(const uint32_t straightPathNum, const float* endPos) const;
	// Finds the straight path from the start to the end position within the polygon corridor
	dtStatus findPathWithJumps(
		const uint32_t agentIdx,
		const dtPolyRef startRef,
		const dtPolyRef endRef,
		const float* startPos,
		const float* endPos,
		uint32_t* polyPathNum
	);
	dtStatus findStraightPath(
		const float* startPos,
		const float* endPos,
		const dtPolyRef* polyPath,
		const uint32_t polyPathNum,
		const int options,
		uint32_t* straightPathNum
	) const;
	void queryPolygonsInTile(
		const dtMeshTile* tile,
		const float* qmin,
		const float* qmax,
		const dtQueryFilter* filter,
		dtPolyQuery* query
	) const;

	void findPolysReachableFromCurrent(
		const AgentCharacteristics* agentChars,
		JumpTransfersCommonData& commonData
	);
	void findPolysReachableJumpingAndClimbing(
		uint32_t edgeIdx,
		const AgentCharacteristics* agentChars,
		JumpTransfersCommonData& commonData
	);
	void findPolysReachableJumpingForward(
		const float* edgeV1,
		const float* edgeV2,
		const float* outPerp,
		const AgentCharacteristics* agentChars,
		JumpTransfersCommonData& commonData,
		StdJmpArr* transfers
	);
	void findPolysReachableJumpingDown(
		const float* edgeV1,
		const float* edgeV2,
		const float* outPerp,
		const AgentCharacteristics* agentChars,
		JumpTransfersCommonData& commonData,
		StdJmpArr* transfers
	);
	void findPolysReachableClimbing(
		const float* edgeV1,
		const float* edgeV2,
		const float* outPerp,
		const AgentCharacteristics* agentChars,
		JumpTransfersCommonData& commonData,
		StdJmpArr* transfers
	);
	void findPolysReachableClimbingOverlapped(
		const AgentCharacteristics* agentChars,
		JumpTransfersCommonData& commonData
	);

	bool findJumpingForwardTransfers(
		const AgentCharacteristics* agentChars,
		const JumpTransfersCommonData& commonData,
		const dtPolyRef polyRef,
		StdJmpArr* transfers
	) const;
	bool findJumpingDownTransfers(
		const AgentCharacteristics* agentChars,
		const JumpTransfersCommonData& commonData,
		const dtPolyRef polyRef,
		StdJmpArr* transfers
	) const;
	bool findClimbingTransfers(
		const AgentCharacteristics* agentChars,
		const JumpTransfersCommonData& commonData,
		const dtPolyRef polyRef,
		StdJmpArr* transfers
	) const;
	bool findClimbingOverlappedPolysTransfers(
		const AgentCharacteristics* agentChars,
		const JumpTransfersCommonData& commonData,
		const dtPolyRef polyRef,
		StdJmpArr* transfers
	) const;

	dtStatus getPathToNodeWithJumps(
		const float* endPos,
		const struct dtNodeJmp* endNode,
		uint32_t* polyPathNum
	);
	dtStatus getPortalPoints(
		dtPolyRef from,
		dtPolyRef to,
		float* left,
		float* right,
		unsigned char& fromType,
		unsigned char& toType
	) const;
	dtStatus getPortalPoints(
		dtPolyRef from,
		const dtPoly* fromPoly,
		const dtMeshTile* fromTile,
		dtPolyRef to,
		const dtPoly* toPoly,
		const dtMeshTile* toTile,
		float* left,
		float* right
	) const;
	dtStatus getEdgeMidPoint(dtPolyRef from, dtPolyRef to, float* mid) const;
	dtStatus getEdgeMidPoint(
		dtPolyRef from,
		const dtPoly* fromPoly,
		const dtMeshTile* fromTile,
		dtPolyRef to,
		const dtPoly* toPoly,
		const dtMeshTile* toTile,
		float* mid
	) const;
	dtStatus closestPointOnPolyBoundary(dtPolyRef ref, const float* pos, float* closest) const;
	dtStatus appendVertex(
		const float* pos,
		const unsigned char flags,
		const dtPolyRef ref,
		float* straightPath,
		unsigned char* straightPathFlags,
		dtPolyRef* straightPathRefs,
		uint32_t* straightPathCount,
		const uint32_t maxStraightPath
	) const;
	dtStatus appendPortals(
		const int startIdx,
		const int endIdx,
		const float* endPos,
		const dtPolyRef* path,
		float* straightPath,
		unsigned char* straightPathFlags,
		dtPolyRef* straightPathRefs,
		uint32_t* straightPathCount,
		const uint32_t maxStraightPath,
		const int options
	) const;

private:
	const dtNavMesh* m_nav{};
	mutable uint64_t m_cntFindPathWithJumpsCalls{};
	uint32_t m_agentCharsSize{};
	AgentCharacteristics* m_agentChars{};
	JmpCollider m_collider;
	StdJmpTransferCache m_transfersCashe;
	class dtNodePoolJmp* m_nodePool{};
	class dtNodeQueueJmp* m_openList{};
	FinalizationPathData m_finPathData;
	float m_polyPickExt[3];
	std::shared_ptr<CalcedPathEntry> m_calcedPath;
};
#endif // ZENGINE_NAVMESH

/// Provides the ability to perform pathfinding related queries against
/// a navigation mesh.
/// @ingroup detour
class dtNavMeshQuery
{
public:
	dtNavMeshQuery();
	~dtNavMeshQuery();
	
	/// Initializes the query object.
	///  @param[in]		nav			Pointer to the dtNavMesh object to use for all queries.
	///  @param[in]		maxNodes	Maximum number of search nodes. [Limits: 0 < value <= 65535]
	/// @returns The status flags for the query.
	dtStatus init(const dtNavMesh* nav, const int maxNodes);
	
	/// @name Standard Pathfinding Functions
	// /@{

	/// Finds a path from the start polygon to the end polygon.
	///  @param[in]		startRef	The refrence id of the start polygon.
	///  @param[in]		endRef		The reference id of the end polygon.
	///  @param[in]		startPos	A position within the start polygon. [(x, y, z)]
	///  @param[in]		endPos		A position within the end polygon. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.) 
	///  							[(polyRef) * @p pathCount]
	///  @param[out]	pathCount	The number of polygons returned in the @p path array.
	///  @param[in]		maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 1]
	dtStatus findPath(dtPolyRef startRef, dtPolyRef endRef,
					  const float* startPos, const float* endPos,
					  const dtQueryFilter* filter,
					  dtPolyRef* path, int* pathCount, const int maxPath) const;

	/// Finds the straight path from the start to the end position within the polygon corridor.
	///  @param[in]		startPos			Path start position. [(x, y, z)]
	///  @param[in]		endPos				Path end position. [(x, y, z)]
	///  @param[in]		path				An array of polygon references that represent the path corridor.
	///  @param[in]		pathSize			The number of polygons in the @p path array.
	///  @param[out]	straightPath		Points describing the straight path. [(x, y, z) * @p straightPathCount].
	///  @param[out]	straightPathFlags	Flags describing each point. (See: #dtStraightPathFlags) [opt]
	///  @param[out]	straightPathRefs	The reference id of the polygon that is being entered at each point. [opt]
	///  @param[out]	straightPathCount	The number of points in the straight path.
	///  @param[in]		maxStraightPath		The maximum number of points the straight path arrays can hold.  [Limit: > 0]
	///  @param[in]		options				Query options. (see: #dtStraightPathOptions)
	/// @returns The status flags for the query.
	dtStatus findStraightPath(const float* startPos, const float* endPos,
							  const dtPolyRef* path, const int pathSize,
							  float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
							  int* straightPathCount, const int maxStraightPath, const int options = 0) const;

	///@}
	/// @name Sliced Pathfinding Functions
	/// Common use case:
	///	-# Call initSlicedFindPath() to initialize the sliced path query.
	///	-# Call updateSlicedFindPath() until it returns complete.
	///	-# Call finalizeSlicedFindPath() to get the path.
	///@{ 

	/// Intializes a sliced path query.
	///  @param[in]		startRef	The refrence id of the start polygon.
	///  @param[in]		endRef		The reference id of the end polygon.
	///  @param[in]		startPos	A position within the start polygon. [(x, y, z)]
	///  @param[in]		endPos		A position within the end polygon. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[in]		options		query options (see: #dtFindPathOptions)
	/// @returns The status flags for the query.
	dtStatus initSlicedFindPath(dtPolyRef startRef, dtPolyRef endRef,
								const float* startPos, const float* endPos,
								const dtQueryFilter* filter, const unsigned int options = 0);

	/// Updates an in-progress sliced path query.
	///  @param[in]		maxIter		The maximum number of iterations to perform.
	///  @param[out]	doneIters	The actual number of iterations completed. [opt]
	/// @returns The status flags for the query.
	dtStatus updateSlicedFindPath(const int maxIter, int* doneIters);

	/// Finalizes and returns the results of a sliced path query.
	///  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.) 
	///  							[(polyRef) * @p pathCount]
	///  @param[out]	pathCount	The number of polygons returned in the @p path array.
	///  @param[in]		maxPath		The max number of polygons the path array can hold. [Limit: >= 1]
	/// @returns The status flags for the query.
	dtStatus finalizeSlicedFindPath(dtPolyRef* path, int* pathCount, const int maxPath);
	
	/// Finalizes and returns the results of an incomplete sliced path query, returning the path to the furthest
	/// polygon on the existing path that was visited during the search.
	///  @param[in]		existing		An array of polygon references for the existing path.
	///  @param[in]		existingSize	The number of polygon in the @p existing array.
	///  @param[out]	path			An ordered list of polygon references representing the path. (Start to end.) 
	///  								[(polyRef) * @p pathCount]
	///  @param[out]	pathCount		The number of polygons returned in the @p path array.
	///  @param[in]		maxPath			The max number of polygons the @p path array can hold. [Limit: >= 1]
	/// @returns The status flags for the query.
	dtStatus finalizeSlicedFindPathPartial(const dtPolyRef* existing, const int existingSize,
										   dtPolyRef* path, int* pathCount, const int maxPath);

	///@}
	/// @name Dijkstra Search Functions
	/// @{ 

	/// Finds the polygons along the navigation graph that touch the specified circle.
	///  @param[in]		startRef		The reference id of the polygon where the search starts.
	///  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	///  @param[in]		radius			The radius of the search circle.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	resultRef		The reference ids of the polygons touched by the circle. [opt]
	///  @param[out]	resultParent	The reference ids of the parent polygons for each result. 
	///  								Zero if a result polygon has no parent. [opt]
	///  @param[out]	resultCost		The search cost from @p centerPos to the polygon. [opt]
	///  @param[out]	resultCount		The number of polygons found. [opt]
	///  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	/// @returns The status flags for the query.
	dtStatus findPolysAroundCircle(dtPolyRef startRef, const float* centerPos, const float radius,
								   const dtQueryFilter* filter,
								   dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
								   int* resultCount, const int maxResult) const;
	
	/// Finds the polygons along the naviation graph that touch the specified convex polygon.
	///  @param[in]		startRef		The reference id of the polygon where the search starts.
	///  @param[in]		verts			The vertices describing the convex polygon. (CCW) 
	///  								[(x, y, z) * @p nverts]
	///  @param[in]		nverts			The number of vertices in the polygon.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	resultRef		The reference ids of the polygons touched by the search polygon. [opt]
	///  @param[out]	resultParent	The reference ids of the parent polygons for each result. Zero if a 
	///  								result polygon has no parent. [opt]
	///  @param[out]	resultCost		The search cost from the centroid point to the polygon. [opt]
	///  @param[out]	resultCount		The number of polygons found.
	///  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	/// @returns The status flags for the query.
	dtStatus findPolysAroundShape(dtPolyRef startRef, const float* verts, const int nverts,
								  const dtQueryFilter* filter,
								  dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
								  int* resultCount, const int maxResult) const;
	
	/// Gets a path from the explored nodes in the previous search.
	///  @param[in]		endRef		The reference id of the end polygon.
	///  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.)
	///  							[(polyRef) * @p pathCount]
	///  @param[out]	pathCount	The number of polygons returned in the @p path array.
	///  @param[in]		maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 0]
	///  @returns		The status flags. Returns DT_FAILURE | DT_INVALID_PARAM if any parameter is wrong, or if
	///  				@p endRef was not explored in the previous search. Returns DT_SUCCESS | DT_BUFFER_TOO_SMALL
	///  				if @p path cannot contain the entire path. In this case it is filled to capacity with a partial path.
	///  				Otherwise returns DT_SUCCESS.
	///  @remarks		The result of this function depends on the state of the query object. For that reason it should only
	///  				be used immediately after one of the two Dijkstra searches, findPolysAroundCircle or findPolysAroundShape.
	dtStatus getPathFromDijkstraSearch(dtPolyRef endRef, dtPolyRef* path, int* pathCount, int maxPath) const;

	/// @}
	/// @name Local Query Functions
	///@{

	/// Finds the polygon nearest to the specified center point.
	/// [opt] means the specified parameter can be a null pointer, in that case the output parameter will not be set.
	///
	///  @param[in]		center		The center of the search box. [(x, y, z)]
	///  @param[in]		halfExtents	The search distance along each axis. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	nearestRef	The reference id of the nearest polygon. Will be set to 0 if no polygon is found.
	///  @param[out]	nearestPt	The nearest point on the polygon. Unchanged if no polygon is found. [opt] [(x, y, z)]
	/// @returns The status flags for the query.
	dtStatus findNearestPoly(const float* center, const float* halfExtents,
							 const dtQueryFilter* filter,
							 dtPolyRef* nearestRef, float* nearestPt) const;

	/// Finds the polygon nearest to the specified center point.
	/// [opt] means the specified parameter can be a null pointer, in that case the output parameter will not be set.
	/// 
	///  @param[in]		center		The center of the search box. [(x, y, z)]
	///  @param[in]		halfExtents	The search distance along each axis. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	nearestRef	The reference id of the nearest polygon. Will be set to 0 if no polygon is found.
	///  @param[out]	nearestPt	The nearest point on the polygon. Unchanged if no polygon is found. [opt] [(x, y, z)]
	///  @param[out]	isOverPoly 	Set to true if the point's X/Z coordinate lies inside the polygon, false otherwise. Unchanged if no polygon is found. [opt]
	/// @returns The status flags for the query.
	dtStatus findNearestPoly(const float* center, const float* halfExtents,
							 const dtQueryFilter* filter,
							 dtPolyRef* nearestRef, float* nearestPt, bool* isOverPoly) const;

	/// Finds polygons that overlap the search box.
	///  @param[in]		center		The center of the search box. [(x, y, z)]
	///  @param[in]		halfExtents		The search distance along each axis. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	polys		The reference ids of the polygons that overlap the query box.
	///  @param[out]	polyCount	The number of polygons in the search result.
	///  @param[in]		maxPolys	The maximum number of polygons the search result can hold.
	/// @returns The status flags for the query.
	dtStatus queryPolygons(const float* center, const float* halfExtents,
						   const dtQueryFilter* filter,
						   dtPolyRef* polys, int* polyCount, const int maxPolys) const;

	/// Finds polygons that overlap the search box.
	///  @param[in]		center		The center of the search box. [(x, y, z)]
	///  @param[in]		halfExtents		The search distance along each axis. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[in]		query		The query. Polygons found will be batched together and passed to this query.
	dtStatus queryPolygons(const float* center, const float* halfExtents,
						   const dtQueryFilter* filter, dtPolyQuery* query) const;

	/// Finds the non-overlapping navigation polygons in the local neighbourhood around the center position.
	///  @param[in]		startRef		The reference id of the polygon where the search starts.
	///  @param[in]		centerPos		The center of the query circle. [(x, y, z)]
	///  @param[in]		radius			The radius of the query circle.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	resultRef		The reference ids of the polygons touched by the circle.
	///  @param[out]	resultParent	The reference ids of the parent polygons for each result. 
	///  								Zero if a result polygon has no parent. [opt]
	///  @param[out]	resultCount		The number of polygons found.
	///  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	/// @returns The status flags for the query.
	dtStatus findLocalNeighbourhood(dtPolyRef startRef, const float* centerPos, const float radius,
									const dtQueryFilter* filter,
									dtPolyRef* resultRef, dtPolyRef* resultParent,
									int* resultCount, const int maxResult) const;

	/// Moves from the start to the end position constrained to the navigation mesh.
	///  @param[in]		startRef		The reference id of the start polygon.
	///  @param[in]		startPos		A position of the mover within the start polygon. [(x, y, x)]
	///  @param[in]		endPos			The desired end position of the mover. [(x, y, z)]
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	resultPos		The result position of the mover. [(x, y, z)]
	///  @param[out]	visited			The reference ids of the polygons visited during the move.
	///  @param[out]	visitedCount	The number of polygons visited during the move.
	///  @param[in]		maxVisitedSize	The maximum number of polygons the @p visited array can hold.
	/// @returns The status flags for the query.
	dtStatus moveAlongSurface(dtPolyRef startRef, const float* startPos, const float* endPos,
							  const dtQueryFilter* filter,
							  float* resultPos, dtPolyRef* visited, int* visitedCount, const int maxVisitedSize) const;
	
	/// Casts a 'walkability' ray along the surface of the navigation mesh from 
	/// the start position toward the end position.
	/// @note A wrapper around raycast(..., RaycastHit*). Retained for backward compatibility.
	///  @param[in]		startRef	The reference id of the start polygon.
	///  @param[in]		startPos	A position within the start polygon representing 
	///  							the start of the ray. [(x, y, z)]
	///  @param[in]		endPos		The position to cast the ray toward. [(x, y, z)]
	///  @param[out]	t			The hit parameter. (FLT_MAX if no wall hit.)
	///  @param[out]	hitNormal	The normal of the nearest wall hit. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[out]	path		The reference ids of the visited polygons. [opt]
	///  @param[out]	pathCount	The number of visited polygons. [opt]
	///  @param[in]		maxPath		The maximum number of polygons the @p path array can hold.
	/// @returns The status flags for the query.
	dtStatus raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
					 const dtQueryFilter* filter,
					 float* t, float* hitNormal, dtPolyRef* path, int* pathCount, const int maxPath) const;
	
	/// Casts a 'walkability' ray along the surface of the navigation mesh from 
	/// the start position toward the end position.
	///  @param[in]		startRef	The reference id of the start polygon.
	///  @param[in]		startPos	A position within the start polygon representing 
	///  							the start of the ray. [(x, y, z)]
	///  @param[in]		endPos		The position to cast the ray toward. [(x, y, z)]
	///  @param[in]		filter		The polygon filter to apply to the query.
	///  @param[in]		flags		govern how the raycast behaves. See dtRaycastOptions
	///  @param[out]	hit			Pointer to a raycast hit structure which will be filled by the results.
	///  @param[in]		prevRef		parent of start ref. Used during for cost calculation [opt]
	/// @returns The status flags for the query.
	dtStatus raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
					 const dtQueryFilter* filter, const unsigned int options,
					 dtRaycastHit* hit, dtPolyRef prevRef = 0) const;


	/// Finds the distance from the specified position to the nearest polygon wall.
	///  @param[in]		startRef		The reference id of the polygon containing @p centerPos.
	///  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	///  @param[in]		maxRadius		The radius of the search circle.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	hitDist			The distance to the nearest wall from @p centerPos.
	///  @param[out]	hitPos			The nearest position on the wall that was hit. [(x, y, z)]
	///  @param[out]	hitNormal		The normalized ray formed from the wall point to the 
	///  								source point. [(x, y, z)]
	/// @returns The status flags for the query.
	dtStatus findDistanceToWall(dtPolyRef startRef, const float* centerPos, const float maxRadius,
								const dtQueryFilter* filter,
								float* hitDist, float* hitPos, float* hitNormal) const;
	
	/// Returns the segments for the specified polygon, optionally including portals.
	///  @param[in]		ref				The reference id of the polygon.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[out]	segmentVerts	The segments. [(ax, ay, az, bx, by, bz) * segmentCount]
	///  @param[out]	segmentRefs		The reference ids of each segment's neighbor polygon. 
	///  								Or zero if the segment is a wall. [opt] [(parentRef) * @p segmentCount] 
	///  @param[out]	segmentCount	The number of segments returned.
	///  @param[in]		maxSegments		The maximum number of segments the result arrays can hold.
	/// @returns The status flags for the query.
	dtStatus getPolyWallSegments(dtPolyRef ref, const dtQueryFilter* filter,
								 float* segmentVerts, dtPolyRef* segmentRefs, int* segmentCount,
								 const int maxSegments) const;

	/// Returns random location on navmesh.
	/// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[in]		frand			Function returning a random number [0..1).
	///  @param[out]	randomRef		The reference id of the random location.
	///  @param[out]	randomPt		The random location. 
	/// @returns The status flags for the query.
	dtStatus findRandomPoint(const dtQueryFilter* filter, float (*frand)(),
							 dtPolyRef* randomRef, float* randomPt) const;

	/// Returns random location on navmesh within the reach of specified location.
	/// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
	/// The location is not exactly constrained by the circle, but it limits the visited polygons.
	///  @param[in]		startRef		The reference id of the polygon where the search starts.
	///  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	///  @param[in]		filter			The polygon filter to apply to the query.
	///  @param[in]		frand			Function returning a random number [0..1).
	///  @param[out]	randomRef		The reference id of the random location.
	///  @param[out]	randomPt		The random location. [(x, y, z)]
	/// @returns The status flags for the query.
	dtStatus findRandomPointAroundCircle(dtPolyRef startRef, const float* centerPos, const float maxRadius,
										 const dtQueryFilter* filter, float (*frand)(),
										 dtPolyRef* randomRef, float* randomPt) const;
	
	/// Finds the closest point on the specified polygon.
	///  @param[in]		ref			The reference id of the polygon.
	///  @param[in]		pos			The position to check. [(x, y, z)]
	///  @param[out]	closest		The closest point on the polygon. [(x, y, z)]
	///  @param[out]	posOverPoly	True of the position is over the polygon.
	/// @returns The status flags for the query.
	dtStatus closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const;
	
	/// Returns a point on the boundary closest to the source point if the source point is outside the 
	/// polygon's xz-bounds.
	///  @param[in]		ref			The reference id to the polygon.
	///  @param[in]		pos			The position to check. [(x, y, z)]
	///  @param[out]	closest		The closest point. [(x, y, z)]
	/// @returns The status flags for the query.
	dtStatus closestPointOnPolyBoundary(dtPolyRef ref, const float* pos, float* closest) const;
	
	/// Gets the height of the polygon at the provided position using the height detail. (Most accurate.)
	///  @param[in]		ref			The reference id of the polygon.
	///  @param[in]		pos			A position within the xz-bounds of the polygon. [(x, y, z)]
	///  @param[out]	height		The height at the surface of the polygon.
	/// @returns The status flags for the query.
	dtStatus getPolyHeight(dtPolyRef ref, const float* pos, float* height) const;

	/// @}
	/// @name Miscellaneous Functions
	/// @{

	/// Returns true if the polygon reference is valid and passes the filter restrictions.
	///  @param[in]		ref			The polygon reference to check.
	///  @param[in]		filter		The filter to apply.
	bool isValidPolyRef(dtPolyRef ref, const dtQueryFilter* filter) const;

	/// Returns true if the polygon reference is in the closed list. 
	///  @param[in]		ref		The reference id of the polygon to check.
	/// @returns True if the polygon is in closed list.
	bool isInClosedList(dtPolyRef ref) const;
	
	/// Gets the node pool.
	/// @returns The node pool.
	class dtNodePool* getNodePool() const { return m_nodePool; }
	
	/// Gets the navigation mesh the query object is using.
	/// @return The navigation mesh the query object is using.
	const dtNavMesh* getAttachedNavMesh() const { return m_nav; }

	/// @}
	
private:
	// Explicitly disabled copy constructor and copy assignment operator
	dtNavMeshQuery(const dtNavMeshQuery&);
	dtNavMeshQuery& operator=(const dtNavMeshQuery&);
	
	/// Queries polygons within a tile.
	void queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
							 const dtQueryFilter* filter, dtPolyQuery* query) const;

	/// Returns portal points between two polygons.
	dtStatus getPortalPoints(dtPolyRef from, dtPolyRef to, float* left, float* right,
							 unsigned char& fromType, unsigned char& toType) const;
	dtStatus getPortalPoints(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
							 dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
							 float* left, float* right) const;
	
	/// Returns edge mid point between two polygons.
	dtStatus getEdgeMidPoint(dtPolyRef from, dtPolyRef to, float* mid) const;
	dtStatus getEdgeMidPoint(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
							 dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
							 float* mid) const;
	
	// Appends vertex to a straight path
	dtStatus appendVertex(const float* pos, const unsigned char flags, const dtPolyRef ref,
						  float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
						  int* straightPathCount, const int maxStraightPath) const;

	// Appends intermediate portal points to a straight path.
	dtStatus appendPortals(const int startIdx, const int endIdx, const float* endPos, const dtPolyRef* path,
						   float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
						   int* straightPathCount, const int maxStraightPath, const int options) const;

	// Gets the path leading to the specified end node.
	dtStatus getPathToNode(struct dtNode* endNode, dtPolyRef* path, int* pathCount, int maxPath) const;
	
private:
	const dtNavMesh* m_nav;				///< Pointer to navmesh data.

	struct dtQueryData
	{
		dtStatus status;
		struct dtNode* lastBestNode;
		float lastBestNodeCost;
		dtPolyRef startRef, endRef;
		float startPos[3], endPos[3];
		const dtQueryFilter* filter;
		unsigned int options;
		float raycastLimitSqr;
	};
	dtQueryData m_query;				///< Sliced query state.

	class dtNodePool* m_tinyNodePool;	///< Pointer to small node pool.
	class dtNodePool* m_nodePool;		///< Pointer to node pool.
	class dtNodeQueue* m_openList;		///< Pointer to open list queue.
};

/// Allocates a query object using the Detour allocator.
/// @return An allocated query object, or null on failure.
/// @ingroup detour
dtNavMeshQuery* dtAllocNavMeshQuery();

/// Frees the specified query object using the Detour allocator.
///  @param[in]		query		A query object allocated using #dtAllocNavMeshQuery
/// @ingroup detour
void dtFreeNavMeshQuery(dtNavMeshQuery* query);

#endif // DETOURNAVMESHQUERY_H