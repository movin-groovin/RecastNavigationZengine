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

#ifndef INPUTGEOM_H
#define INPUTGEOM_H

#include "MeshLoaderObj.h"
#include "Recast.h"
#include "Common.h"

#include <vector>
#include <utility>
#include <limits>
#include <memory>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <cassert>
#include <unordered_map>
#include <type_traits>

#include <immintrin.h>

#define PRINT_STRUCTURE_STAT
//#define PRINT_TRI_VS_SEG_LATENCY_TOTAL 1
#define PRINT_AFTER_N_POLYS 1000
//#define PRINT_TRI_VS_SEG_LATENCY 1
//#define PRINT_TRI_VS_OBB_LATENCY 1

struct BuildSettings
{
	// Cell size in world units
	float cellSize;
	// Cell height in world units
	float cellHeight;
	// Agent height in world units
	float agentHeight;
	// Agent radius in world units
	float agentRadius;
	// Agent max climb in world units
	float agentMaxClimb;
	// Agent max slope in degrees
	float agentMaxSlope;
	// Region minimum size in voxels.
	// regionMinSize = sqrt(regionMinArea)
	float regionMinSize;
	// Region merge size in voxels.
	// regionMergeSize = sqrt(regionMergeArea)
	float regionMergeSize;
	// Edge max length in world units
	float edgeMaxLen;
	// Edge max error in voxels
	float edgeMaxError;
	float vertsPerPoly;
	// Detail sample distance in voxels
	float detailSampleDist;
	// Detail sample max error in voxel heights.
	float detailSampleMaxError;
	// Partition type, see SamplePartitionType
	int partitionType;
	// Bounds of the area to mesh
	float navMeshBMin[3];
	float navMeshBMax[3];
	// Size of the tiles in voxels
	float tileSize;
};

struct Aabb3D
{
    float min[3];
    float max[3];
    int polyIndex;
};

struct OBB
{
    float dir[9]; // 3 vectors
    float center[3];
    float halfWidth[3];
};

struct OBBExt
{
    float verts[3 * 8];
    OBB b;
};

class Octree
{
private:
    struct Node
    {
        float bmin[3];
        float bmax[3];
        Node* childs[8];
        int m_num = 0;
        int* m_trianglePolys = nullptr;

        Node() {std::memset(childs, 0, sizeof(childs));}
        ~Node() {
            delete [] m_trianglePolys;
            for (int i = 0; i < 8; ++i) {
                if (!childs[i]) break;
                delete childs[i];
                childs[i] = nullptr;
            }
        }
        bool isLeaf() const {return m_trianglePolys;}
    };

public:
    Octree(int maxDepth = 20, int minTrisInLeaf = 101);
    ~Octree();

    bool Load(rcMeshLoaderObjExt* mesh);
    bool isMemInsufficient() const {return m_memInsufficient;}
    bool detectSegmentPolyCollision(
        const float* start, const float* end, float& t, int n
    ) const;
    void printStat() const
    {
        printf("Delta x: %f, y: %f, z: %f\n", m_bmax[0] - m_bmin[0],
               m_bmax[1] - m_bmin[1], m_bmax[2] - m_bmin[2]);
        printf("Polys num: %d, leafs num: %d, average polys in leaf: %f\n"
               "max polys in leaf: %d, min polys in leaf: %d, current max depth: "
               "%d\n", m_polyNum, m_leafNum, m_averPolysInLeaf,
               m_maxPolysInLeaf, m_minPolysInLeaf, m_curMaxDepthReached
            );
    }
    static bool checkOverlapAabb(
        const float* start, const float* end, const float* bmin, const float* bmax
    );

private:
    Node* LoadDo(
        int depth, const float* center, const float* span, const float* verts,
        const int* tris, const Aabb3D* bboxes, int polysNum
    );
    bool detectSegmentPolyCollisionDo(
        const float* start, const float* end, const Node* cur, float& t
    ) const;
    static void calcAabb(const float* verts, const int* triangle, Aabb3D* bbox);

private:
    const int m_maxDepth;
    const int m_minTrisInLeaf;
    float m_bmin[3];
    float m_bmax[3];
    Node m_root;
	rcMeshLoaderObjExt* m_mesh = nullptr;
    std::vector<std::vector<int>> m_constrTimeIds;
    bool m_memInsufficient = false;
    int m_polyNum = 0;
    int m_leafNum = 0;
    float m_averPolysInLeaf = 0.f;
    int m_maxPolysInLeaf = 0;
    int m_minPolysInLeaf = std::numeric_limits<int>::max();
    int m_curMaxDepthReached = 0;
    const int* m_triIds = nullptr;
    const float* m_verts = nullptr;
    // detection cast (metrics)
    mutable int m_totalNodes;
    mutable int m_leafNodes;
    mutable int m_polys;
};

class BminBmaxSegmentTree;

template <typename T>
struct ArrayBuffer
{
    ArrayBuffer() = default;
    ~ArrayBuffer() { cleanup(); }
    void cleanup() {
        delete [] data;
        data = nullptr;
        size = 0;
        num = 0;
    }

    int size = 0;
    int num = 0;
    T* data = nullptr;
};

class alignas(__m128) Grid2dBvh
{
public:
	enum {
		SUCCESSFUL,
		ERROR_NO_MEMORY,
		ERROR_LOGIC_ERROR,
		ERROR_PARSING_ERROR
	};

	struct TrianglesData
    {
		TrianglesData() = default;
		TrianglesData(const TrianglesData&) = delete;
		const TrianglesData operator=(const TrianglesData&) = delete;
		// move operators deleted automatically
		~TrianglesData() {
			delete [] verts;
			verts = nullptr;
			delete [] tris;
			tris = nullptr;
			delete [] triFlags;
			triFlags = nullptr;
            delete [] normals;
            normals = nullptr;
		}

		void clearCurrent() {
			vertsNumCurrent = 0;
			trisNumCurrent = 0;
		}

		int vertsNum = 0;
		float* verts = nullptr;
		int trisNum = 0;
		int* tris = nullptr;
		uint8_t* triFlags = nullptr;
		float* normals = nullptr;
		int vertsNumCurrent = 0;
		int trisNumCurrent = 0;
    };
	struct OffMeshData
	{
        OffMeshData() = default;
        ~OffMeshData() {
            delete [] offMeshVerts; offMeshVerts = nullptr;
            delete [] offMeshRads; offMeshRads = nullptr;
            delete [] offMeshDirs; offMeshDirs = nullptr;
            delete [] offMeshAreas; offMeshAreas = nullptr;
            delete [] offMeshFlags; offMeshFlags = nullptr;
            delete [] offMeshId; offMeshId = nullptr;
        }

        float* offMeshVerts = nullptr;
        float* offMeshRads = nullptr;
        uint8_t* offMeshDirs = nullptr;
        uint8_t* offMeshAreas = nullptr;
        uint16_t* offMeshFlags = nullptr;
        uint32_t* offMeshId = nullptr;
        int offMeshSize = 0;
        int offMeshNum = 0;
	};

private:
	static const int VOBS_NUM_COLLIDE_CHEKING = 8;
    // TODO replace to common header (also from MeshLoaderExt)
    static const int REGULAR_VERTS_BLOCK = 3;
#ifdef USAGE_SSE_1_0
    static const int CUR_VERTS_BLOCK = 4;
#else
    static const int CUR_VERTS_BLOCK = 3;
#endif
    static constexpr float COST_CHECK_BBOX = 0.125f;
    static constexpr float COST_CHECK_TRI = 1.f;
    static constexpr int LIMIT_POLYS_STOP = 16; // 8, 1;

    struct CellBoundingPair
    {
        float min[3];
        float max[3];
    };

	struct XzGridBorders
	{
		int xiMin = -1;
		int xiMax = -1;
		int ziMin = -1;
		int ziMax = -1;
	};

	struct MarkedEntry
	{
		MarkedEntry() = default;
		~MarkedEntry() {
			delete [] gridIds;
			gridIds = nullptr;
		}

		int copy(MarkedEntry& to) {
			if (!data.copy(to.data)) {
				return ERROR_NO_MEMORY;
			}
			to.idsNum = idsNum;
			to.idsSize = idsSize;
            to.gridIds = new(std::nothrow) int[to.idsSize];
			if (!gridIds) {
				return ERROR_NO_MEMORY;
			}
			std::memcpy(to.gridIds, gridIds, sizeof(int) * to.idsNum);
			return SUCCESSFUL;
		}

		rcMeshLoaderObjExt::MarkedEntry data;
		int idsSize = 0;
		int idsNum = 0;
		int* gridIds = nullptr;
	};
    struct alignas(__m128) BvhNode
    {
        float min[3];
        float padding; // for sse
        float max[3];
        int32_t triId = 0;
    };
	static_assert(sizeof(BvhNode) == 32, "Incorrect BvhNode size");
	struct GridCell
    {
		struct VobPosResidence {
			int vobIndex = -1;
			//int posIndex = -1;
		};

		float bmin[3];
        float bmax[3];
        int childsNumber = 0;
        const BvhNode* childs = nullptr;
		int vobResidencesNum = 0;
		VobPosResidence* vobResidence = nullptr;
		int markedNum = 0;
		int markedSize = 0;
		int* markedIndices = nullptr;

        ~GridCell() {
			//delete [] childs;
			freeAlignedArr<BvhNode>(const_cast<BvhNode*>(childs), 0);
			childs = nullptr;
			delete [] vobResidence;
			vobResidence = nullptr;
			delete [] markedIndices;
			markedIndices = nullptr;
        }
    };
	struct MeshEntry
	{
        MeshEntry() = default;
        ~MeshEntry () {
            //delete [] childs;
			freeAlignedArr<BvhNode>(const_cast<BvhNode*>(childs), 0);
            childs = nullptr;
        }

        rcMeshLoaderObjExt::MeshEntry mesh;
		int childsNumber = 0;
		const BvhNode* childs = nullptr;
	};
	using VobEntry = rcMeshLoaderObjExt::VobEntry;

public:
    Grid2dBvh ();
    ~Grid2dBvh ();

    int load(rcContext* ctx, rcMeshLoaderObjExt* mesh, int cellSize);
	bool isLoaded() const;

    int getOverlappingRectCellIds(
		const float* min, const float* max, int* cellIds, int idsSize
	) const;
    int getOverlappingRectMarkedAreaIds(
        const float* min, const float* max, int* markedAreaIds, int idsSize
    ) const;
	const TrianglesData& getEmptyOverlappingRectData() const;
	const TrianglesData& extractOverlappingRectData(int cellId) const;
	void extractOverlappingRectData(int cellId, TrianglesData& customData) const;
	void moverStateUpdate(const char* name, int stateId);

	bool segTriCollisionFirstHit(const float* start, const float* end, float& t) const;
	bool segTriCollisionNearestHit(const float* start, const float* end, float& t) const;
	//bool obbTriCollisionFirstHit(const OBB* b) const;
	bool obbTriCollisionFirstHit(const OBBExt* be) const;
#ifdef PRINT_STRUCTURE_STAT
    void printStat(rcContext* ctx) const;
#endif
#if (PRINT_TRI_VS_SEG_LATENCY || PRINT_TRI_VS_OBB_LATENCY)
	int getNodesPerCall() const;
	int getLeafesPerCall() const;
	int getPolysPerCall() const;
	void clearStatPerCall() const;
#endif

	const TrianglesData& getRenderingData() const;
	const rcMeshLoaderObjExt::MarkedEntry* getMarkedArea(int i) const;
	int getMarkedAreaSize() const;
	int addMarkedArea(
		const float* verts,
		const int nverts,
		const float minh,
		const float maxh,
		int area
	);
	void deleteMarkedArea(int n);
	void totalDeleteMarkedAreas();
	const OffMeshData& getOffMeshData() const;
	int addOffMeshConn(
		const float* spos,
		const float* epos,
		const float rad,
		unsigned char bidir,
		unsigned char area,
		unsigned short flags
	);
	void deleteOffMeshConn(int i);
    void getBounds(float* bMin, float* bMax) const;

private:
	bool segTriCollisionVobFirstHit(int vobId, const float* start, const float* end, float& t) const;
	bool segTriCollisionVobNearestHit(int vobId, const float* start, const float* end, float& t) const;
	bool obbTriCollisionVobFirstHit(int vobId, const OBBExt* be) const;

	void release();
    int loadInternal(rcContext* ctx, rcMeshLoaderObjExt* mesh, int cellSize);
	int constructVobs(rcMeshLoaderObjExt* mesh);
	int constructRenderingData(rcMeshLoaderObjExt* mesh);
	int constructOverlappingRectData(
		std::unique_ptr<std::pair<int, int>[]> trisVertsPerCellStatic
	);
	int constructMarkedAreas(rcMeshLoaderObjExt* mesh);
	int constructMarkedAreas(int mNum, const rcMeshLoaderObjExt::MarkedEntry* marked);
	int constructOffmeshesOnLadders();
	void fillPolyFlags(
		PolyAreaFlags::FlagType* flagsTo,
		const PolyAreaFlags::FlagType* flagsFrom,
		int trisNum
	);
    bool checkTriangleBelongAabb (
        const float* bmin, const float* bmax, const int* triangle
    ) const;
    std::pair<BvhNode*, int> makeBvh(
        const Aabb3D* bboxes, int* boxIds, const int trisNum
    ) const;
    void subdivideMedian(
        const Aabb3D* bboxes, int* boxIds, BvhNode* bnodes, int i, int j, int& curNodeNum
#ifdef PRINT_STRUCTURE_STAT
        ,int depth, int& maxBoxesInGridCell
#endif
    ) const;
    void subdivideSah(
        const Aabb3D* bboxes, int* boxIds, BvhNode* bnodes, int i, int j, int& curNodeNum
#ifdef PRINT_STRUCTURE_STAT
        ,int depth, int& maxBoxesInGridCell
#endif
    ) const;
	XzGridBorders calcXzGridBorders(const float* min, const float* max) const;
	int linkMarkedAreaWithGrid(int vertsNum, const float* verts, int eInd, MarkedEntry& e);

    static void calc3DAabb(const float* verts, const int* triangle, Aabb3D* bbox, int vertsBlock);
	static void copyDataFromBvh (
		TrianglesData& resData,
		int& trisPos,
		int& vertsPos,
		const BvhNode* curNode,
		const BvhNode* endNode,
		const rcMeshLoaderObjExt::Position* pos,
		const float* verts,
		const int* tris,
		const PolyAreaFlags::FlagType* triFlags
	);
    static float calcSah(
        const BminBmaxSegmentTree& tree,
        int i,
        int mid,
        int j,
        float* totalMin,
        float* totalMax
    );
    static float calcHalfSurfaceArea(const float* bboxDiff);
    static float calcPartSahValue(const float* diffTotal, const float* bboxDiff, const int n);

	static void transformVertex(const float* vertex, const float* trafo, float* vertexNew);
	static void transformDirection(const float* normal, const float* trafo, float* normalNew);
	static void transformVertex(
		const float* vertex, const rcMeshLoaderObjExt::Position* pos, float* vertexNew
	);

private:
    int m_cellSize = 0;
    float m_cellSizeInv = 0.f;
#ifdef USAGE_SSE_1_0
    __m128 m_cellSizeInvVec;
#endif
    int m_cellsNum = 0;
    GridCell* m_grid = nullptr;
	int m_trisNum = 0;
	int* m_tris = nullptr;
	PolyAreaFlags::FlagType* m_triFlags = nullptr;
	int m_vertsNum = 0;
	float* m_verts = nullptr;
	int m_vobsNum = 0;
	VobEntry* m_vobs = nullptr;
	int m_vobsMeshesNum = 0;
	MeshEntry* m_vobsMeshes = nullptr;
	LinearHashMultiStrToInt<> m_moverNameToVob;
#ifdef USAGE_SSE_1_0
    __m128 m_worldMinVecXzXz;
#endif
	float m_worldMin[3];
    float m_worldMax[3];
    float m_worldSize[3];
    int m_wszCellsX;
    int m_wszCellsY;
    int m_wszCellsZ;
	mutable TrianglesData m_overlappingRectData;
    ArrayBuffer<MarkedEntry> m_markedAreas;
    OffMeshData m_offMeshConns;
	TrianglesData m_renderingData;
	rcMeshLoaderObjExt* m_mesh = nullptr;
    // statisctics per construction
#ifdef PRINT_STRUCTURE_STAT
    mutable int m_totalNodes = 0;
    mutable int m_leafNodes = 0;
    mutable int m_internalNodes = 0;
    mutable int m_maxDepth = 0;
    mutable int m_curDepth = 0;
    mutable int m_maxTrisInGridCell = 0;
    mutable int m_maxBoxesInGridCell = 0;
    mutable int m_bytesPerConstruction = 0;
    mutable int m_bytesForData = 0;
#endif
    // statistics per call
#if (PRINT_TRI_VS_SEG_LATENCY || PRINT_TRI_VS_OBB_LATENCY)
    mutable int totalNodesTraversed = 0;
    mutable int totalLeafesTraversed = 0;
    mutable int totalPolysTraversed = 0;
#endif
};

class alignas(__m128) InputGeom
{
private:
	static const int STR_SIZE = 1024;
	static const int MAX_VOLUMES = 256;

public:
	InputGeom();
	~InputGeom();

	InputGeom(const InputGeom&) = delete;
	InputGeom& operator=(const InputGeom&) = delete;

    bool loadFromDir(
        class rcContext* ctx, const char* filepath, float offsetSize, float bvhGridSize
    );

	// service calls
	rcContext& getCtx() { return *m_ctx; }
	const Grid2dBvh& getSpace() { return m_space; }
	int getVertCount() const;
	int getTriCount() const;
	const char* getNavMeshName() const;
	const char* getMarkedMeshName() const;
	const char* getBaseMeshName() const;

	// Method to return static mesh data.
	const rcMeshLoaderObjExt* getMeshExt() const { return m_meshExt; }
	const float* getMeshBoundsMin() const { return m_meshBMin; }
	const float* getMeshBoundsMax() const { return m_meshBMax; }
	const float* getNavMeshBoundsMin() const { return m_hasBuildSettings ? m_buildSettings.navMeshBMin : m_meshBMin; }
	const float* getNavMeshBoundsMax() const { return m_hasBuildSettings ? m_buildSettings.navMeshBMax : m_meshBMax; }
	const BuildSettings* getBuildSettings() const { return m_hasBuildSettings ? &m_buildSettings : 0; }

	// collisions
    bool raycastMesh(const float* src, const float* dst, float& tmin, bool nearestHit) const;
    bool obbCollDetect(const OBBExt* be) const;

	// mesh extracting
    int getOverlappingRectCellIds(
		const float* min, const float* max, int* cellIds, int idsSize
	) const;
	const Grid2dBvh::TrianglesData& extractOverlappingRectData(int cellId) const;

	// Off-Mesh connections.
	int getOffMeshConnectionCount() const;
	const float* getOffMeshConnectionVerts() const;
	const float* getOffMeshConnectionRads() const;
	const unsigned char* getOffMeshConnectionDirs() const;
	const unsigned char* getOffMeshConnectionAreas() const;
	const unsigned short* getOffMeshConnectionFlags() const;
	const unsigned int* getOffMeshConnectionId() const;
	void addOffMeshConnection(
		const float* spos, const float* epos, const float rad,
		unsigned char bidir, unsigned char area, unsigned short flags
	);
	void deleteOffMeshConnection(int i);
	void drawOffMeshConnections(struct duDebugDraw* dd, bool hilight = false);

	// Box Volumes.
	int getConvexVolumeCount() const;
	const rcMeshLoaderObjExt::MarkedEntry* getConvexVolume(int i) const;
	void addConvexVolume(
		const float* verts,
		const int nverts,
		const float minh,
		const float maxh,
		unsigned char area
	);
	void deleteConvexVolume(int i);
	void totalDeleteMarkedAreas();
	void drawConvexVolumes(struct duDebugDraw* dd, bool hilight = false);

private:
	bool loadMesh(
		class rcContext* ctx,
		const char* navMeshName,
		const char* staticMesh,
		const char* vobsMesh,
		const char* markedMesh,
		float offsetSize,
		float bvhGridSize
	);

private:
	rcContext* m_ctx;
	//Octree m_oct;
	Grid2dBvh m_space;
	char m_markedMeshName[STR_SIZE];
	char m_baseMeshName[STR_SIZE];
	rcMeshLoaderObjExt* m_meshExt;
	float m_meshBMin[3];
	float m_meshBMax[3];
	BuildSettings m_buildSettings;
	bool m_hasBuildSettings;
};

#endif // INPUTGEOM_H
