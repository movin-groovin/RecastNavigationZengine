#ifndef MESH_LIB_H
#define MESH_LIB_H

#ifdef LOGGING_ENABLED

#define PRINT_STRUCTURE_STAT
//#define PRINT_TOTAL_COLLISION_STAT
#ifdef PRINT_TOTAL_COLLISION_STAT
static const uint64_t PRINT_PER_N_CALLS = 5;
#endif // PRINT_TOTAL_COLLISION_STAT

#endif // LOGGING_ENABLED

#include "Common.h"
#include "Geometry.h"
#include <cstdint>
#include <memory>
#include <limits>

namespace mesh
{

// constants and PODs
struct Constants
{
	static const int REGULAR_VERTS_BLOCK = 3;
	static const int SSE_1_VERTS_BLOCK = 4;
#ifdef USAGE_SSE_1_0
	static const int CUR_VERTS_BLOCK = SSE_1_VERTS_BLOCK;
#else
	static const int CUR_VERTS_BLOCK = REGULAR_VERTS_BLOCK;
#endif
};

struct PolyFlagsCollision
{
	enum
	{
		WATER = 1,
		SOLID = 2,
		LAVA = 4
	};
};

struct FlagType
{
	uint32_t isTriangle : 1;
	uint32_t isVobPos : 1;
	uint32_t isActiveVobPos : 1;
	uint32_t reserved : 3;
	uint32_t vobIdOrCollFlags : 20;
	uint32_t isInhabited : 1;
	uint32_t polyFlags : 5;
};

// structs and classes
struct MarkedArea
{
	MarkedArea() = default;
	~MarkedArea() {
		delete[] verts;
		verts = nullptr;
	}

	bool copy(MarkedArea& to) {
		to.vertsNum = vertsNum;
		to.minh = minh;
		to.maxh = maxh;
		to.area = area;
		to.verts = new(std::nothrow) float[3 * to.vertsNum];
		if (!to.verts) {
			return false;
		}
		std::memcpy(to.verts, verts, sizeof(float) * 3 * to.vertsNum);
		return true;
	}

	int vertsNum = 0;
	float* verts = nullptr;
	float minh = 0;
	float maxh = 0;
	int area = -1;
};

struct VobPosition
{
	float aabbMin[3];
	float aabbMax[3];
	float trafo[4 * 4];
	float invTrafo[4 * 4];
	int aabbTri; // idx of virtual vob's tri in bvh with static mesh
};

struct VobEntry
{
	static const int INIT_POS_CNT = 1;

	VobEntry() :
		vobName(nullptr),
		vobType(-1),
		meshIndex(-1),
		posCnt(0),
		activePosIndex(-1),
		positions(nullptr)
#ifdef RENDERING_ENABLED
		, vertsPosRendering(-1)
#endif
	{}

	~VobEntry() {
		delete[] vobName;
		vobName = nullptr;
		delete[] positions;
		positions = nullptr;
	}

	bool init(int posNum = INIT_POS_CNT) {
		posCnt = posNum;
		positions = new((std::nothrow)) VobPosition[posCnt];
		return positions;
	}

	bool isMover() const {
		return vobType == common::VobType::MOVER_UNIDIRECTION || vobType == common::VobType::MOVER_BIDIRECTION;
	}

	bool isDoor() const {
		return vobType == common::VobType::DOOR;
	}

	bool isLadder() const {
		return vobType == common::VobType::LADDER;
	}

	bool hasInfluenceToNavmesh() const {
		return isLadder() || isDoor();
	}

	bool allocVobName() {
		vobName = new(std::nothrow) char[common::Constants::NAME_SIZE];
		return vobName;
	}

	bool copy(VobEntry& to) const {
		if (this == &to)
			return false;
		if (!posCnt)
			return false;

		if (vobName) {
			if (!to.allocVobName()) {
				return false;
			}
			std::strncpy(to.vobName, vobName, common::Constants::NAME_SIZE);
		}
		to.vobType = vobType;
		to.meshIndex = meshIndex;
		to.posCnt = posCnt;
		to.activePosIndex = activePosIndex;
		to.positions = new(std::nothrow) VobPosition[to.posCnt];
		if (!to.positions)
			return false;
		for (int i = 0; i < to.posCnt; ++i) {
			std::memcpy(&to.positions[i], &positions[i], sizeof(VobPosition));
		}
#ifdef RENDERING_ENABLED
		to.vertsPosRendering = vertsPosRendering;
#endif
		return true;
	}

public:
	char* vobName;
	int vobType;
	int meshIndex;
	int posCnt;
	int activePosIndex;
	VobPosition* positions;
#ifdef RENDERING_ENABLED
	int vertsPosRendering;
#endif
};

struct VobMeshEntry
{
	VobMeshEntry() = default;
	~VobMeshEntry() {
		delete[] visualName;
		visualName = nullptr;
		delete[] verts;
		verts = nullptr;
		delete[] tris;
		tris = nullptr;
		delete[] normals;
		normals = nullptr;
		delete[] flags;
		flags = nullptr;
	}

	static char* allocVisualNameData() {
		return new(std::nothrow) char[common::Constants::NAME_SIZE];
	}

	bool allocVisualName() {
		visualName = allocVisualNameData();
		return visualName;
	}

	bool isEmpty() const { return !vertCount; }

	char* visualName = nullptr;
	float* verts = nullptr;
	int* tris = nullptr;
	float* normals = nullptr;
	FlagType* flags = nullptr;
	int vertCount = 0;
	int triCount = 0;
};

class MeshLoaderInterface
{
public:
	MeshLoaderInterface() = default;
	virtual ~MeshLoaderInterface() = default;

	virtual const float* getVerts() const = 0;
	virtual const float* getNormals() const = 0;
	virtual const int* getTris() const = 0;
	virtual const FlagType* getFlags() const = 0;
	virtual int getVertCount() const = 0;
	virtual int getTriCount() const = 0;
	virtual int getVertCountStatic() const = 0;
	virtual int getTriCountStatic() const = 0;
	virtual int getMaxVertsPerVob() const = 0;
	virtual int getMaxTrisPerVob() const = 0;
	virtual int getVobsCnt() const = 0;
	virtual int getMoversCnt() const = 0;
	virtual int getVobMeshesCnt() const = 0;
	virtual const VobEntry* getVobs() const = 0;
	virtual const common::HashMultiStrToInt& getNameToVobMeshIndex() const = 0;
	virtual const VobMeshEntry* getVobMeshes() const = 0;
	virtual const MarkedArea* getMarked() const = 0;
	virtual const int getMarkedCount() const = 0;
	virtual bool isLoaded() const = 0;
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

		Node() { std::memset(childs, 0, sizeof(childs)); }
		~Node() {
			delete[] m_trianglePolys;
			for (int i = 0; i < 8; ++i) {
				if (!childs[i]) break;
				delete childs[i];
				childs[i] = nullptr;
			}
		}
		bool isLeaf() const { return m_trianglePolys; }
	};

public:
	Octree(
#ifdef LOGGING_ENABLED
		common::BaseLogger* log,
#endif
		int maxDepth = 20,
		int minTrisInLeaf = 101
	);
	~Octree() = default;

	bool Load(MeshLoaderInterface* mesh);
	bool isMemInsufficient() const { return m_memInsufficient; }
	bool detectSegmentPolyCollision(
		const float* start, const float* end, float& t, int n
	) const;
#ifdef PRINT_STRUCTURE_STAT
	void printStat() const;
#endif
	static bool checkOverlapAabb(
		const float* start, const float* end, const float* bmin, const float* bmax
	);

private:
	Node* LoadDo(
		std::unique_ptr<std::unique_ptr<int[]>[]>& constrTimeIds,
		int depth, const float* center, const float* span, const float* verts,
		const int* tris, const geometry::AabbTri* bboxes, int polysNum
	);
	bool detectSegmentPolyCollisionDo(
		const float* start, const float* end, const Node* cur, float& t
	) const;
	static void calcAabb(const float* verts, const int* triangle, geometry::AabbTri* bbox);

private:
#ifdef LOGGING_ENABLED
	common::BaseLogger* m_log;
#endif
	const int m_maxDepth;
	const int m_minTrisInLeaf;
	float m_bmin[3];
	float m_bmax[3];
	Node m_root;
	bool m_memInsufficient = false;
	int m_vertsNum = 0;
	int m_polyNum = 0;
	int m_leafNum = 0;
	float m_averPolysInLeaf = 0.f;
	int m_maxPolysInLeaf = 0;
	int m_minPolysInLeaf = std::numeric_limits<int>::max();
	int m_curMaxDepthReached = 0;
	std::unique_ptr<int[]> m_tris;
	std::unique_ptr<float[]> m_verts;
	// detection cast (metrics)
	mutable int m_totalNodes;
	mutable int m_leafNodes;
	mutable int m_polys;
};

class alignas(__m128) Grid2dBvh
{
public:
	enum: int {
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
		~TrianglesData() { release(); }

		void release() {
			delete[] verts;
			verts = nullptr;
			delete[] tris;
			tris = nullptr;
			delete[] triFlags;
			triFlags = nullptr;
			delete[] normals;
			normals = nullptr;
			vertsNum = 0;
			trisNum = 0;
			clearCurrent();
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
		~OffMeshData() { release(); }

		void release() {
			delete[] offMeshVerts;
			offMeshVerts = nullptr;
			delete[] offMeshRads;
			offMeshRads = nullptr;
			delete[] offMeshDirs;
			offMeshDirs = nullptr;
			delete[] offMeshAreas;
			offMeshAreas = nullptr;
			delete[] offMeshFlags;
			offMeshFlags = nullptr;
			delete[] offMeshId;
			offMeshId = nullptr;
			offMeshSize = 0;
			offMeshNum = 0;
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
	static const int VNC_CHK = VOBS_NUM_COLLIDE_CHEKING - 1;
	static_assert(
		(VOBS_NUM_COLLIDE_CHEKING | VNC_CHK) == (VOBS_NUM_COLLIDE_CHEKING * 2 - 1),
		"VOBS_NUM_COLLIDE_CHEKING must be power of 2"
	);
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
			delete[] gridIds;
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

		MarkedArea data;
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
	// for attention purposes: if bvh changed, fix this place
	static_assert(sizeof(BvhNode) == 32, "Incorrect BvhNode size");
	struct GridCell
	{
		struct VobPosResidence {
			int vobIndex = -1;
		};

		GridCell() = default;
		~GridCell() { release(); }

		void release() {
			common::freeAlignedArr<BvhNode>(const_cast<BvhNode*>(childs), 0);
			childs = nullptr;
			delete [] vobResidence;
			vobResidence = nullptr;
			delete [] markedIndices;
			markedIndices = nullptr;
			childsNumber = 0;
			vobResidencesNum = 0;
			markedNum = 0;
			markedSize = 0;
		}

		float bmin[3];
		float bmax[3];
		int childsNumber = 0;
		const BvhNode* childs = nullptr;
		int vobResidencesNum = 0;
		VobPosResidence* vobResidence = nullptr;
		int markedNum = 0;
		int markedSize = 0;
		int* markedIndices = nullptr;
	};
	struct BvhVobMeshEntry
	{
		BvhVobMeshEntry() = default;
		~BvhVobMeshEntry() { release(); }
		void release()
		{
			delete [] visualName;
			visualName = nullptr;
			common::freeAlignedArr(verts, 0);
			verts = nullptr;
			delete [] tris;
			tris = nullptr;
			delete [] normals;
			normals = nullptr;
			delete [] flags;
			flags = nullptr;
			common::freeAlignedArr(childs, 0);
			childs = nullptr;
			vertCount = 0;
			triCount = 0;
			childsNumber = 0;
		}

		bool isEmpty() const { return !visualName || !verts; }

		bool copy(const VobMeshEntry& from)
		{
			if (!from.vertCount || !from.triCount || !from.visualName)
				return false;

			char* tmpVisualName = VobMeshEntry::allocVisualNameData();
			float* tmpVerts = common::allocAlignedArr<float>(from.vertCount * Constants::CUR_VERTS_BLOCK, 16);
			int* tmpTris = new(std::nothrow) int[from.triCount * 3];
			float* tmpNormals = nullptr;
			if (from.normals) {
				tmpNormals = new(std::nothrow) float[from.triCount * 3];
			}
			FlagType* tmpFlags = new(std::nothrow) FlagType[from.triCount];
			if (!tmpVisualName || !tmpVerts || !tmpTris || (from.normals && !tmpNormals) || !tmpFlags) {
				return false;
			}
			release();

			visualName = tmpVisualName;
			verts = tmpVerts;
			tris = tmpTris;
			normals = tmpNormals;
			flags = tmpFlags;
			std::strncpy(visualName, from.visualName, common::Constants::NAME_SIZE - 1);
			visualName[common::Constants::NAME_SIZE - 1] = '\0';
			for (int i = 0; i < from.vertCount; ++i) {
				std::memcpy(verts + i * Constants::CUR_VERTS_BLOCK, from.verts + i * 3, 3 * sizeof(float));
#ifdef USAGE_SSE_1_0
				verts[i * Constants::CUR_VERTS_BLOCK + 3] = 0.f;
#endif
			}
			for (int i = 0, n = 3 * from.triCount; i < n; ++i) {
				tris[i] = from.tris[i] * Constants::CUR_VERTS_BLOCK;
			}
			if (normals) {
				std::memcpy(normals, from.normals, from.triCount * 3 * sizeof(float));
			}
			std::memcpy(flags, from.flags, from.triCount * sizeof(FlagType));
			vertCount = from.vertCount;
			triCount = from.triCount;
			return true;
		}

		char* visualName = nullptr;
		float* verts = nullptr;
		int* tris = nullptr;
		float* normals = nullptr;
		FlagType* flags = nullptr;
		int vertCount = 0;
		int triCount = 0;
		BvhNode* childs = nullptr;
		int childsNumber = 0;
	};

public:
	Grid2dBvh();
	~Grid2dBvh();

	void release();
	void init(
#ifdef LOGGING_ENABLED
		common::BaseLogger* log
#endif
	);
	int load(MeshLoaderInterface* mesh, int cellSize);
	bool isLoaded() const;
	bool saveBinaryMesh() const;
	bool loadBinaryMesh() const;

	int getOverlappingRectCellIds(
		const float* min, const float* max, int* cellIds, int idsSize
	) const;
	int getOverlappingRectMarkedAreaIds(
		const float* min, const float* max, int* markedAreaIds, int idsSize
	) const;
	const TrianglesData& getEmptyOverlappingRectData() const;
	const TrianglesData& extractOverlappingRectData(int cellId) const;
	void extractOverlappingRectData(int cellId, TrianglesData& customData) const;
	void moverStateUpdate(const char* name, const int stateId);

	bool segTriCollisionFirstHit(const float* start, const float* end, float& t) const;
	bool segTriCollisionNearestHit(const float* start, const float* end, float& t) const;
	//bool obbTriCollisionFirstHit(const geometry::Obb* obb) const;
	bool obbTriCollisionFirstHit(const geometry::Obb* obb) const;
#ifdef PRINT_STRUCTURE_STAT
	void printStat() const;
#endif
#if (PRINT_TRI_VS_SEG_LATENCY || PRINT_TRI_VS_OBB_LATENCY)
	int getNodesPerCall() const;
	int getLeafesPerCall() const;
	int getPolysPerCall() const;
	void clearStatPerCall() const;
#endif
#ifdef RENDERING_ENABLED
	int getVobsNum() const;
	const geometry::AabbVob* getVobsAabbsData() const;
	const TrianglesData& getRenderingData() const;
#endif
	const MarkedArea* getMarkedArea(int i) const;
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
	void getWorldSize(float* res) const;
	int getCellSize() const;

private:
	bool segTriCollisionVobFirstHit(int vobId, const float* start, const float* end, float& t) const;
	bool segTriCollisionVobNearestHit(int vobId, const float* start, const float* end, float& t) const;
	bool obbTriCollisionVobFirstHit(int vobId, const geometry::Obb* obb) const;

	void clearState();
	int loadInternal(MeshLoaderInterface* mesh, int cellSize);
	int constructVobs(MeshLoaderInterface* mesh);
#ifdef RENDERING_ENABLED
	int constructRenderingData(MeshLoaderInterface* mesh);
	int constructVobsAbbsData(MeshLoaderInterface* mesh);
#endif
	int constructOverlappingRectData(
		std::unique_ptr<std::pair<int, int>[]> trisVertsPerCellStatic
	);
	int constructMarkedAreas(MeshLoaderInterface* mesh);
	int constructMarkedAreas(int mNum, const MarkedArea* marked);
	int constructOffmeshesOnLadders();
	void fillPolyFlags(
		FlagType* flagsTo,
		const FlagType* flagsFrom,
		int trisNum
	);
	bool checkTriangleBelongAabb(
		const float* bmin, const float* bmax, const int* triangle
	) const;
	std::pair<BvhNode*, int> makeBvh(
		const geometry::AabbTri* bboxes, int* boxIds, const int trisNum
	) const;
	void subdivideMedian(
		const geometry::AabbTri* bboxes, int* boxIds, BvhNode* bnodes, int i, int j, int& curNodeNum
#ifdef PRINT_STRUCTURE_STAT
		, int depth, int& maxBoxesInGridCell
#endif
	) const;
	void subdivideSah(
		const geometry::AabbTri* bboxes, int* boxIds, BvhNode* bnodes, int i, int j, int& curNodeNum
#ifdef PRINT_STRUCTURE_STAT
		, int depth, int& maxBoxesInGridCell
#endif
	) const;
	XzGridBorders calcXzGridBorders(const float* min, const float* max) const;
	int linkMarkedAreaWithGrid(int vertsNum, const float* verts, int eInd, MarkedEntry& e);

	static void calcTriAabb(const float* verts, const int* triangle, geometry::AabbTri* bbox, int vertsBlock);
	static float calcSah(
		const geometry::BminBmaxSegmentTree& tree,
		int i,
		int mid,
		int j,
		float* totalMin,
		float* totalMax
	);
	static float calcHalfSurfaceArea(const float* bboxDiff);
	static float calcPartSahValue(const float* diffTotal, const float* bboxDiff, const int n);

	static void copyTriDataFromBvh(
		TrianglesData& resData,
		int& trisPos,
		int& vertsPos,
		const BvhNode* curNode,
		const BvhNode* endNode,
		const float* verts,
		const int* tris,
		const FlagType* triFlags
	);
	static void copyVobTriDataFromBvh(
		TrianglesData& resData,
		int& trisPos,
		int& vertsPos,
		const BvhNode* curNode,
		const BvhNode* endNode,
		const VobPosition* pos,
		const float* verts,
		const int* tris,
		const FlagType* triFlags
	);
	static void copyBottomTriangles(
		TrianglesData& resData, int& trisPos, int& vertsPos, const VobPosition* pos, const uint8_t type
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
	FlagType* m_triFlags = nullptr;
	int m_vertsNum = 0;
	float* m_verts = nullptr;
	int m_vobsNum = 0;
	VobEntry* m_vobs = nullptr;
	int m_vobsMeshesNum = 0;
	BvhVobMeshEntry* m_vobsMeshes = nullptr;
	common::HashMultiStrToInt m_moverNameToVob;
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
	common::ArrayBuffer<MarkedEntry> m_markedAreas;
	OffMeshData m_offMeshConns;
#ifdef RENDERING_ENABLED
	geometry::AabbVob* m_vobsAabbsData = nullptr;
	TrianglesData m_renderingData;
#endif
#ifdef LOGGING_ENABLED
	common::BaseLogger* m_log = nullptr;
#endif
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
#if defined(PRINT_TRI_VS_SEG_LATENCY) || defined(PRINT_TRI_VS_OBB_LATENCY)
	mutable int totalNodesTraversed = 0;
	mutable int totalLeafesTraversed = 0;
	mutable int totalPolysTraversed = 0;
#endif
};

} // namespace mesh

#endif // MESH_LIB_H