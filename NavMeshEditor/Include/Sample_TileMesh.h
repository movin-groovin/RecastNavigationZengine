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

#ifndef RECASTSAMPLETILEMESH_H
#define RECASTSAMPLETILEMESH_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "Recast.h"
#include "InputGeom.h"
#include "Mesh.h"

//#include <unordered_map>
#include <string>
#include <thread>
#include <memory>
#include <atomic>
#include <chrono>
#include <cstdint>

class Sample_TileMesh : public Sample
{
private:
    static const int NAVMESH_QUERY_MAX_NODES = 2048 * 4;
	static const uint32_t NAVMESH_DEFAULT_TILES_SIZE = 16;
	static const uint32_t NAVMESH_DEFAULT_POLYS_SIZE = 32;

	static constexpr float CHECK_BBOX_HEIGHT = 250.f;
	static constexpr float MAX_CLIMB_HEIGHT = 450.f;
	static constexpr float SHRINK_COEFF = 0.95f;

	struct AsyncBuildContext
	{
		int x = -1;
		int y = -1;
		float tileBmin[3];
		float tileBmax[3];
		int dataSize = 0;
		unsigned char* data = nullptr;
	};

	union IntFloat
	{
		int vi;
		float vf;
	};

protected:
	struct ThreadContext
	{
		ThreadContext() = default;
		~ThreadContext() { cleanup(true); }
		void cleanup(bool cleanIds)
		{
			if (cleanIds) {
                cellIds.release();
                markedAreaIds.release();
				meshData.release();
			}
			rcFreeHeightField(solid);
			solid = 0;
			rcFreeCompactHeightfield(chf);
			chf = 0;
			rcFreeContourSet(cset);
			cset = 0;
			rcFreePolyMesh(pmesh);
			pmesh = 0;
			rcFreePolyMeshDetail(dmesh);
			dmesh = 0;
		}
		void init(int vertsNum, int trisNum)
		{
			try {
				meshData.verts = new float[3 * vertsNum];
				meshData.tris = new int[3 * trisNum];
				meshData.triFlags = new uint8_t[trisNum];
			} catch (...) {
				delete [] meshData.verts;
				meshData.verts = nullptr;
				delete [] meshData.tris;
				meshData.tris = nullptr;
				delete [] meshData.triFlags;
				meshData.triFlags = nullptr;
				throw;
			}
			meshData.vertsNum = vertsNum;
			meshData.trisNum = trisNum;
		}

		mesh::Grid2dBvh::TrianglesData meshData;
		common::ArrayBuffer<int> cellIds;
		common::ArrayBuffer<int> markedAreaIds;
		rcHeightfield* solid = nullptr;
		rcCompactHeightfield* chf = nullptr;
		rcContourSet* cset = nullptr;
		rcPolyMesh* pmesh = nullptr;
		rcPolyMeshDetail* dmesh = nullptr;
		rcConfig cfg;
	};

protected:
	bool m_keepInterResults;
	bool m_buildAll;
	float m_totalBuildTimeMs;
	std::unique_ptr<ThreadContext[]> m_asyncBuildData;
	std::thread m_asyncBuild;
	bool m_collected;
	std::unique_ptr<common::NavmeshGenParams[]> m_navGenParams;
	
	enum DrawMode
	{
        DRAWMODE_NAVMESH,
		DRAWMODE_NAVMESH_TRANS,
		DRAWMODE_NAVMESH_BVTREE,
		DRAWMODE_NAVMESH_NODES,
		DRAWMODE_NAVMESH_PORTALS,
		DRAWMODE_NAVMESH_INVIS,
		DRAWMODE_MESH,
		DRAWMODE_VOXELS,
		DRAWMODE_VOXELS_WALKABLE,
		DRAWMODE_VOXELS_WALKABLE_BBOXES,
		DRAWMODE_COMPACT,
		DRAWMODE_COMPACT_BBOXES,
		DRAWMODE_COMPACT_DISTANCE,
		DRAWMODE_COMPACT_REGIONS,
		DRAWMODE_REGION_CONNECTIONS,
		DRAWMODE_RAW_CONTOURS,
		DRAWMODE_BOTH_CONTOURS,
		DRAWMODE_CONTOURS,
		DRAWMODE_POLYMESH,
        DRAWMODE_POLYMESH_DETAIL,
		MAX_DRAWMODE
	};
		
	DrawMode m_drawMode;
    bool m_showNonTriPolys;
    bool m_highlightLiquidPolys;
	bool m_showVobsAabbs;
	bool m_showAverageNavmeshPolys;
	bool m_showPreliminaryJumpData;
	
	bool m_continueMeshGenWhileTileError;
	int m_maxTiles;
	int m_maxPolysPerTile;
	float m_tileSize;
	
	unsigned int m_tileCol;
	float m_lastBuiltTileBmin[3];
	float m_lastBuiltTileBmax[3];
	std::atomic_int m_tileBuildTime;
	std::atomic_int m_tileMemUsage;
	std::atomic_int m_tileTriCount;
    bool m_navmeshUpdated;
	std::atomic_bool m_asyncNavMeshGeneration;
	std::atomic_bool m_interruptAsyncBuilding;
	std::atomic_int m_asyncBuildingProgress;

protected:
	int buildTileMesh(
		int threadIndex,
		const int tx,
		const int ty,
		const float* bmin,
		const float* bmax,
		int& dataSize,
		unsigned char*& data
	);
	void saveAll(const char* path, const dtNavMesh* mesh);
	dtNavMesh* loadAll(const char* path);
	
public:
	Sample_TileMesh();
	virtual ~Sample_TileMesh();
	Sample_TileMesh(const Sample_TileMesh&) = delete;
	Sample_TileMesh& operator=(const Sample_TileMesh&) = delete;
	
	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
    virtual void handleRender(const float* cameraPos);
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(class InputGeom* geom);
	virtual bool handleBuild();
	virtual void interruptAsyncBuilding();
	virtual float getAsyncBuildingProgress() const;
	virtual bool isAsyncBuilding() const;
	
	void getTilePos(const float* pos, int& tx, int& ty);
	void buildTile(const float* pos);
	void removeTile(const float* pos);
	void buildAllTiles();
	void removeAllTiles();
	void printNavmeshInfo(const dtNavMesh* mesh) const;

private:
	bool initNavMesh();
	void initAsyncBuildData();
	bool initJmpNavmeshQuery();
	bool checkAndReinitJmpNavMeshQuery();

	void buildAllTilesDo(const float* bmin, const float* bmax, int tw, int th, float tcs);
	void collectNavmeshGenParams(common::NavmeshGenParams& params) const;
	void handleRenderOverlayOffsetPlanes(double* proj, double* model, int* view);

	std::unique_ptr<dtPoly::JmpAbilityInfoPoly[]> calcPreliminaryJumpData(
		const dtMeshTile* ctile,
		const float checkBboxFwdDst,
		const float checkBboxHeight,
		const float minClimbHeight,
		const float minClimbOverlappedHeight,
		const float maxClimbHeight,
		const float shrinkCoeff
	);
	void calcPreliminaryJumpData(
		std::unique_ptr<dtPoly::JmpAbilityInfoPoly[]>& outData,
		const dtMeshTile* ctile,
		const float checkBboxFwdDst,
		const float checkBboxHeight,
		const float minClimbHeight,
		const float minClimbOverlappedHeight,
		const float maxClimbHeight,
		const float shrinkCoeff
	);
	bool checkAbilityJumpDownOrForward(
		const float* v1,
		const float* v2,
		const float* polyCenter,
		const float checkBboxFwdDst,
		const float checkBboxHeight,
		const float shrinkCoeff
	);
	bool checkAbilityClimb(
		const float* v1,
		const float* v2,
		const float* polyCenter,
		const float forwardDistance,
		const float minClimbHeight,
		const float maxClimbHeight,
		const class dtQueryFilter* filter
	);
	bool checkAbilityClimbOverlapped(
		const float* polyVertices,
		const int verticesNum,
		const float* polyNorm,
		const float polyDist,
		const float minClimbHeight,
		const float maxClimbHeight,
		const class dtQueryFilter* filter
	);
};

#endif // RECASTSAMPLETILEMESH_H
