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

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <thread>
#include "Sample.h"
#include "InputGeom.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCrowd.h"
#include "imgui.h"
#include "SDL.h"
#include "SDL_opengl.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

static unsigned int areaToColFunc(unsigned int area)
{
    switch(area)
    {
    // Ground (0) : light blue
    case common::SamplePolyAreas::SAMPLE_POLYAREA_GROUND: return duRGBA(0, 192, 255, 255);
    // Water : blue
    case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER: return duRGBA(0, 0, 255, 255);
    // dodgerblue
    case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_WALKING: return duRGBA(128, 128, 255, 255);
    // mediumblue
    case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_FORDING: return duRGBA(0, 128, 128, 255);
    // blue
    case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_SWIMMING: return duRGBA(0, 0, 128, 255);
    // Road : brown
    case common::SamplePolyAreas::SAMPLE_POLYAREA_ROAD: return duRGBA(50, 20, 12, 255);
    // Forest : green
    case common::SamplePolyAreas::SAMPLE_POLYAREA_FOREST: return duRGBA(0, 255, 0, 255);
    // Door : cyan
	case common::SamplePolyAreas::SAMPLE_POLYAREA_DOOR: return duRGBA(255, 0, 0, 255);//duRGBA(0, 255, 255, 255);
    // Ladder : gray
    case common::SamplePolyAreas::SAMPLE_POLYAREA_LADDER: return duRGBA(128, 128, 128, 255);
    // Lava : orange
    case common::SamplePolyAreas::SAMPLE_POLYAREA_LAVA: return duRGBA(255, 69, 0, 255);
    // Unexpected : red
    default: return duRGBA(255, 0, 0, 255);
    }
}

unsigned int SampleDebugDraw::areaToCol(unsigned int area)
{
    return areaToColFunc(area);
}

Sample::Sample() :
	m_geom(0),
	m_navMesh(0),
	m_navQuery(0),
	m_crowd(0),
	m_navMeshDrawFlags(DU_DRAWNAVMESH_OFFMESHCONS|DU_DRAWNAVMESH_CLOSEDLIST),
	m_camSpeed(1.f),
	m_threadsNum(1.f),
	m_filterLowHangingObstacles(true),
	m_filterLedgeSpans(true),
	m_filterWalkableLowHeightSpans(true),
	m_erodeBorderSpans(false),
	m_tool(0),
    m_ctx(0),
	m_ddVboMesh(&areaToColFunc),
	m_ddVboNvm(&areaToColFunc),
	m_ddVboNvmMisc(&areaToColFunc),
	m_ddVboNvmTile(&areaToColFunc)
{
	resetCommonSettings();
	m_navQuery = dtAllocNavMeshQuery();
	m_crowd = dtAllocCrowd();

	for (int i = 0; i < MAX_TOOLS; i++)
		m_toolStates[i] = 0;

	m_camSpeed = 5000.f;
	m_threadsMax = static_cast<float>(std::thread::hardware_concurrency());
	if (m_threadsMax <= 1.f) {
		m_threadsMax = 1.f;
	} else {
		m_threadsMax -= 1.f;
	}
}

Sample::~Sample()
{
	dtFreeNavMeshQuery(m_navQuery);
	dtFreeNavMesh(m_navMesh);
	dtFreeCrowd(m_crowd);
	delete m_tool;
	for (int i = 0; i < MAX_TOOLS; i++)
		delete m_toolStates[i];
}

void Sample::setTool(SampleTool* tool)
{
	delete m_tool;
	m_tool = tool;
	if (tool)
		m_tool->init(this);
}

void Sample::handleSettings()
{
}

void Sample::handleTools()
{
}

void Sample::handleDebugMode()
{
}

void Sample::handleRender(const float* /*cameraPos*/)
{
    if (!m_geom)
		return;
	
	// Draw mesh
	duDebugDrawTriMesh(&m_dd, m_geom->getMeshExt()->getVerts(), m_geom->getMeshExt()->getVertCount(),
					   m_geom->getMeshExt()->getTris(), m_geom->getMeshExt()->getNormals(), m_geom->getMeshExt()->getTriCount(), 0, 1.0f);
	// Draw bounds
	const float* bmin = m_geom->getMeshBoundsMin();
	const float* bmax = m_geom->getMeshBoundsMax();
	duDebugDrawBoxWire(&m_dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
}

void Sample::handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/)
{
}

void Sample::handleMeshChanged(InputGeom* geom)
{
	m_geom = geom;
	resetDrawers();
}

void Sample::resetDrawers()
{
	m_ddVboMesh.reset();
	m_ddVboNvm.reset();
	m_ddVboNvmMisc.reset();
	m_ddVboNvmTile.reset();
}

void Sample::resetNavMeshDrawers()
{
	m_ddVboNvm.reset();
	m_ddVboNvmMisc.reset();
	m_ddVboNvmTile.reset();
}

void Sample::interruptAsyncBuilding()
{
	;
}

float Sample::getAsyncBuildingProgress() const
{
	return -1.f;
}

bool Sample::isAsyncBuilding() const
{
	return false;
}

void Sample::resetCommonSettings()
{
    m_cellSize = 10.f;//0.3f;
    m_cellHeight = 12.f;//0.2f;
    m_agentHeight = 180.f;//2.0f;
	m_agentLiquidWalk = 2.f;
	m_agentLiquidFord = 5.f;
	m_agentLiquidSwim = std::ceil(m_agentHeight / m_cellHeight) - 1;
    m_agentRadius = 35.f;//0.6f;
    m_agentMaxClimb = 40.f;//0.9f;
    m_agentMaxSlope = 50.f;//45.0f;
    m_regionMinSize = 15.f;//8;
    m_regionMergeSize = 15.f;//20;
    m_edgeMaxLen = 3.f;//12.0f;
    m_edgeMaxError = 2.8f;//1.3f;
	m_vertsPerPoly = 6.0f;
    m_detailSampleDist = 5.f;//6.0f;
	m_detailSampleMaxError = 1.0f;
	m_partitionType = SAMPLE_PARTITION_WATERSHED;
}

void Sample::handleCommonSettings()
{
	imguiSlider("Navmesh gen threads number", &m_threadsNum, 1.f, m_threadsMax, 1.f);
	imguiSlider("Cam speed", &m_camSpeed, 1.f, 5000.f, 1.f);

    imguiLabel("Rasterization");
    imguiSlider("Cell Size", &m_cellSize, /*0.1f*/1.f, 30.f, /*0.1f*/1.f);
    imguiSlider("Cell Height", &m_cellHeight, /*0.1f*/1.f, 30.f, /*0.1f*/1.f);
	
	if (m_geom)
	{
		const float* bmin = m_geom->getMeshBoundsMin();
		const float* bmax = m_geom->getMeshBoundsMax();
		int gw = 0, gh = 0;
		rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
		char text[64];
		snprintf(text, 64, "Voxels  %d x %d", gw, gh);
		imguiValue(text);
	}
	
	imguiSeparator();
	imguiLabel("Agent");
    imguiSlider("Height", &m_agentHeight, 50.f, 500.f, 5.f/*0.1f, 5.0f, 0.1f*/);
    imguiSlider("Radius", &m_agentRadius, 1.f, 200.f, 1.f/*0.0f, 5.0f, 0.1f*/);
    imguiSlider("Max Climb", &m_agentMaxClimb, 1.f, 50.f, 1.f/*0.1f, 5.0f, 0.1f*/);
	imguiSlider("Max Liquid Depth Walking", &m_agentLiquidWalk, 1.f, 10.f, 1.f);
	imguiSlider("Max Liquid Depth Fording", &m_agentLiquidFord, 2.f, 20.f, 1.f);
	if (m_agentLiquidFord <= m_agentLiquidWalk) m_agentLiquidFord += 1.f;
	imguiSlider("Min Liquid Depth Swimming", &m_agentLiquidSwim, 3.f, 30.f, 1.f);
	if (m_agentLiquidSwim <= m_agentLiquidFord) m_agentLiquidSwim += 1.f;
	imguiSlider("Max Slope", &m_agentMaxSlope, 0.0f, 90.0f, 1.0f);
	
	imguiSeparator();
	imguiLabel("Region");
    imguiSlider("Min Region Size", &m_regionMinSize, 0.0f, 150.0f, 1.0f);
	imguiSlider("Merged Region Size", &m_regionMergeSize, 0.0f, 150.0f, 1.0f);

	imguiSeparator();
	imguiLabel("Partitioning");
	if (imguiCheck("Watershed", m_partitionType == SAMPLE_PARTITION_WATERSHED))
		m_partitionType = SAMPLE_PARTITION_WATERSHED;
	if (imguiCheck("Monotone", m_partitionType == SAMPLE_PARTITION_MONOTONE))
		m_partitionType = SAMPLE_PARTITION_MONOTONE;
	if (imguiCheck("Layers", m_partitionType == SAMPLE_PARTITION_LAYERS))
		m_partitionType = SAMPLE_PARTITION_LAYERS;
	
	imguiSeparator();
	imguiLabel("Filtering");
	if (imguiCheck("Low Hanging Obstacles", m_filterLowHangingObstacles))
		m_filterLowHangingObstacles = !m_filterLowHangingObstacles;
	if (imguiCheck("Ledge Spans", m_filterLedgeSpans))
		m_filterLedgeSpans= !m_filterLedgeSpans;
	if (imguiCheck("Walkable Low Height Spans", m_filterWalkableLowHeightSpans))
		m_filterWalkableLowHeightSpans = !m_filterWalkableLowHeightSpans;
	if (imguiCheck("Erode border spans", m_erodeBorderSpans))
		m_erodeBorderSpans = !m_erodeBorderSpans;

	imguiSeparator();
	imguiLabel("Polygonization");
	imguiSlider("Max Edge Length", &m_edgeMaxLen, 0.0f, 50.0f, 1.0f);
    imguiSlider("Max Edge Error", &m_edgeMaxError, 0.1f, 30.0f, 0.1f);
	imguiSlider("Verts Per Poly", &m_vertsPerPoly, 3.0f, 12.0f, 1.0f);		

	imguiSeparator();
	imguiLabel("Detail Mesh");
	imguiSlider("Sample Distance", &m_detailSampleDist, 0.0f, 50.f/*16.0f*/, 1.0f);
	imguiSlider("Max Sample Error", &m_detailSampleMaxError, 0.0f, 50.f/*16.0f*/, 1.0f);
	
	imguiSeparator();
}

void Sample::handleClick(const float* s, const float* p, bool shift)
{
	if (m_tool)
		m_tool->handleClick(s, p, shift);
}

void Sample::handleToggle()
{
	if (m_tool)
		m_tool->handleToggle();
}

void Sample::handleStep()
{
	if (m_tool)
		m_tool->handleStep();
}

bool Sample::handleBuild()
{
	return true;
}

void Sample::handleUpdate(const float dt)
{
	if (m_tool)
		m_tool->handleUpdate(dt);
	updateToolStates(dt);
}


void Sample::updateToolStates(const float dt)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleUpdate(dt);
	}
}

void Sample::initToolStates(Sample* sample)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->init(sample);
	}
}

void Sample::resetToolStates()
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->reset();
	}
}

void Sample::renderToolStates()
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleRender();
	}
}

void Sample::renderOverlayToolStates(double* proj, double* model, int* view)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleRenderOverlay(proj, model, view);
	}
}

static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

dtNavMesh* Sample::loadAll(const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp)
		return 0;
	dtNavMesh* mesh = loadAll(fp);
	fclose(fp);
	return mesh;
}

dtNavMesh* Sample::loadAll(FILE* fp)
{
	// Read header.
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (readLen != 1)
	{
		return 0;
	}
	if (header.magic != NAVMESHSET_MAGIC)
	{
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		return 0;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		return 0;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		return 0;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (readLen != 1)
		{
			return 0;
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		readLen = fread(data, tileHeader.dataSize, 1, fp);
		if (readLen != 1)
		{
			dtFree(data);
			return 0;
		}

		if (!((dtMeshHeader*)data)->bvNodeCount)
		{
			dtFree(data);
			dtFreeNavMesh(mesh);
			m_ctx->log(RC_LOG_ERROR, "Error of Sample::loadAll, can't load tile without BVH");
			return 0;
		}
		mesh->addTile(
			data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0
		);
	}

	return mesh;
}

void Sample::saveAll(const char* path, const dtNavMesh* mesh)
{
	if (!mesh)
		return;
	FILE* fp = fopen(path, "wb");
	if (!fp)
		return;
	saveAll(fp, mesh);
	fclose(fp);
}

void Sample::saveAll(FILE* fp, const dtNavMesh* mesh)
{
	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}
}
