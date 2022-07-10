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
#include <chrono>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Sample_TileMesh.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "NavMeshTesterTool.h"
#include "ConvexVolumeTool.h"

#ifdef WIN32
	#define snprintf _snprintf
#endif

inline unsigned int nextPow2(unsigned int v)
{
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
}

inline unsigned int ilog2(unsigned int v)
{
    unsigned int r;
    unsigned int shift;
    r = (v > 0xffff) << 4; v >>= r;
    shift = (v > 0xff) << 3; v >>= shift; r |= shift;
    shift = (v > 0xf) << 2; v >>= shift; r |= shift;
    shift = (v > 0x3) << 1; v >>= shift; r |= shift;
    r |= (v >> 1);
    return r;
}

class NavMeshTileTool : public SampleTool
{
    Sample_TileMesh* m_sample;
    float m_hitPos[3];
    bool m_hitPosSet;
    bool& m_navmeshUpdated;

public:

    NavMeshTileTool(bool& navmeshUpdated) :
        m_sample(0),
        m_hitPosSet(false),
        m_navmeshUpdated(navmeshUpdated)
    {
        m_hitPos[0] = m_hitPos[1] = m_hitPos[2] = 0;
    }

    virtual ~NavMeshTileTool()
    {
    }

    virtual int type() { return TOOL_TILE_EDIT; }

    virtual void init(Sample* sample)
    {
        m_sample = (Sample_TileMesh*)sample;
    }

    virtual void reset() {}

    virtual void handleMenu()
    {
        imguiLabel("Create Tiles");
        if (imguiButton("Create All"))
        {
            if (m_sample)
                m_sample->buildAllTiles();
        }
        if (imguiButton("Remove All"))
        {
            if (m_sample)
                m_sample->removeAllTiles();
        }
    }

    virtual void handleClick(const float* /*s*/, const float* p, bool shift)
    {
        m_hitPosSet = true;
        rcVcopy(m_hitPos,p);
        if (m_sample)
        {
            if (shift)
                m_sample->removeTile(m_hitPos);
            else
                m_sample->buildTile(m_hitPos);
           m_navmeshUpdated = true;
        }
    }

    virtual void handleToggle() {}

    virtual void handleStep() {}

    virtual void handleUpdate(const float /*dt*/) {}

    virtual void handleRender()
    {
        if (m_hitPosSet)
        {
            const float s = m_sample->getAgentRadius();
            glColor4ub(0,0,0,128);
            glLineWidth(2.0f);
            glBegin(GL_LINES);
            glVertex3f(m_hitPos[0]-s,m_hitPos[1]+0.1f,m_hitPos[2]);
            glVertex3f(m_hitPos[0]+s,m_hitPos[1]+0.1f,m_hitPos[2]);
            glVertex3f(m_hitPos[0],m_hitPos[1]-s+0.1f,m_hitPos[2]);
            glVertex3f(m_hitPos[0],m_hitPos[1]+s+0.1f,m_hitPos[2]);
            glVertex3f(m_hitPos[0],m_hitPos[1]+0.1f,m_hitPos[2]-s);
            glVertex3f(m_hitPos[0],m_hitPos[1]+0.1f,m_hitPos[2]+s);
            glEnd();
            glLineWidth(1.0f);
        }
    }

    virtual void handleRenderOverlay(double* proj, double* model, int* view)
    {
        GLdouble x, y, z;
        if (m_hitPosSet && gluProject((GLdouble)m_hitPos[0], (GLdouble)m_hitPos[1], (GLdouble)m_hitPos[2],
                                      model, proj, view, &x, &y, &z))
        {
            int tx=0, ty=0;
            m_sample->getTilePos(m_hitPos, tx, ty);
            char text[32];
            snprintf(text,32,"(%d,%d)", tx,ty);
            imguiDrawText((int)x, (int)y-25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,220));
        }

        // Tool help
        const int h = view[3];
        imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "LMB: Rebuild hit tile.  Shift+LMB: Clear hit tile.", imguiRGBA(255,255,255,192));
    }
};

Sample_TileMesh::Sample_TileMesh() :
    m_keepInterResults(false),
    m_buildAll(true),
    m_totalBuildTimeMs(0),
    m_drawMode(DRAWMODE_NAVMESH),
    m_showNonTriPolys(false),
    m_highlightLiquidPolys(false),
	m_displayRefIdsInPath(false),
	m_continueMeshGenWhileTileError(true),
    m_maxTiles(0),
    m_maxPolysPerTile(0),
    m_tileSize(/*32*/256),
    m_tileCol(duRGBA(0,0,0,32)),
    m_tileBuildTime(0),
    m_tileMemUsage(0),
    m_tileTriCount(0),
	m_navmeshUpdated(),
	m_asyncNavMeshGeneration(false),
	m_interruptAsyncBuilding(false),
	m_asyncBuildingProgress(0),
	m_msgIdx(0)
{
    resetCommonSettings();
    memset(m_lastBuiltTileBmin, 0, sizeof(m_lastBuiltTileBmin));
    memset(m_lastBuiltTileBmax, 0, sizeof(m_lastBuiltTileBmax));
	m_asyncBuildData.reset( new ThreadContext[ static_cast<int>(m_threadsMax) ] );
    setTool(new NavMeshTileTool(m_navmeshUpdated));
}

Sample_TileMesh::~Sample_TileMesh()
{
    dtFreeNavMesh(m_navMesh);
    m_navMesh = 0;
}

void Sample_TileMesh::initAsyncBuildData()
{
	const Grid2dBvh& space = m_geom->getSpace();
	auto& dat = space.getEmptyOverlappingRectData();
	for (int i = 0, n = static_cast<int>(m_threadsMax); i < n; ++i) {
		m_asyncBuildData[i].cleanup(true);
		m_asyncBuildData[i].init(dat.vertsNum, dat.trisNum);
	}
}

void Sample_TileMesh::handleSettings()
{
    Sample::handleCommonSettings();

	if (imguiCheck("Keep Itermediate Results", m_keepInterResults)) {
        m_keepInterResults = !m_keepInterResults;
	}
	if (imguiCheck("Build All Tiles", m_buildAll)) {
        m_buildAll = !m_buildAll;
	}
	if (imguiCheck("Continue gen while tile error", m_continueMeshGenWhileTileError)) {
		m_continueMeshGenWhileTileError = !m_continueMeshGenWhileTileError;
	}
    imguiLabel("Tiling");
    imguiSlider("TileSize", &m_tileSize, 16.0f, 1024.0f, 16.0f);

    if (m_geom)
    {
        char text[64];
        int gw = 0, gh = 0;
        const float* bmin = m_geom->getNavMeshBoundsMin();
        const float* bmax = m_geom->getNavMeshBoundsMax();
        rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
        const int ts = (int)m_tileSize;
        const int tw = (gw + ts-1) / ts;
        const int th = (gh + ts-1) / ts;
        snprintf(text, 64, "Tiles  %d x %d", tw, th);
        imguiValue(text);

        // Max tiles and max polys affect how the tile IDs are caculated.
        // There are 22 bits available for identifying a tile and a polygon.
        int tileBits = rcMin((int)ilog2(nextPow2(tw*th)), 14);
        if (tileBits > 14) tileBits = 14;
        int polyBits = 22 - tileBits;
        m_maxTiles = 1 << tileBits;
        m_maxPolysPerTile = 1 << polyBits;
        snprintf(text, 64, "Max Tiles  %d", m_maxTiles);
        imguiValue(text);
        snprintf(text, 64, "Max Polys  %d", m_maxPolysPerTile);
        imguiValue(text);
    }
    else
    {
        m_maxTiles = 0;
        m_maxPolysPerTile = 0;
    }

    imguiSeparator();

    imguiIndent();
    imguiIndent();

    if (imguiButton("Save"))
    {
		saveAll(m_geom->getNavMeshName(), m_navMesh);
    }

    if (imguiButton("Load"))
    {
        dtFreeNavMesh(m_navMesh);
		m_navMesh = loadAll(m_geom->getNavMeshName());
		m_navQuery->init(m_navMesh, NAVMESH_QUERY_MAX_NODES);
		resetNavMeshDrawers();
    }

    imguiUnindent();
    imguiUnindent();

    char msg[64];
    snprintf(msg, 64, "Build Time: %.1fms", m_totalBuildTimeMs);
    imguiLabel(msg);

    imguiSeparator();

    imguiSeparator();

}

void Sample_TileMesh::handleTools()
{
    int type = !m_tool ? TOOL_NONE : m_tool->type();

    if (imguiCheck("Test Navmesh", type == TOOL_NAVMESH_TESTER))
    {
        NavMeshTesterTool* tool = new NavMeshTesterTool(getInputGeom());
        setTool(tool);
    }
    if (imguiCheck("Create Tiles", type == TOOL_TILE_EDIT))
    {
        setTool(new NavMeshTileTool(m_navmeshUpdated));
    }
    if (imguiCheck("Create Convex Volumes", type == TOOL_CONVEX_VOLUME))
    {
        setTool(new ConvexVolumeTool);
    }

    imguiSeparatorLine();

    imguiIndent();

	if (m_tool) {
		m_tool->handleMenu();
	}
	if (m_navmeshUpdated) {
		resetNavMeshDrawers();
		m_navmeshUpdated = false;
	}

    imguiUnindent();
}

void Sample_TileMesh::handleDebugMode()
{
	// Check which modes are valid.
	bool valid[MAX_DRAWMODE];
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		valid[i] = false;

	if (m_geom)
	{
		const ThreadContext& dat = m_asyncBuildData[0];
		valid[DRAWMODE_NAVMESH] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_TRANS] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_BVTREE] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_NODES] = m_navQuery != 0;
		valid[DRAWMODE_NAVMESH_PORTALS] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_INVIS] = m_navMesh != 0;
		valid[DRAWMODE_MESH] = true;
		valid[DRAWMODE_VOXELS] = dat.solid != 0;
		valid[DRAWMODE_VOXELS_WALKABLE] = dat.solid != 0;
		valid[DRAWMODE_VOXELS_WALKABLE_BBOXES] = dat.solid != 0;
		valid[DRAWMODE_COMPACT] = dat.chf != 0;
		valid[DRAWMODE_COMPACT_BBOXES] = dat.chf != 0;
		valid[DRAWMODE_COMPACT_DISTANCE] = dat.chf != 0;
		valid[DRAWMODE_COMPACT_REGIONS] = dat.chf != 0;
		valid[DRAWMODE_REGION_CONNECTIONS] = dat.cset != 0;
		valid[DRAWMODE_RAW_CONTOURS] = dat.cset != 0;
		valid[DRAWMODE_BOTH_CONTOURS] = dat.cset != 0;
		valid[DRAWMODE_CONTOURS] = dat.cset != 0;
		valid[DRAWMODE_POLYMESH] = dat.pmesh != 0;
		valid[DRAWMODE_POLYMESH_DETAIL] = dat.dmesh != 0;
	}

	int unavail = 0;
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		if (!valid[i]) unavail++;

	if (unavail == MAX_DRAWMODE)
		return;

	// mesh
	imguiLabel("Draw");
	if (imguiCheck("Highlight nontri polys", m_showNonTriPolys))
	{
		m_showNonTriPolys = !m_showNonTriPolys;
		m_ddVboMesh.reset();
	}
	if (imguiCheck("Highlight liquid polys", m_highlightLiquidPolys))
	{
		m_highlightLiquidPolys = !m_highlightLiquidPolys;
		m_ddVboMesh.reset();
	}

	// navmesh
	if (imguiCheck("Input Mesh", m_drawMode == DRAWMODE_MESH, valid[DRAWMODE_MESH]))
	{
		m_drawMode = DRAWMODE_MESH;
	}
	if (imguiCheck("Navmesh", m_drawMode == DRAWMODE_NAVMESH, valid[DRAWMODE_NAVMESH]))
	{
		if (m_drawMode != DRAWMODE_NAVMESH)
		{
			m_ddVboNvm.reset();
			m_ddVboNvmTile.reset();
		}
		m_drawMode = DRAWMODE_NAVMESH;
	}
	if (imguiCheck("Navmesh Invis", m_drawMode == DRAWMODE_NAVMESH_INVIS, valid[DRAWMODE_NAVMESH_INVIS]))
	{
		if (m_drawMode != DRAWMODE_NAVMESH_INVIS)
		{
			m_ddVboNvmMisc.reset();
			m_ddVboNvmTile.reset();
		}
		m_drawMode = DRAWMODE_NAVMESH_INVIS;
	}
	if (imguiCheck("Navmesh Trans", m_drawMode == DRAWMODE_NAVMESH_TRANS, valid[DRAWMODE_NAVMESH_TRANS]))
	{
		if (m_drawMode != DRAWMODE_NAVMESH_TRANS)
		{
			m_ddVboNvmMisc.reset();
			m_ddVboNvmTile.reset();
		}
		m_drawMode = DRAWMODE_NAVMESH_TRANS;
	}
	if (imguiCheck("Navmesh BVTree", m_drawMode == DRAWMODE_NAVMESH_BVTREE, valid[DRAWMODE_NAVMESH_BVTREE]))
	{
		if (m_drawMode != DRAWMODE_NAVMESH_BVTREE)
		{
			m_ddVboNvmMisc.reset();
			m_ddVboNvmTile.reset();
		}
		m_drawMode = DRAWMODE_NAVMESH_BVTREE;
	}
	if (imguiCheck("Navmesh Nodes", m_drawMode == DRAWMODE_NAVMESH_NODES, valid[DRAWMODE_NAVMESH_NODES]))
	{
		if (m_drawMode != DRAWMODE_NAVMESH_NODES)
		{
			m_ddVboNvmMisc.reset();
			m_ddVboNvmTile.reset();
		}
		m_drawMode = DRAWMODE_NAVMESH_NODES;
	}
	if (imguiCheck("Navmesh Portals", m_drawMode == DRAWMODE_NAVMESH_PORTALS, valid[DRAWMODE_NAVMESH_PORTALS]))
	{
		if (m_drawMode != DRAWMODE_NAVMESH_PORTALS)
		{
			m_ddVboNvmMisc.reset();
			m_ddVboNvmTile.reset();
		}
		m_drawMode = DRAWMODE_NAVMESH_PORTALS;
	}

	// tile mesh
	if (imguiCheck("Voxels", m_drawMode == DRAWMODE_VOXELS, valid[DRAWMODE_VOXELS]))
	{
		if (m_drawMode != DRAWMODE_VOXELS)
			m_ddVboNvmTile.reset();
		m_drawMode = DRAWMODE_VOXELS;
	}
    if (imguiCheck("Walkable Voxels", m_drawMode == DRAWMODE_VOXELS_WALKABLE, valid[DRAWMODE_VOXELS_WALKABLE]))
    {
		if (m_drawMode != DRAWMODE_VOXELS_WALKABLE)
			m_ddVboNvmTile.reset();
		m_drawMode = DRAWMODE_VOXELS_WALKABLE;
    }
    if (imguiCheck("Walkable Voxels Bboxes", m_drawMode == DRAWMODE_VOXELS_WALKABLE_BBOXES, valid[DRAWMODE_VOXELS_WALKABLE_BBOXES]))
    {
		if (m_drawMode != DRAWMODE_VOXELS_WALKABLE_BBOXES)
			m_ddVboNvmTile.reset();
		m_drawMode = DRAWMODE_VOXELS_WALKABLE_BBOXES;
    }
    if (imguiCheck("Compact", m_drawMode == DRAWMODE_COMPACT, valid[DRAWMODE_COMPACT]))
    {
		if (m_drawMode != DRAWMODE_COMPACT)
			m_ddVboNvmTile.reset();
		m_drawMode = DRAWMODE_COMPACT;
    }
    if (imguiCheck("Compact Bboxes", m_drawMode == DRAWMODE_COMPACT_BBOXES, valid[DRAWMODE_COMPACT_BBOXES]))
    {
		if (m_drawMode != DRAWMODE_COMPACT_BBOXES)
			m_ddVboNvmTile.reset();
		m_drawMode = DRAWMODE_COMPACT_BBOXES;
    }
	if (imguiCheck("Compact Distance", m_drawMode == DRAWMODE_COMPACT_DISTANCE, valid[DRAWMODE_COMPACT_DISTANCE]))
	{
		if (m_drawMode != DRAWMODE_COMPACT_DISTANCE)
			m_ddVboNvmTile.reset();
		m_drawMode = DRAWMODE_COMPACT_DISTANCE;
	}
	if (imguiCheck("Compact Regions", m_drawMode == DRAWMODE_COMPACT_REGIONS, valid[DRAWMODE_COMPACT_REGIONS]))
	{
		if (m_drawMode != DRAWMODE_COMPACT_REGIONS)
			m_ddVboNvmTile.reset();
		m_drawMode = DRAWMODE_COMPACT_REGIONS;
	}
	if (imguiCheck("Region Connections", m_drawMode == DRAWMODE_REGION_CONNECTIONS, valid[DRAWMODE_REGION_CONNECTIONS]))
	{
		if (m_drawMode != DRAWMODE_REGION_CONNECTIONS)
			m_ddVboNvmTile.reset();
		m_drawMode = DRAWMODE_REGION_CONNECTIONS;
	}
	if (imguiCheck("Raw Contours", m_drawMode == DRAWMODE_RAW_CONTOURS, valid[DRAWMODE_RAW_CONTOURS]))
	{
		if (m_drawMode != DRAWMODE_RAW_CONTOURS)
			m_ddVboNvmTile.reset();
		m_drawMode = DRAWMODE_RAW_CONTOURS;
	}
	if (imguiCheck("Both Contours", m_drawMode == DRAWMODE_BOTH_CONTOURS, valid[DRAWMODE_BOTH_CONTOURS]))
	{
		if (m_drawMode != DRAWMODE_BOTH_CONTOURS)
			m_ddVboNvmTile.reset();
		m_drawMode = DRAWMODE_BOTH_CONTOURS;
	}
	if (imguiCheck("Contours", m_drawMode == DRAWMODE_CONTOURS, valid[DRAWMODE_CONTOURS]))
	{
		if (m_drawMode != DRAWMODE_CONTOURS)
			m_ddVboNvmTile.reset();
		m_drawMode = DRAWMODE_CONTOURS;
	}
	if (imguiCheck("Poly Mesh", m_drawMode == DRAWMODE_POLYMESH, valid[DRAWMODE_POLYMESH]))
	{
		if (m_drawMode != DRAWMODE_POLYMESH)
			m_ddVboNvmTile.reset();
		m_drawMode = DRAWMODE_POLYMESH;
	}
	if (imguiCheck("Poly Mesh Detail", m_drawMode == DRAWMODE_POLYMESH_DETAIL, valid[DRAWMODE_POLYMESH_DETAIL]))
	{
		if (m_drawMode != DRAWMODE_POLYMESH_DETAIL)
			m_ddVboNvmTile.reset();
		m_drawMode = DRAWMODE_POLYMESH_DETAIL;
	}

    if (unavail)
    {
        imguiValue("Tick 'Keep Itermediate Results'");
        imguiValue("rebuild some tiles to see");
        imguiValue("more debug mode options.");
    }
}

void Sample_TileMesh::handleRender(const float* cameraPos)
{
	if (!m_geom || !m_geom->getMeshExt())
        return;
    const float texScale = 1.0f / (m_cellSize * 10.0f);
    // Draw mesh
    if (m_drawMode != DRAWMODE_NAVMESH_TRANS)
    {
		const Grid2dBvh& space = m_geom->getSpace();
        const Grid2dBvh::TrianglesData& rndData = space.getRenderingData();
		if (!m_ddVboMesh)
		{
			duDebugDrawTriMeshSlopeFast(
				&m_ddVboMesh, rndData.verts, rndData.vertsNumCurrent, rndData.tris,
				rndData.triFlags, rndData.normals, rndData.trisNumCurrent,
				m_agentMaxSlope, texScale, cameraPos, m_showNonTriPolys, m_highlightLiquidPolys
			);
		}
        m_geom->drawOffMeshConnections(&m_dd);
		m_ddVboMesh.draw();
    }

    glDepthMask(GL_FALSE);

    // Draw bounds
    const float* bmin = m_geom->getNavMeshBoundsMin();
    const float* bmax = m_geom->getNavMeshBoundsMax();
    duDebugDrawBoxWire(&m_dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
    // Tiling grid.
    int gw = 0, gh = 0;
    rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
    const int tw = (gw + (int)m_tileSize-1) / (int)m_tileSize;
    const int th = (gh + (int)m_tileSize-1) / (int)m_tileSize;
    const float s = m_tileSize*m_cellSize;
    duDebugDrawGridXZ(&m_dd, bmin[0],bmin[1],bmin[2], tw,th, s, duRGBA(0,0,0,64), 1.0f);
    duDebugDrawBoxWire(
        &m_dd, m_lastBuiltTileBmin[0],m_lastBuiltTileBmin[1],m_lastBuiltTileBmin[2],
        m_lastBuiltTileBmax[0],m_lastBuiltTileBmax[1],m_lastBuiltTileBmax[2], m_tileCol,
        1.0f
    );

	// Draw navmesh
    if (m_navMesh && m_navQuery &&
        (m_drawMode == DRAWMODE_NAVMESH ||
         m_drawMode == DRAWMODE_NAVMESH_TRANS ||
         m_drawMode == DRAWMODE_NAVMESH_BVTREE ||
         m_drawMode == DRAWMODE_NAVMESH_NODES ||
         m_drawMode == DRAWMODE_NAVMESH_PORTALS ||
         m_drawMode == DRAWMODE_NAVMESH_INVIS)
	) {
        if (m_drawMode != DRAWMODE_NAVMESH_INVIS)
		{
			if (!m_ddVboNvm)
			{
				duDebugDrawNavMeshWithClosedListFast(
					&m_ddVboNvm, *m_navMesh, *m_navQuery, m_navMeshDrawFlags
				);
			}
			m_ddVboNvm.draw();
        }
		if (m_drawMode == DRAWMODE_NAVMESH_BVTREE)
		{
			if (!m_ddVboNvmMisc)
				duDebugDrawNavMeshBVTree(&m_ddVboNvmMisc, *m_navMesh);
			m_ddVboNvmMisc.draw();
		}
		if (m_drawMode == DRAWMODE_NAVMESH_PORTALS)
		{
			if (!m_ddVboNvmMisc)
				duDebugDrawNavMeshPortals(&m_ddVboNvmMisc, *m_navMesh);
			m_ddVboNvmMisc.draw();
		}
		if (m_drawMode == DRAWMODE_NAVMESH_NODES)
		{
			if (!m_ddVboNvmMisc)
				duDebugDrawNavMeshNodes(&m_ddVboNvmMisc, *m_navQuery);
			m_ddVboNvmMisc.draw();
		}
		//duDebugDrawNavMeshPolysWithFlags(&m_dd, *m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0,0,0,128));
    }

    glDepthMask(GL_TRUE);

	// Draw active tile
	const ThreadContext& dat = m_asyncBuildData[0];
    if (dat.chf && m_drawMode == DRAWMODE_COMPACT)
	{
		if (!m_ddVboNvmTile)
			duDebugDrawCompactHeightfieldSolid(&m_ddVboNvmTile, *dat.chf);
		m_ddVboNvmTile.draw();
    }
    if (dat.chf && m_drawMode == DRAWMODE_COMPACT_BBOXES)
	{
		if (!m_ddVboNvmTile)
			duDebugDrawCompactHeightfieldSolidBboxes(&m_ddVboNvmTile, *dat.chf);
		m_ddVboNvmTile.draw();
    }
	if (dat.chf && m_drawMode == DRAWMODE_COMPACT_DISTANCE)
	{
		if (!m_ddVboNvmTile)
			duDebugDrawCompactHeightfieldDistance(&m_ddVboNvmTile, *dat.chf);
		m_ddVboNvmTile.draw();
	}
	if (dat.chf && m_drawMode == DRAWMODE_COMPACT_REGIONS)
	{
		if (!m_ddVboNvmTile)
			duDebugDrawCompactHeightfieldRegions(&m_ddVboNvmTile, *dat.chf);
		m_ddVboNvmTile.draw();
	}
	if (dat.solid && m_drawMode == DRAWMODE_VOXELS)
    {
        glEnable(GL_FOG);
		if (!m_ddVboNvmTile)
			duDebugDrawHeightfieldSolid(&m_ddVboNvmTile, *dat.solid);
		m_ddVboNvmTile.draw();
        glDisable(GL_FOG);
    }
	if (dat.solid && m_drawMode == DRAWMODE_VOXELS_WALKABLE)
    {
        glEnable(GL_FOG);
		if (!m_ddVboNvmTile)
			duDebugDrawHeightfieldWalkable(&m_ddVboNvmTile, *dat.solid);
		m_ddVboNvmTile.draw();
        glDisable(GL_FOG);
    }
    if (dat.solid && m_drawMode == DRAWMODE_VOXELS_WALKABLE_BBOXES)
    {
        glEnable(GL_FOG);
		if (!m_ddVboNvmTile)
			duDebugDrawHeightfieldWalkableBboxes(&m_ddVboNvmTile, *dat.solid);
		m_ddVboNvmTile.draw();
        glDisable(GL_FOG);
    }
	if (dat.cset && m_drawMode == DRAWMODE_RAW_CONTOURS)
    {
        glDepthMask(GL_FALSE);
		if (!m_ddVboNvmTile)
			duDebugDrawRawContours(&m_ddVboNvmTile, *dat.cset);
		m_ddVboNvmTile.draw();
        glDepthMask(GL_TRUE);
    }
	if (dat.cset && m_drawMode == DRAWMODE_BOTH_CONTOURS)
    {
        glDepthMask(GL_FALSE);
		if (!m_ddVboNvmTile)
		{
			duDebugDrawRawContours(&m_ddVboNvmTile, *dat.cset, 0.5f);
			duDebugDrawContours(&m_ddVboNvmTile, *dat.cset);
		}
		m_ddVboNvmTile.draw();
        glDepthMask(GL_TRUE);
    }
	if (dat.cset && m_drawMode == DRAWMODE_CONTOURS)
    {
        glDepthMask(GL_FALSE);
		if (!m_ddVboNvmTile)
			duDebugDrawContours(&m_ddVboNvmTile, *dat.cset);
		m_ddVboNvmTile.draw();
        glDepthMask(GL_TRUE);
    }
	if (dat.chf && dat.cset && m_drawMode == DRAWMODE_REGION_CONNECTIONS)
    {
		if (!m_ddVboNvmTile)
		{
			duDebugDrawCompactHeightfieldRegions(&m_ddVboNvmTile, *dat.chf);
			glDepthMask(GL_FALSE);
			duDebugDrawRegionConnections(&m_ddVboNvmTile, *dat.cset);
			glDepthMask(GL_TRUE);
		}
		m_ddVboNvmTile.draw();
    }
	if (dat.pmesh && m_drawMode == DRAWMODE_POLYMESH)
    {
        glDepthMask(GL_FALSE);
		if (!m_ddVboNvmTile)
			duDebugDrawPolyMesh(&m_ddVboNvmTile, *dat.pmesh);
		m_ddVboNvmTile.draw();
        glDepthMask(GL_TRUE);
    }
	if (dat.dmesh && m_drawMode == DRAWMODE_POLYMESH_DETAIL)
    {
        glDepthMask(GL_FALSE);
		if (!m_ddVboNvmTile)
			duDebugDrawPolyMeshDetail(&m_ddVboNvmTile, *dat.dmesh);
		m_ddVboNvmTile.draw();
        glDepthMask(GL_TRUE);
    }

	m_geom->drawConvexVolumes(&m_dd);

    if (m_tool)
        m_tool->handleRender();
    renderToolStates();

    glDepthMask(GL_TRUE);
}

void Sample_TileMesh::handleRenderOverlay(double* proj, double* model, int* view)
{
    GLdouble x, y, z;

    // Draw start and end point labels
	if (
		m_tileBuildTime > 0 &&
		gluProject(
			(GLdouble)(m_lastBuiltTileBmin[0]+m_lastBuiltTileBmax[0])/2,
			(GLdouble)(m_lastBuiltTileBmin[1]+m_lastBuiltTileBmax[1])/2,
			(GLdouble)(m_lastBuiltTileBmin[2]+m_lastBuiltTileBmax[2])/2,
			model, proj, view, &x, &y, &z
	)) {
		char text[128];
		snprintf(
			text, 128, "time: %d ms, tris: %d, mem: %.1f KB",
			m_tileBuildTime.load(), m_tileTriCount.load(), m_tileMemUsage.load() / 1024.f
		);
        imguiDrawText((int)x, (int)y-25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,220));
    }

    if (m_tool)
        m_tool->handleRenderOverlay(proj, model, view);
    renderOverlayToolStates(proj, model, view);
}

void Sample_TileMesh::handleMeshChanged(InputGeom* geom)
{
    Sample::handleMeshChanged(geom);

    const BuildSettings* buildSettings = geom->getBuildSettings();
	if (buildSettings && buildSettings->tileSize > 0) {
        m_tileSize = buildSettings->tileSize;
	}

	initAsyncBuildData();
    dtFreeNavMesh(m_navMesh);
    m_navMesh = 0;

    if (m_tool)
    {
        m_tool->reset();
        m_tool->init(this);
    }
    resetToolStates();
    initToolStates(this);
}

bool Sample_TileMesh::initNavMesh()
{
	dtFreeNavMesh(m_navMesh);
	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		m_ctx->log(RC_LOG_ERROR, "initNavMesh: Could not allocate navmesh");
		return false;
	}

	dtNavMeshParams params;
	rcVcopy(params.orig, m_geom->getNavMeshBoundsMin());
	params.tileWidth = m_tileSize*m_cellSize;
	params.tileHeight = m_tileSize*m_cellSize;
	params.maxTiles = m_maxTiles;
	params.maxPolys = m_maxPolysPerTile;
	dtStatus status;
	status = m_navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "initNavMesh: Could not init navmesh");
		return false;
	}

	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "initNavMesh: Could not init Detour navmesh query");
		return false;
	}

	return true;
}

bool Sample_TileMesh::handleBuild()
{
	if (!m_geom)
    {
		m_ctx->log(RC_LOG_ERROR, "handleBuild: No loaded mesh");
        return false;
    }
	if (!initNavMesh())
	{
		m_ctx->log(RC_LOG_ERROR, "handleBuild: Fail of navmesh init");
		return false;
	}

    if (m_buildAll)
        buildAllTiles();

    if (m_tool)
        m_tool->init(this);
    initToolStates(this);

    return true;
}

void Sample_TileMesh::collectSettings(BuildSettings& settings)
{
    Sample::collectSettings(settings);

    settings.tileSize = m_tileSize;
}

void Sample_TileMesh::buildTile(const float* pos)
{
	if (!m_geom)
	{
		m_ctx->log(RC_LOG_ERROR, "buildTile: No loaded mesh");
		return;
	}
	if (!m_navMesh)
	{
		if (!initNavMesh())
		{
			m_ctx->log(RC_LOG_ERROR, "buildTile: Fail of navmesh init");
			return;
		}
	}

    const float* bmin = m_geom->getNavMeshBoundsMin();
    const float* bmax = m_geom->getNavMeshBoundsMax();

    const float ts = m_tileSize*m_cellSize;
    const int tx = (int)((pos[0] - bmin[0]) / ts);
    const int ty = (int)((pos[2] - bmin[2]) / ts);

    m_lastBuiltTileBmin[0] = bmin[0] + tx*ts;
    m_lastBuiltTileBmin[1] = bmin[1];
    m_lastBuiltTileBmin[2] = bmin[2] + ty*ts;

    m_lastBuiltTileBmax[0] = bmin[0] + (tx+1)*ts;
    m_lastBuiltTileBmax[1] = bmax[1];
    m_lastBuiltTileBmax[2] = bmin[2] + (ty+1)*ts;

    m_tileCol = duRGBA(255,255,255,64);

    int dataSize = 0;
	m_tileMemUsage = 0;
	m_tileBuildTime = 0;
	m_tileTriCount = 0;
	unsigned char* data = nullptr;
	m_ctx->log(RC_LOG_WARNING, "Building tile with x: %d, y: %d", tx, ty);
	int ret = buildTileMesh(
		0, tx, ty, m_lastBuiltTileBmin, m_lastBuiltTileBmax, dataSize, data
	);
	if (ret) {
		m_ctx->log(
			RC_LOG_ERROR,
			"Error of single buildTileMesh, code: %d, tile x: %d, y: %d",
			ret, tx, ty
		);
		return;
	}

    // Remove any previous data (navmesh owns and deletes the data).
    m_navMesh->removeTile(m_navMesh->getTileRefAt(tx,ty,0),0,0);

    // Add tile, or leave the location empty.
    if (data)
    {
        // Let the navmesh own the data.
        dtStatus status = m_navMesh->addTile(data,dataSize,DT_TILE_FREE_DATA,0,0);
        if (dtStatusFailed(status))
            dtFree(data);
    }
	resetNavMeshDrawers();

	m_ctx->dumpLog(m_msgIdx, "Build Tile (%d,%d):", tx, ty);
	m_msgIdx = m_ctx->getLogCount();
}

void Sample_TileMesh::getTilePos(const float* pos, int& tx, int& ty)
{
    if (!m_geom) return;

    const float* bmin = m_geom->getNavMeshBoundsMin();

    const float ts = m_tileSize*m_cellSize;
    tx = (int)((pos[0] - bmin[0]) / ts);
    ty = (int)((pos[2] - bmin[2]) / ts);
}

void Sample_TileMesh::removeTile(const float* pos)
{
    if (!m_geom) return;
    if (!m_navMesh) return;

    const float* bmin = m_geom->getNavMeshBoundsMin();
    const float* bmax = m_geom->getNavMeshBoundsMax();

    const float ts = m_tileSize*m_cellSize;
    const int tx = (int)((pos[0] - bmin[0]) / ts);
    const int ty = (int)((pos[2] - bmin[2]) / ts);

    m_lastBuiltTileBmin[0] = bmin[0] + tx*ts;
    m_lastBuiltTileBmin[1] = bmin[1];
    m_lastBuiltTileBmin[2] = bmin[2] + ty*ts;

    m_lastBuiltTileBmax[0] = bmin[0] + (tx+1)*ts;
    m_lastBuiltTileBmax[1] = bmax[1];
    m_lastBuiltTileBmax[2] = bmin[2] + (ty+1)*ts;

    m_tileCol = duRGBA(128,32,16,64);

    m_navMesh->removeTile(m_navMesh->getTileRefAt(tx,ty,0),0,0);

	resetNavMeshDrawers();
}

void Sample_TileMesh::buildAllTiles()
{
    if (!m_geom) return;
    if (!m_navMesh) return;

    const float* bmin = m_geom->getNavMeshBoundsMin();
    const float* bmax = m_geom->getNavMeshBoundsMax();
    int gw = 0, gh = 0;
    rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
    const int ts = (int)m_tileSize;
    const int tw = (gw + ts-1) / ts;
    const int th = (gh + ts-1) / ts;
    const float tcs = m_tileSize*m_cellSize;

	m_ctx->resetLog();
	m_msgIdx = 0;
	m_tileMemUsage = 0;
	m_tileBuildTime = 0;
	m_tileTriCount = 0;
	m_asyncBuild = std::thread(
		&Sample_TileMesh::buildAllTilesDo, this, bmin, bmax, tw, th, tcs
	);
	m_asyncBuild.detach();
	m_asyncNavMeshGeneration = true;
}

void Sample_TileMesh::buildAllTilesDo(
	const float* bmin, const float* bmax, int tw, int th, float tcs
) {
	static const float FIRST_STAGE_PROGRESS = 0.95;
	static const float SECOND_STAGE_PROGRESS = 0.05;
	static const int TILES_PER_ITERATION_ADD = 50;
	std::unique_ptr<AsyncBuildContext[]> tilesData;
	int n = tw * th;
	std::atomic_int dataIndex(0);
	std::atomic_bool error(false);
	auto processor = [this, &tilesData, n, &dataIndex, &error] (
		int threadIndex, std::atomic_bool* ready
	) {
		try {
			while (true) {
				if (error || m_interruptAsyncBuilding) {
					break;
				}
				int i = dataIndex.fetch_add(1);
				if (i >= n) {
					break;
				}
				AsyncBuildContext& e = tilesData[i];
				e.data = nullptr;
				int ret = buildTileMesh(
					threadIndex, e.x, e.y, e.tileBmin, e.tileBmax, e.dataSize, e.data
				);
				if (ret) {
					m_ctx->log(
						RC_LOG_ERROR,
						"Error of buildTileMesh, code: %d, tile x: %d, y: %d",
						ret, e.x, e.y
					);
					if (!m_continueMeshGenWhileTileError) {
						error = true;
					}
				}
			}
		}
		catch (const std::exception& e) {
			error = true;
			m_ctx->log(RC_LOG_ERROR, "Mth: Has occured an exception, msg: %s", e.what());
		}
		catch (...) {
			error = true;
			m_ctx->log(RC_LOG_ERROR, "Mth: Has occured an unknown exception");
		}
		ready->store(true);
	};
	auto finCalculation = [this] () {
		m_ctx->enableTimer(true);
		m_ctx->stopTimer(RC_TIMER_TEMP);
		m_totalBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TEMP)/1000.0f;
		resetNavMeshDrawers();
		m_asyncNavMeshGeneration = false;
		m_interruptAsyncBuilding = false;
		m_msgIdx = m_ctx->getLogCount();
		m_asyncBuildingProgress = 0;
	};

	m_interruptAsyncBuilding = false;
	try {
		// Start the build process.
		m_ctx->startTimer(RC_TIMER_TEMP);
		m_ctx->enableTimer(false);

		tilesData.reset(new AsyncBuildContext [n]);
		if (!tilesData) {
			m_ctx->log(RC_LOG_ERROR, "Can't allocate memory for tiles data");
			finCalculation();
			return;
		}
		for (int y = 0; y < th; ++y)
		{
			for (int x = 0; x < tw; ++x)
			{
				AsyncBuildContext& e = tilesData[y * tw + x];
				e.y = y;
				e.x = x;
				e.tileBmin[0] = bmin[0] + x*tcs;
				e.tileBmin[1] = bmin[1];
				e.tileBmin[2] = bmin[2] + y*tcs;
				e.tileBmax[0] = bmin[0] + (x+1)*tcs;
				e.tileBmax[1] = bmax[1];
				e.tileBmax[2] = bmin[2] + (y+1)*tcs;
			}
		}

		const int nThreads = static_cast<int>(m_threadsNum);
		std::unique_ptr<std::pair<std::thread, std::atomic_bool>[]> threads(
			new std::pair<std::thread, std::atomic_bool>[nThreads]
		);
		for (int i = 0; i < nThreads; ++i) {
			auto& ref = threads[i];
			ref.second = false;
			ref.first = std::thread(processor, i, &ref.second);
		}
		for (int i = 0; i < nThreads; ++i) {
			while (!threads[i].second) {
				std::this_thread::sleep_for(std::chrono::seconds(1));
				float val = (FIRST_STAGE_PROGRESS * ((float)dataIndex.load() / n)) * 100.f;
				IntFloat conv;
				conv.vf = val;
				m_asyncBuildingProgress = conv.vi;
			}
		}
		for (int i = 0; i < nThreads; ++i) {
			threads[i].first.join();
		}
		if (error || m_interruptAsyncBuilding) {
			for (int i = 0; i < n; ++i) {
				dtFree(tilesData[i].data);
			}
			if (error)
				m_ctx->log(RC_LOG_ERROR, "Has occured an error of buildTileMesh");
			finCalculation();
			return;
		}

		for (int i = 0; i < n; ++i) {
			AsyncBuildContext& e = tilesData[i];
			// Remove any previous data (navmesh owns and deletes the data).
			m_navMesh->removeTile(m_navMesh->getTileRefAt(e.x, e.y, 0), 0, 0);
			// Let the navmesh own the data.
			dtStatus status;
			if (!e.data) {
				m_ctx->log(RC_LOG_WARNING, "Empty tile data has skipped");
				status = DT_SUCCESS;
			} else {
				status = m_navMesh->addTile(e.data, e.dataSize, DT_TILE_FREE_DATA, 0, 0);
			}
			if (dtStatusFailed(status)) {
				dtFree(e.data);
				for (int j = i; j < n; ++j) {
					dtFree(tilesData[j].data);
				}
				m_ctx->log(RC_LOG_ERROR, "Has occured an error of addTile");
				break;
			}
			if (i && i % TILES_PER_ITERATION_ADD == 0) {
				float val =
					(FIRST_STAGE_PROGRESS + SECOND_STAGE_PROGRESS * ((float)i / n)) * 100.f;
				IntFloat conv;
				conv.vf = val;
				m_asyncBuildingProgress = conv.vi;
			}
		}
		m_ctx->log(RC_LOG_PROGRESS, "Progress: 100%");
	}
	catch (const std::exception& e) {
		m_ctx->log(RC_LOG_ERROR, "Has occured an exception, msg: %s", e.what());
	}
	catch (...) {
		m_ctx->log(RC_LOG_ERROR, "Has occured an unknown exception");
	}

	// Stop the build process.
	finCalculation();
}

void Sample_TileMesh::interruptAsyncBuilding()
{
	m_interruptAsyncBuilding = true;
}

float Sample_TileMesh::getAsyncBuildingProgress() const
{
	int val = m_asyncBuildingProgress.load();
	IntFloat conv;
	conv.vi = val;
	return conv.vf;
}

bool Sample_TileMesh::isAsyncBuilding() const
{
	return m_asyncNavMeshGeneration;
}

void Sample_TileMesh::removeAllTiles()
{
    if (!m_geom || !m_navMesh)
        return;

    const float* bmin = m_geom->getNavMeshBoundsMin();
    const float* bmax = m_geom->getNavMeshBoundsMax();
    int gw = 0, gh = 0;
    rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
    const int ts = (int)m_tileSize;
    const int tw = (gw + ts-1) / ts;
    const int th = (gh + ts-1) / ts;

    for (int y = 0; y < th; ++y)
        for (int x = 0; x < tw; ++x)
            m_navMesh->removeTile(m_navMesh->getTileRefAt(x,y,0),0,0);
	resetNavMeshDrawers();
}

int Sample_TileMesh::buildTileMesh(
	int threadIndex,
	const int tx,
	const int ty,
	const float* bmin,
	const float* bmax,
	int& dataSize,
	unsigned char*& data
) {
	auto saveMaxAtomicInt = [] (std::atomic_int& atomicVal, int newVal) {
		int cur = atomicVal.load();
		if (newVal <= cur)
			return;
		while (!atomicVal.compare_exchange_strong(cur, newVal));
	};

	const Grid2dBvh& space = m_geom->getSpace();
	if (!space.isLoaded())
    {
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified");
		return 1;
    }
	ThreadContext& genCtx = m_asyncBuildData[threadIndex];
	genCtx.cleanup(false);

    // Init build configuration from GUI
	memset(&genCtx.cfg, 0, sizeof(genCtx.cfg));
	genCtx.cfg.cs = m_cellSize;
	genCtx.cfg.ch = m_cellHeight;
	genCtx.cfg.walkableSlopeAngle = m_agentMaxSlope;
	genCtx.cfg.walkableHeight = (int)ceilf(m_agentHeight / genCtx.cfg.ch);
	genCtx.cfg.walkableClimb = (int)floorf(m_agentMaxClimb / genCtx.cfg.ch);
	genCtx.cfg.walkableRadius = (int)ceilf(m_agentRadius / genCtx.cfg.cs);
	genCtx.cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	genCtx.cfg.maxSimplificationError = m_edgeMaxError;
	genCtx.cfg.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	genCtx.cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	genCtx.cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	genCtx.cfg.tileSize = (int)m_tileSize;
	genCtx.cfg.borderSize = genCtx.cfg.walkableRadius + 3; // Reserve enough padding.
	genCtx.cfg.width = genCtx.cfg.tileSize + genCtx.cfg.borderSize*2;
	genCtx.cfg.height = genCtx.cfg.tileSize + genCtx.cfg.borderSize*2;
	genCtx.cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	genCtx.cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;

    // Expand the heighfield bounding box by border size to find the extents of geometry we need to build this tile.
    //
    // This is done in order to make sure that the navmesh tiles connect correctly at the borders,
    // and the obstacles close to the border work correctly with the dilation process.
    // No polygons (or contours) will be created on the border area.
    //
    // IMPORTANT!
    //
    //   :''''''''':
    //   : +-----+ :
    //   : |     | :
    //   : |     |<--- tile to build
    //   : |     | :
    //   : +-----+ :<-- geometry needed
    //   :.........:
    //
    // You should use this bounding box to query your input geometry.
    //
    // For example if you build a navmesh for terrain, and want the navmesh tiles to match the terrain tile size
    // you will need to pass in data from neighbour terrain tiles too! In a simple case, just pass in all the 8 neighbours,
    // or use the bounding box below to only pass in a sliver of each of the 8 neighbours.
	rcVcopy(genCtx.cfg.bmin, bmin);
	rcVcopy(genCtx.cfg.bmax, bmax);
	genCtx.cfg.bmin[0] -= genCtx.cfg.borderSize * genCtx.cfg.cs;
	genCtx.cfg.bmin[2] -= genCtx.cfg.borderSize * genCtx.cfg.cs;
	genCtx.cfg.bmax[0] += genCtx.cfg.borderSize * genCtx.cfg.cs;
	genCtx.cfg.bmax[2] += genCtx.cfg.borderSize * genCtx.cfg.cs;

    // Reset build times gathering.
    m_ctx->resetTimers();

    // Allocate voxel heightfield where we rasterize our input data to.
	genCtx.solid = rcAllocHeightfield();
	if (!genCtx.solid)
    {
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'");
		return 2;
    }
	if (!rcCreateHeightfield(
		m_ctx, *genCtx.solid, genCtx.cfg.width, genCtx.cfg.height,
		genCtx.cfg.bmin, genCtx.cfg.bmax, genCtx.cfg.cs, genCtx.cfg.ch
	)) {
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield");
		return 3;
    }

    int n = space.getOverlappingRectCellIds(
        genCtx.cfg.bmin, genCtx.cfg.bmax, genCtx.cellIds.data, genCtx.cellIds.num
	);
    if (n > genCtx.cellIds.num) {
        delete [] genCtx.cellIds.data;
        genCtx.cellIds.data = nullptr;
        genCtx.cellIds.data = new int[n];
        if (!genCtx.cellIds.data) {
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory while cell ids allocation");
			return 4;
		}
        genCtx.cellIds.num = n;
        n = space.getOverlappingRectCellIds(
            genCtx.cfg.bmin, genCtx.cfg.bmax, genCtx.cellIds.data, genCtx.cellIds.num
		);
	}
	if (!n) {
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Empty space !");
		return 5;
	}
	// Start the build process.
	m_ctx->startTimer(RC_TIMER_TOTAL);
	if (!m_asyncNavMeshGeneration) {
		char msgBuf[256];
		snprintf(
			msgBuf, 256, "buildNavigation: %d x %d cells, grids number: %d",
			genCtx.cfg.width, genCtx.cfg.height, n
		);
		m_ctx->log(RC_LOG_PROGRESS, "%s", msgBuf);
	}

	int tileTriCount = 0;
	for (int i = 0; i < n; ++i)
    {
		Grid2dBvh::TrianglesData& dat = genCtx.meshData;
        space.extractOverlappingRectData(genCtx.cellIds.data[i], dat);
		tileTriCount += dat.trisNumCurrent;

		rcMarkWalkableTriangles(
			m_ctx, genCtx.cfg.walkableSlopeAngle, dat.verts, dat.vertsNumCurrent,
			dat.tris, dat.trisNumCurrent, dat.triFlags
		);

		if (!rcRasterizeTriangles(
			m_ctx, dat.verts, dat.vertsNumCurrent, dat.tris, dat.triFlags,
			dat.trisNumCurrent, *genCtx.solid, genCtx.cfg.walkableClimb
		))
			return 6;
    }
	saveMaxAtomicInt(m_tileTriCount, tileTriCount);

    // Once all geometry is rasterized, we do initial pass of filtering to
    // remove unwanted overhangs caused by the conservative rasterization
    // as well as filter spans where the character cannot possibly stand.
	if (m_filterLowHangingObstacles) {
		rcFilterLowHangingWalkableObstacles(m_ctx, genCtx.cfg.walkableClimb, *genCtx.solid);
	}
	if (m_filterLedgeSpans) {
		rcFilterLedgeSpans(
			m_ctx, genCtx.cfg.walkableHeight, genCtx.cfg.walkableClimb, *genCtx.solid
		);
	}
	if (m_filterWalkableLowHeightSpans) {
		rcFilterWalkableLowHeightSpans(m_ctx, genCtx.cfg.walkableHeight, *genCtx.solid);
	}

    // Compact the heightfield so that it is faster to handle from now on.
    // This will result more cache coherent data as well as the neighbours
    // between walkable cells will be calculated.
	genCtx.chf = rcAllocCompactHeightfield();
	if (!genCtx.chf)
    {
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'");
		return 7;
    }
	if (!rcBuildCompactHeightfield_FirstVersion(
		m_ctx, genCtx.cfg.walkableHeight, genCtx.cfg.walkableClimb, *genCtx.solid,
		*genCtx.chf, static_cast<int>(m_agentLiquidWalk), static_cast<int>(m_agentLiquidFord),
		static_cast<int>(m_agentLiquidSwim)
	)) {
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data");
		return 8;
    }

    // Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea_SecondVersion(
	//if (!rcErodeWalkableArea(
		m_ctx, genCtx.cfg.walkableRadius, *genCtx.chf, genCtx.solid, m_erodeBorderSpans
	)) {
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode");
		return 9;
    }

	if (!m_keepInterResults)
	{
		rcFreeHeightField(genCtx.solid);
		genCtx.solid = 0;
	}

    // (Optional) Mark areas.
    n = space.getOverlappingRectMarkedAreaIds(
        genCtx.cfg.bmin, genCtx.cfg.bmax, genCtx.markedAreaIds.data, genCtx.markedAreaIds.size
    );
    if (n > genCtx.markedAreaIds.size) {
        delete [] genCtx.markedAreaIds.data;
        genCtx.markedAreaIds.data = nullptr;
        genCtx.markedAreaIds.data = new int[n];
        if (!genCtx.markedAreaIds.data) {
            m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory while marked area ids allocation");
            return 10;
        }
        genCtx.markedAreaIds.size = n;
        n = space.getOverlappingRectMarkedAreaIds(
            genCtx.cfg.bmin, genCtx.cfg.bmax, genCtx.markedAreaIds.data, genCtx.markedAreaIds.size
        );
    }
    if (n) {
        std::sort(genCtx.markedAreaIds.data, genCtx.markedAreaIds.data + n);
        int* end = std::unique(genCtx.markedAreaIds.data, genCtx.markedAreaIds.data + n);
        n = static_cast<int>(end - genCtx.markedAreaIds.data);
        for (int i = 0; i < n; ++i) {
            const rcMeshLoaderObjExt::MarkedEntry& area =
                *space.getMarkedArea(genCtx.markedAreaIds.data[i]);
            rcMarkConvexPolyArea(
                m_ctx, area.verts, area.vertsNum, area.minh,
                area.maxh, (unsigned char)area.area, *genCtx.chf
            );
        }
    }

    // Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
    // There are 3 martitioning methods, each with some pros and cons:
    // 1) Watershed partitioning
    //   - the classic Recast partitioning
    //   - creates the nicest tessellation
    //   - usually slowest
    //   - partitions the heightfield into nice regions without holes or overlaps
    //   - the are some corner cases where this method creates produces holes and overlaps
    //      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
    //      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
    //   * generally the best choice if you precompute the nacmesh, use this if you have large open areas
    // 2) Monotone partioning
    //   - fastest
    //   - partitions the heightfield into regions without holes and overlaps (guaranteed)
    //   - creates long thin polygons, which sometimes causes paths with detours
    //   * use this if you want fast navmesh generation
    // 3) Layer partitoining
    //   - quite fast
    //   - partitions the heighfield into non-overlapping regions
    //   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
    //   - produces better triangles than monotone partitioning
    //   - does not have the corner cases of watershed partitioning
    //   - can be slow and create a bit ugly tessellation (still better than monotone)
    //     if you have large open areas with small obstacles (not a problem if you use tiles)
    //   * good choice to use for tiled navmesh with medium and small sized tiles

    if (m_partitionType == SAMPLE_PARTITION_WATERSHED)
    {
        // Prepare for region partitioning, by calculating distance field along the walkable surface.
		if (!rcBuildDistanceField(m_ctx, *genCtx.chf))
        {
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field");
            return 11;
        }

        // Partition the walkable surface into simple regions without holes.
		if (!rcBuildRegions(
			m_ctx, *genCtx.chf, genCtx.cfg.borderSize, genCtx.cfg.minRegionArea,
			genCtx.cfg.mergeRegionArea
		)) {
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions");
            return 12;
        }
    }
    else if (m_partitionType == SAMPLE_PARTITION_MONOTONE)
    {
        // Partition the walkable surface into simple regions without holes.
        // Monotone partitioning does not need distancefield.
		if (!rcBuildRegionsMonotone(
			m_ctx, *genCtx.chf, genCtx.cfg.borderSize, genCtx.cfg.minRegionArea,
			genCtx.cfg.mergeRegionArea
		)) {
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions");
            return 13;
        }
    }
    else // SAMPLE_PARTITION_LAYERS
    {
        // Partition the walkable surface into simple regions without holes.
		if (!rcBuildLayerRegions(
			m_ctx, *genCtx.chf, genCtx.cfg.borderSize, genCtx.cfg.minRegionArea
		)) {
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions");
            return 14;
        }
    }

    // Create contours.
	genCtx.cset = rcAllocContourSet();
	if (!genCtx.cset)
    {
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'");
        return 15;
    }
	if (!rcBuildContours(
		m_ctx, *genCtx.chf, genCtx.cfg.maxSimplificationError,
		genCtx.cfg.maxEdgeLen, *genCtx.cset
	)) {
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours");
        return 16;
    }
	if (genCtx.cset->nconts == 0)
    {
		m_ctx->log(RC_LOG_WARNING, "buildNavigation: No contours");
		return 0;
    }

    // Build polygon navmesh from the contours.
	genCtx.pmesh = rcAllocPolyMesh();
	if (!genCtx.pmesh)
    {
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'");
        return 17;
    }
	if (!rcBuildPolyMesh(m_ctx, *genCtx.cset, genCtx.cfg.maxVertsPerPoly, *genCtx.pmesh))
    {
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours");
        return 18;
	}

    // Build detail mesh.
	genCtx.dmesh = rcAllocPolyMeshDetail();
	if (!genCtx.dmesh)
    {
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'dmesh'");
        return 19;
    }
	if (!rcBuildPolyMeshDetail(
		m_ctx, *genCtx.pmesh, *genCtx.chf, genCtx.cfg.detailSampleDist,
		genCtx.cfg.detailSampleMaxError, *genCtx.dmesh
	)) {
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could build polymesh detail");
        return 20;
    }

    if (!m_keepInterResults)
    {
		rcFreeCompactHeightfield(genCtx.chf);
		genCtx.chf = 0;
		rcFreeContourSet(genCtx.cset);
		genCtx.cset = 0;
    }

    unsigned char* navData = 0;
    int navDataSize = 0;
	if (genCtx.cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
    {
		if (genCtx.pmesh->nverts >= 0xffff)
        {
            // The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			m_ctx->log(
				RC_LOG_ERROR, "buildNavigation: Too many vertices per tile %d (max: %d)",
				genCtx.pmesh->nverts, 0xffff
			);
            return 21;
        }

        // Update poly flags from areas.
		for (int i = 0; i < genCtx.pmesh->npolys; ++i)
        {
			if (genCtx.pmesh->areas[i] == RC_WALKABLE_AREA) {
				genCtx.pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;
				genCtx.pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
                continue;
            }
            bool inhabited = PolyAreaFlags::isInhabitedFlag(genCtx.pmesh->areas[i]);
            unsigned char area = PolyAreaFlags::clearInhabitedFlag(genCtx.pmesh->areas[i]);
			//assert(genCtx.pmesh->areas[i] != PolyAreaFlags::WATER_COMMON);
			genCtx.pmesh->flags[i] = SAMPLE_POLYFLAGS_INHABITED & (uint16_t(0) - inhabited);
            if (area == PolyAreaFlags::WATER_COMMON) {
				genCtx.pmesh->areas[i] = SAMPLE_POLYAREA_WATER;
				genCtx.pmesh->flags[i] |= SAMPLE_POLYFLAGS_DISABLED;
            }
            else if (area == PolyAreaFlags::WATER_SHALLOW) {
				genCtx.pmesh->areas[i] = SAMPLE_POLYAREA_WATER_WALKING;
				genCtx.pmesh->flags[i] |= SAMPLE_POLYFLAGS_WATER_WALKING;
            }
            else if (area == PolyAreaFlags::WATER_MIDDLE) {
				genCtx.pmesh->areas[i] = SAMPLE_POLYAREA_WATER_FORDING;
				genCtx.pmesh->flags[i] |= SAMPLE_POLYFLAGS_WATER_FORDING;
            }
            else if (area == PolyAreaFlags::WATER_DEEP) {
				genCtx.pmesh->areas[i] = SAMPLE_POLYAREA_WATER_SWIMMING;
				genCtx.pmesh->flags[i] |= SAMPLE_POLYFLAGS_WATER_SWIMMING;
            }
            else if (area == PolyAreaFlags::GROUND) {
				genCtx.pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;
				genCtx.pmesh->flags[i] |= SAMPLE_POLYFLAGS_WALK;
            }
            else if (area == PolyAreaFlags::LAVA) {
				genCtx.pmesh->areas[i] = SAMPLE_POLYAREA_LAVA;
				genCtx.pmesh->flags[i] |= SAMPLE_POLYFLAGS_DISABLED;
            }
            else if (area == PolyAreaFlags::ROAD) {
				genCtx.pmesh->areas[i] = SAMPLE_POLYAREA_ROAD;
				genCtx.pmesh->flags[i] |= SAMPLE_POLYFLAGS_ROAD | SAMPLE_POLYFLAGS_WALK;
            }
            else if (area == PolyAreaFlags::FOREST) {
				genCtx.pmesh->areas[i] = SAMPLE_POLYAREA_FOREST;
				genCtx.pmesh->flags[i] |= SAMPLE_POLYFLAGS_FOREST | SAMPLE_POLYFLAGS_WALK;
            }
            else if (area == PolyAreaFlags::DOOR) {
				genCtx.pmesh->areas[i] = SAMPLE_POLYAREA_DOOR;
				genCtx.pmesh->flags[i] |= SAMPLE_POLYFLAGS_DOOR | SAMPLE_POLYFLAGS_WALK;
            }
            else if (area == PolyAreaFlags::LADDER) {
				genCtx.pmesh->areas[i] = SAMPLE_POLYAREA_LADDER;
				genCtx.pmesh->flags[i] |= SAMPLE_POLYFLAGS_LADDER | SAMPLE_POLYFLAGS_WALK;
            }
            else {
                assert(false);
            }
        }

        dtNavMeshCreateParams params;
        memset(&params, 0, sizeof(params));
		params.verts = genCtx.pmesh->verts;
		params.vertCount = genCtx.pmesh->nverts;
		params.polys = genCtx.pmesh->polys;
		params.polyAreas = genCtx.pmesh->areas;
		params.polyFlags = genCtx.pmesh->flags;
		params.polyCount = genCtx.pmesh->npolys;
		params.nvp = genCtx.pmesh->nvp;
		params.detailMeshes = genCtx.dmesh->meshes;
		params.detailVerts = genCtx.dmesh->verts;
		params.detailVertsCount = genCtx.dmesh->nverts;
		params.detailTris = genCtx.dmesh->tris;
		params.detailTriCount = genCtx.dmesh->ntris;
        params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
        params.offMeshConRad = m_geom->getOffMeshConnectionRads();
        params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
        params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
        params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
        params.offMeshConUserID = m_geom->getOffMeshConnectionId();
        params.offMeshConCount = m_geom->getOffMeshConnectionCount();
        params.walkableHeight = m_agentHeight;
        params.walkableRadius = m_agentRadius;
        params.walkableClimb = m_agentMaxClimb;
        params.tileX = tx;
        params.tileY = ty;
        params.tileLayer = 0;
		rcVcopy(params.bmin, genCtx.pmesh->bmin);
		rcVcopy(params.bmax, genCtx.pmesh->bmax);
		params.cs = genCtx.cfg.cs;
		params.ch = genCtx.cfg.ch;
        params.buildBvTree = true;

        if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
        {
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build Detour navmesh");
            return 22;
        }
    }
	saveMaxAtomicInt(m_tileMemUsage, navDataSize);
    m_ctx->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats
	if (!m_asyncNavMeshGeneration) {
		duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
		m_ctx->log(
			RC_LOG_PROGRESS,
			">> Polymesh: %d vertices  %d polygons",
			genCtx.pmesh->nverts, genCtx.pmesh->npolys
		);
	}

	int timeValMs = static_cast<int>(std::ceil(m_ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.f));
	saveMaxAtomicInt(m_tileBuildTime, timeValMs);
    dataSize = navDataSize;
	data = navData;
	return 0;
}

void Sample_TileMesh::saveAll(const char* path, const dtNavMesh* mesh)
{
	if (!mesh)
		return;
	FILE* fp = fopen(path, "wb");
	if (!fp)
		return;
	fwrite(&m_cellSize, sizeof(m_cellSize), 1, fp);
	fwrite(&m_cellHeight, sizeof(m_cellHeight), 1, fp);
	fwrite(&m_agentHeight, sizeof(m_agentHeight), 1, fp);
	fwrite(&m_agentLiquidWalk, sizeof(m_agentLiquidWalk), 1, fp);
	fwrite(&m_agentLiquidFord, sizeof(m_agentLiquidFord), 1, fp);
	fwrite(&m_agentLiquidSwim, sizeof(m_agentLiquidSwim), 1, fp);
	fwrite(&m_agentRadius, sizeof(m_agentRadius), 1, fp);
	fwrite(&m_agentMaxClimb, sizeof(m_agentMaxClimb), 1, fp);
	fwrite(&m_agentMaxSlope, sizeof(m_agentMaxSlope), 1, fp);
	fwrite(&m_regionMinSize, sizeof(m_regionMinSize), 1, fp);
	fwrite(&m_regionMergeSize, sizeof(m_regionMergeSize), 1, fp);
	fwrite(&m_edgeMaxLen, sizeof(m_edgeMaxLen), 1, fp);
	fwrite(&m_edgeMaxError, sizeof(m_edgeMaxError), 1, fp);
	fwrite(&m_vertsPerPoly, sizeof(m_vertsPerPoly), 1, fp);
	fwrite(&m_detailSampleDist, sizeof(m_detailSampleDist), 1, fp);
	fwrite(&m_detailSampleMaxError, sizeof(m_detailSampleMaxError), 1, fp);
	fwrite(&m_partitionType, sizeof(m_partitionType), 1, fp);
	fwrite(&m_filterLowHangingObstacles, sizeof(m_filterLowHangingObstacles), 1, fp);
	fwrite(&m_filterLedgeSpans, sizeof(m_filterLedgeSpans), 1, fp);
	fwrite(&m_filterWalkableLowHeightSpans, sizeof(m_filterWalkableLowHeightSpans), 1, fp);
	fwrite(&m_tileSize, sizeof(m_tileSize), 1, fp);
	Sample::saveAll(fp, mesh);
	fclose(fp);
}

dtNavMesh* Sample_TileMesh::loadAll(const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp)
		return 0;
	fread(&m_cellSize, sizeof(m_cellSize), 1, fp);
	fread(&m_cellHeight, sizeof(m_cellHeight), 1, fp);
	fread(&m_agentHeight, sizeof(m_agentHeight), 1, fp);
	fread(&m_agentLiquidWalk, sizeof(m_agentLiquidWalk), 1, fp);
	fread(&m_agentLiquidFord, sizeof(m_agentLiquidFord), 1, fp);
	fread(&m_agentLiquidSwim, sizeof(m_agentLiquidSwim), 1, fp);
	fread(&m_agentRadius, sizeof(m_agentRadius), 1, fp);
	fread(&m_agentMaxClimb, sizeof(m_agentMaxClimb), 1, fp);
	fread(&m_agentMaxSlope, sizeof(m_agentMaxSlope), 1, fp);
	fread(&m_regionMinSize, sizeof(m_regionMinSize), 1, fp);
	fread(&m_regionMergeSize, sizeof(m_regionMergeSize), 1, fp);
	fread(&m_edgeMaxLen, sizeof(m_edgeMaxLen), 1, fp);
	fread(&m_edgeMaxError, sizeof(m_edgeMaxError), 1, fp);
	fread(&m_vertsPerPoly, sizeof(m_vertsPerPoly), 1, fp);
	fread(&m_detailSampleDist, sizeof(m_detailSampleDist), 1, fp);
	fread(&m_detailSampleMaxError, sizeof(m_detailSampleMaxError), 1, fp);
	fread(&m_partitionType, sizeof(m_partitionType), 1, fp);
	fread(&m_filterLowHangingObstacles, sizeof(m_filterLowHangingObstacles), 1, fp);
	fread(&m_filterLedgeSpans, sizeof(m_filterLedgeSpans), 1, fp);
	fread(&m_filterWalkableLowHeightSpans, sizeof(m_filterWalkableLowHeightSpans), 1, fp);
	fread(&m_tileSize, sizeof(m_tileSize), 1, fp);
	dtNavMesh* mesh = Sample::loadAll(fp);
	fclose(fp);
	return mesh;
}
