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
#include <stdlib.h>
#include <string.h>
#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif
#include "imgui.h"
#include "NavMeshTesterTool.h"
#include "Sample.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "DetourCommon.h"

#include <algorithm>
#include <chrono>
#include <string>
#include <sstream>

#ifdef WIN32
#	define snprintf _snprintf
#endif

// Uncomment this to dump all the requests in stdout.
#define DUMP_REQS

// Returns a random number [0..1]
static float frand()
{
//	return ((float)(rand() & 0xffff)/(float)0xffff);
	return (float)rand()/(float)RAND_MAX;
}

inline bool inRange(const float* v1, const float* v2, const float r, const float h)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return (dx*dx + dz*dz) < r*r && fabsf(dy) < h;
}

static void calcPolyCenter(const dtMeshTile* tile, const dtPoly* poly, float* center)
{
    center[0] = center[1] = center[2] = 0.f;
    const int n = (int)poly->vertCount;
    for (int i = 0; i < n; ++i)
    {
        const float* v = &tile->verts[poly->verts[i] * 3];
        center[0] += v[0];
        center[1] += v[1];
        center[2] += v[2];
    }
    const float mul = 1.0f / n;
    center[0] *= mul;
    center[1] *= mul;
    center[2] *= mul;
}

static int fixupCorridor(dtPolyRef* path, const int npath, const int maxPath,
						 const dtPolyRef* visited, const int nvisited)
{
	int furthestPath = -1;
	int furthestVisited = -1;
	
	// Find furthest common polygon.
	for (int i = npath-1; i >= 0; --i)
	{
		bool found = false;
		for (int j = nvisited-1; j >= 0; --j)
		{
			if (path[i] == visited[j])
			{
				furthestPath = i;
				furthestVisited = j;
				found = true;
			}
		}
		if (found)
			break;
	}

	// If no intersection found just return current path. 
	if (furthestPath == -1 || furthestVisited == -1)
		return npath;
	
	// Concatenate paths.	

	// Adjust beginning of the buffer to include the visited.
	const int req = nvisited - furthestVisited;
	const int orig = rcMin(furthestPath+1, npath);
	int size = rcMax(0, npath-orig);
	if (req+size > maxPath)
		size = maxPath-req;
	if (size)
		memmove(path+req, path+orig, size*sizeof(dtPolyRef));
	
	// Store visited
	for (int i = 0; i < req; ++i)
		path[i] = visited[(nvisited-1)-i];				
	
	return req+size;
}

// This function checks if the path has a small U-turn, that is,
// a polygon further in the path is adjacent to the first polygon
// in the path. If that happens, a shortcut is taken.
// This can happen if the target (T) location is at tile boundary,
// and we're (S) approaching it parallel to the tile edge.
// The choice at the vertex can be arbitrary, 
//  +---+---+
//  |:::|:::|
//  +-S-+-T-+
//  |:::|   | <-- the step can end up in here, resulting U-turn path.
//  +---+---+
static int fixupShortcuts(dtPolyRef* path, int npath, dtNavMeshQuery* navQuery)
{
	if (npath < 3)
		return npath;

	// Get connected polygons
	static const int maxNeis = 16;
	dtPolyRef neis[maxNeis];
	int nneis = 0;

	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (dtStatusFailed(navQuery->getAttachedNavMesh()->getTileAndPolyByRef(path[0], &tile, &poly)))
		return npath;
	
	for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
	{
		const dtLink* link = &tile->links[k];
		if (link->ref != 0)
		{
			if (nneis < maxNeis)
				neis[nneis++] = link->ref;
		}
	}

	// If any of the neighbour polygons is within the next few polygons
	// in the path, short cut to that polygon directly.
	static const int maxLookAhead = 6;
	int cut = 0;
	for (int i = dtMin(maxLookAhead, npath) - 1; i > 1 && cut == 0; i--) {
		for (int j = 0; j < nneis; j++)
		{
			if (path[i] == neis[j]) {
				cut = i;
				break;
			}
		}
	}
	if (cut > 1)
	{
		int offset = cut-1;
		npath -= offset;
		for (int i = 1; i < npath; i++)
			path[i] = path[i+offset];
	}

	return npath;
}

static bool getSteerTarget(dtNavMeshQuery* navQuery, const float* startPos, const float* endPos,
						   const float minTargetDist,
						   const dtPolyRef* path, const int pathSize,
						   float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef,
						   float* outPoints = 0, int* outPointCount = 0)							 
{
	// Find steer target.
	static const int MAX_STEER_POINTS = 3;
	float steerPath[MAX_STEER_POINTS*3];
	unsigned char steerPathFlags[MAX_STEER_POINTS];
	dtPolyRef steerPathPolys[MAX_STEER_POINTS];
	int nsteerPath = 0;
	navQuery->findStraightPath(startPos, endPos, path, pathSize,
							   steerPath, steerPathFlags, steerPathPolys, &nsteerPath, MAX_STEER_POINTS);
	if (!nsteerPath)
		return false;
		
	if (outPoints && outPointCount)
	{
		*outPointCount = nsteerPath;
		for (int i = 0; i < nsteerPath; ++i)
			dtVcopy(&outPoints[i*3], &steerPath[i*3]);
	}

	
	// Find vertex far enough to steer to.
	int ns = 0;
	while (ns < nsteerPath)
	{
		// Stop at Off-Mesh link or when point is further than slop away.
		if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
			!inRange(&steerPath[ns*3], startPos, minTargetDist, 1000.0f))
			break;
		ns++;
	}
	// Failed to find good point to steer to.
	if (ns >= nsteerPath)
		return false;
	
	dtVcopy(steerPos, &steerPath[ns*3]);
	steerPos[1] = startPos[1];
	steerPosFlag = steerPathFlags[ns];
	steerPosRef = steerPathPolys[ns];
	
	return true;
}

// ======================================================

//void collisionDetectorGeometryWorld::calcObbPoints(const geometry::OBB* b, float* obbPoints)
//{
//    // forward, left, up
//    rcVadd(obbPoints, b->center, b->dir);
//    Vadd(obbPoints, b->dir + 3);
//    Vadd(obbPoints, b->dir + 6);
//
//    rcVadd(obbPoints + 3, b->center, b->dir);
//    Vadd(obbPoints + 3, b->dir + 3);
//    Vsub(obbPoints + 3, b->dir + 6);
//
//    rcVadd(obbPoints + 6, b->center, b->dir);
//    Vsub(obbPoints + 6, b->dir + 3);
//    Vadd(obbPoints + 6, b->dir + 6);
//
//    rcVadd(obbPoints + 9, b->center, b->dir);
//    Vsub(obbPoints + 9, b->dir + 3);
//    Vsub(obbPoints + 9, b->dir + 6);
//
//    rcVsub(obbPoints + 12, b->center, b->dir);
//    Vadd(obbPoints + 12, b->dir + 3);
//    Vadd(obbPoints + 12, b->dir + 6);
//
//    rcVsub(obbPoints + 15, b->center, b->dir);
//    Vadd(obbPoints + 15, b->dir + 3);
//    Vsub(obbPoints + 15, b->dir + 6);
//
//    rcVsub(obbPoints + 18, b->center, b->dir);
//    Vsub(obbPoints + 18, b->dir + 3);
//    Vadd(obbPoints + 18, b->dir + 6);
//
//    rcVsub(obbPoints + 21, b->center, b->dir);
//    Vsub(obbPoints + 21, b->dir + 3);
//    Vsub(obbPoints + 21, b->dir + 6);
//}
//
//void collisionDetectorGeometryWorld::calcObbExt(const float* src,
//    const float* dst,
//    const float deltaH,
//    const float halfWidth,
//    const float scale,
//    geometry::OBBExt* be
//) {
//    rcVsub(be->b.dir + 0, dst, src);
//    Vmul(be->b.dir + 0, 0.5f);
//    (be->b.dir + 3)[0] = be->b.dir[2];
//    (be->b.dir + 3)[1] = 0.f;
//    (be->b.dir + 3)[2] = -1.f * be->b.dir[0];
//    rcVnormalize(be->b.dir + 3);
//    Vmul(be->b.dir + 3, halfWidth);
//    (be->b.dir + 6)[0] = 0.f;
//    (be->b.dir + 6)[1] = deltaH;
//    (be->b.dir + 6)[2] = 0.f;
//    rcVadd(be->b.center, src, be->b.dir);
//    rcVadd(be->b.center, be->b.center, be->b.dir + 6);
//    Vmul(be->b.dir + 3, scale); // 0.95 ?
//    Vmul(be->b.dir + 6, scale); // 0.95 ?
//    be->b.halfWidth[0] = Vlen(be->b.dir + 0);
//    be->b.halfWidth[1] = deltaH;
//    be->b.halfWidth[2] = Vlen(be->b.dir + 3);
//    calcObbPoints(&be->b, be->verts);
//}
//
//bool collisionDetectorGeometryWorld::detectJumpForwardCollisions(
//    const float* src, const float* dst, float deltaH
//) const {
//    static const float HALF_WIDTH = 30.f;
//    if (!m_active)
//        return false;
//    ++m_callsNum;
//    float tmin{};
//    float start[3] = {src[0], src[1] + deltaH * 0.5f, src[2]};
//    float end[3] = {dst[0], dst[1] + deltaH * 0.5f, dst[2]};
//    bool res = m_inGeom->raycastMesh(start, end, tmin);
//    if (res) return true;
//    geometry::OBBExt be;
//    calcObbExt(src, dst, deltaH, HALF_WIDTH, 0.95f, &be);
//    return m_inGeom->obbCollDetect(&be);
//}
//
//bool collisionDetectorGeometryWorld::detectClimbCollisions(
//    const float* src, const float* dst, float xzDist, float deltaH
//) const {
//    static const float HALF_WIDTH = 30.f;
//    if (!m_active)
//        return false;
//    ++m_callsNum;
//    float dir[3] = {src[0] - dst[0], 0, src[2] - dst[2]};
//    //rcVsub(dir, src, dst);
//    dtVnormalize(dir);
//    float start[3] = {dst[0], dst[1] + 5.f,/*deltaH * 0.5f,*/ dst[2]};
//    float mid[3] = {
//        dst[0] + dir[0] * xzDist,
//        dst[1] + 5.f,//deltaH * 0.5f,
//        dst[2] + dir[2] * xzDist
//    };
//    float tmin{};
//    bool res = m_inGeom->raycastMesh(start, mid, tmin);
//    if (res) return true;
//    float end[3] = {src[0], src[1] + 5.f,/*deltaH * 0.5f,*/ src[2]};
//    res = m_inGeom->raycastMesh(mid, end, tmin);
//    if (res) return true;
//    geometry::OBBExt be;
//    calcObbExt(start, mid, deltaH, HALF_WIDTH, 0.95f, &be);
//    res = m_inGeom->obbCollDetect(&be);
//    if (res) return true;
//    calcObbExt(mid, end, deltaH, HALF_WIDTH, 0.95f, &be);
//    return m_inGeom->obbCollDetect(&be);
//
//}
//
//bool collisionDetectorGeometryWorld::detectJumpDownCollisions(
//    const float* src, const float* dst, float deltaH
//) const {
//    static const float HALF_WIDTH = 30.f;
//    if (!m_active)
//        return false;
//    ++m_callsNum;
//    float dir[3] = {dst[0] - src[0], 0, dst[2] - src[2]};
//    //rcVsub(dir, dst, src);
//    dtVnormalize(dir);
//    float start[3] = {src[0], src[1] + 5.f,/*deltaH * 0.5f,*/ src[2]};
//    float mid[3] = {
//        src[0] + dir[0] * 100.f,
//        src[1] + 5.f,//deltaH * 0.5f,
//        src[2] + dir[2] * 100.f
//    };
//    float tmin{};
//    bool res = m_inGeom->raycastMesh(start, mid, tmin);
//    if (res) return true;
//    float end[3] = {dst[0], dst[1] + 5.f,/*deltaH * 0.5f,*/dst[2]};
//    res = m_inGeom->raycastMesh(mid, end, tmin);
//    if (res) return true;
//    geometry::OBBExt be;
//    calcObbExt(start, mid, deltaH, HALF_WIDTH, 1.f, &be);
//    res = m_inGeom->obbCollDetect(&be);
//    if (res) return true;
//    calcObbExt(mid, end, deltaH, HALF_WIDTH, 1.f, &be);
//    return m_inGeom->obbCollDetect(&be);
//}
//
//bool collisionDetectorGeometryWorld::detectCollision(const float* src, const float* dst) const {
//    if (!m_active)
//        return false;
//    ++m_callsNum;
//    float tmin{};
//    float start[3] = {src[0], src[1], src[2]};
//    float end[3] = {dst[0], dst[1], dst[2]};
//    return m_inGeom->raycastMesh(start, end, tmin);
//}

// ======================================================

static bool canTransfer(const uint8_t from, const uint8_t to)
{
    switch(from)
    {
		case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER: {
			//assert(1 != 1);
			//return false; // technical flag of water, normaly we can't meet it at this stage
			return true;
		}
		case common::SamplePolyAreas::SAMPLE_POLYAREA_GROUND:
		case common::SamplePolyAreas::SAMPLE_POLYAREA_ROAD:
		case common::SamplePolyAreas::SAMPLE_POLYAREA_FOREST:
		case common::SamplePolyAreas::SAMPLE_POLYAREA_DOOR:
		case common::SamplePolyAreas::SAMPLE_POLYAREA_LADDER:
		case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_WALKING: {
			return true;
		}
		case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_FORDING: {
			switch(to)
			{
				case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_WALKING:
				case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_FORDING:
				case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_SWIMMING:
					return true;
				default:
					return false;
			}
		}
		case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_SWIMMING: {
			switch(to)
			{
				case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_FORDING:
				case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_SWIMMING:
					return true;
				default:
					return false;
			}
		}
		default: {
			assert(1 != 1);
			return false;
		}
    }
}

static bool canTransferJumping(const int /*jumpingType*/, const uint8_t /*from*/, const uint8_t /*to*/)
{
	return false;
	//  switch(jumpingType)
  //  {
		//case JUMP_DOWN:
		//case JUMP_FORWARD: {
		//	switch(from)
		//	{
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER: {
		//			//assert(1 != 1);
		//			// technical flag of water, normaly we can't meet it at this stage
		//			return false;
		//		}
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_GROUND:
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_ROAD:
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_FOREST:
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_DOOR:
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_LADDER:
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_WALKING: {
		//			return true;
		//		}
		//		default: {
		//			return false;
		//		}
		//	}
		//}
		//case CLIMB:
		//case CLIMB_OVERLAPPED_POLYS: {
		//	switch(from)
		//	{
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER: {
		//			//assert(1 != 1);
		//			return false; // technical flag of water, normaly we can't meet it at this stage
		//		}
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_GROUND:
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_ROAD:
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_FOREST:
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_DOOR:
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_LADDER:
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_WALKING:
		//		case common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_FORDING: {
		//			return to == common::SamplePolyAreas::SAMPLE_POLYAREA_GROUND;
		//		}
		//		default: {
		//			return false;
		//		}
		//	}
		//}
		//default: {
		//	assert(1 != 1);
		//	return false;
		//}
  //  }
}

NavMeshTesterTool::NavMeshTesterTool(InputGeom *inGeom, BuildContext* ctx):
	m_ctx(ctx),
    m_sample(0),
	m_navMesh(0),
	m_navQuery(0),
	m_pathFindStatus(DT_FAILURE),
	m_toolMode(TOOLMODE_PATHFIND_FOLLOW),
	m_straightPathOptions(0),
	m_displayRefIdsInPath(false),
    m_straightWithJumpsPathOptions(false),
    //m_collDet(inGeom, m_straightWithJumpsPathOptions),
	m_startRef(0),
	m_endRef(0),
	m_npolys(0),
	m_nstraightPath(0),
	m_nsmoothPath(0),
	m_nrandPoints(0),
	m_randPointsInCircle(false),
	m_hitResult(false),
	m_distanceToWall(0),
	m_sposSet(false),
	m_eposSet(false),
	m_pathIterNum(0),
	m_pathIterPolyCount(0),
    m_steerPointCount(0)
{
	m_filter.setIncludeFlags(common::SamplePolyFlags::SAMPLE_POLYFLAGS_ALL ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_DISABLED);
	m_filter.setExcludeFlags(0);

	m_polyPickExt[0] = 2;
    m_polyPickExt[1] = POLY_PICK_Y;//4;
	m_polyPickExt[2] = 2;
	
	m_neighbourhoodRadius = 2.5f;
	m_randomRadius = 5.0f;

    //m_filter.setCanTransfer(&canTransfer);
    //m_filter.setCanTransferJumping(&canTransferJumping);
}

NavMeshTesterTool::~NavMeshTesterTool() {
    //clearPolyRawDataHolder<StdPolyRawDataHolder>();
    //clearNonConnectedTransfersHolder<TransfersCache>();
}

void NavMeshTesterTool::init(Sample* sample)
{
    m_sample = sample;
	m_navMesh = sample->getNavMesh();
	m_navQuery = sample->getNavMeshQuery();
	recalc();

	if (m_navQuery)
	{
		// Change costs.
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_GROUND, 1.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_ROAD, 1.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_DOOR, 1.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_LADDER, 1.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_WATER, 5.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_WALKING, 1.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_FORDING, 3.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_SWIMMING, 5.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_LAVA, 100000.0f);
	}
	
	m_neighbourhoodRadius = sample->getAgentRadius() * 20.0f;
	m_randomRadius = sample->getAgentRadius() * 30.0f;

    //if (!initPathfindingSubsystem())
    //    m_initError = true;
}

void NavMeshTesterTool::init(dtNavMesh* navMesh, dtNavMeshQuery* navQuery)
{
    m_sample = nullptr;
    m_navMesh = navMesh;
    m_navQuery = navQuery;

    if (m_navQuery)
    {
        // Change costs.
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_GROUND, 1.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_ROAD, 1.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_DOOR, 1.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_LADDER, 1.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_WATER, 5.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_WALKING, 1.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_FORDING, 3.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_WATER_SWIMMING, 5.0f);
        m_filter.setAreaCost(common::SamplePolyAreas::SAMPLE_POLYAREA_LAVA, 100000.0f);
    }

    m_neighbourhoodRadius = 1.f;
    m_randomRadius = 1.f;

    //if (!initPathfindingSubsystem())
    //    m_initError = true;
}

void NavMeshTesterTool::handleMenu()
{
	if (imguiCheck("Pathfind Follow", m_toolMode == TOOLMODE_PATHFIND_FOLLOW))
	{
		m_toolMode = TOOLMODE_PATHFIND_FOLLOW;
		recalc();
	}
	if (imguiCheck("Pathfind Straight", m_toolMode == TOOLMODE_PATHFIND_STRAIGHT))
	{
		m_toolMode = TOOLMODE_PATHFIND_STRAIGHT;
		recalc();
	}
	if (m_toolMode == TOOLMODE_PATHFIND_STRAIGHT)
	{
		imguiIndent();
		imguiLabel("Vertices at crossings");
		if (imguiCheck("None", m_straightPathOptions == 0))
		{
			m_straightPathOptions = 0;
			recalc();
		}
		if (imguiCheck("Area", m_straightPathOptions == DT_STRAIGHTPATH_AREA_CROSSINGS))
		{
			m_straightPathOptions = DT_STRAIGHTPATH_AREA_CROSSINGS;
			recalc();
		}
		if (imguiCheck("All", m_straightPathOptions == DT_STRAIGHTPATH_ALL_CROSSINGS))
		{
			m_straightPathOptions = DT_STRAIGHTPATH_ALL_CROSSINGS;
			recalc();
		}

		imguiUnindent();
	}
    //if (imguiCheck(
    //    "Pathfind Straight with jumps",
    //    m_toolMode == TOOLMODE_PATHFIND_STRAIGHT_WITH_JUMPS
    //)) {
    //    m_collDet.disable();
    //    m_toolMode = TOOLMODE_PATHFIND_STRAIGHT_WITH_JUMPS;
    //    recalc();
    //}
    //if (m_toolMode == TOOLMODE_PATHFIND_STRAIGHT_WITH_JUMPS)
    //{
    //    imguiIndent();
    //    imguiLabel("Use world geometry collision detections");
    //    if (imguiCheck("No", m_straightWithJumpsPathOptions == 0))
    //    {
    //        m_straightWithJumpsPathOptions = 0;
    //        m_collDet.disable();
    //        m_toolMode = TOOLMODE_PATHFIND_STRAIGHT_WITH_JUMPS;
    //        recalc();
    //    }
    //    if (imguiCheck("Yes", m_straightWithJumpsPathOptions == 1))
    //    {
    //        m_straightWithJumpsPathOptions = 1;
    //        m_collDet.enable();
    //        m_toolMode = TOOLMODE_PATHFIND_STRAIGHT_WITH_JUMPS;
    //        recalc();
    //    }
    //    imguiUnindent();
    //}
	if (imguiCheck("Pathfind Sliced", m_toolMode == TOOLMODE_PATHFIND_SLICED))
	{
		m_toolMode = TOOLMODE_PATHFIND_SLICED;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Distance to Wall", m_toolMode == TOOLMODE_DISTANCE_TO_WALL))
	{
		m_toolMode = TOOLMODE_DISTANCE_TO_WALL;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Raycast", m_toolMode == TOOLMODE_RAYCAST))
	{
		m_toolMode = TOOLMODE_RAYCAST;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Find Polys in Circle", m_toolMode == TOOLMODE_FIND_POLYS_IN_CIRCLE))
	{
		m_toolMode = TOOLMODE_FIND_POLYS_IN_CIRCLE;
		recalc();
	}
	if (imguiCheck("Find Polys in Shape", m_toolMode == TOOLMODE_FIND_POLYS_IN_SHAPE))
	{
		m_toolMode = TOOLMODE_FIND_POLYS_IN_SHAPE;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Find Local Neighbourhood", m_toolMode == TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD))
	{
		m_toolMode = TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Display path ref ids", m_displayRefIdsInPath))
	{
		m_displayRefIdsInPath = !m_displayRefIdsInPath;
	}

	imguiSeparator();
	
	if (imguiButton("Set Random Start"))
	{
		dtStatus status = m_navQuery->findRandomPoint(&m_filter, frand, &m_startRef, m_spos);
		if (dtStatusSucceed(status))
		{
			m_sposSet = true;
			recalc();
		}
	}
	if (imguiButton("Set Random End", m_sposSet))
	{
		if (m_sposSet)
		{
			dtStatus status = m_navQuery->findRandomPointAroundCircle(m_startRef, m_spos, m_randomRadius, &m_filter, frand, &m_endRef, m_epos);
			if (dtStatusSucceed(status))
			{
				m_eposSet = true;
				recalc();
			}
		}
	}

	imguiSeparator();

	if (imguiButton("Make Random Points"))
	{
		m_randPointsInCircle = false;
		m_nrandPoints = 0;
		for (int i = 0; i < MAX_RAND_POINTS; i++)
		{
			float pt[3];
			dtPolyRef ref;
			dtStatus status = m_navQuery->findRandomPoint(&m_filter, frand, &ref, pt);
			if (dtStatusSucceed(status))
			{
				dtVcopy(&m_randPoints[m_nrandPoints*3], pt);
				m_nrandPoints++;
			}
		}
	}
	if (imguiButton("Make Random Points Around", m_sposSet))
	{
		if (m_sposSet)
		{
			m_nrandPoints = 0;
			m_randPointsInCircle = true;
			for (int i = 0; i < MAX_RAND_POINTS; i++)
			{
				float pt[3];
				dtPolyRef ref;
				dtStatus status = m_navQuery->findRandomPointAroundCircle(m_startRef, m_spos, m_randomRadius, &m_filter, frand, &ref, pt);
				if (dtStatusSucceed(status))
				{
					dtVcopy(&m_randPoints[m_nrandPoints*3], pt);
					m_nrandPoints++;
				}
			}
		}
	}

	
	imguiSeparator();

    imguiLabel("Include Flags");

	imguiIndent();
	if (imguiCheck("Walk", (m_filter.getIncludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_WALK) != 0))
	{
		m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_WALK);
		recalc();
	}
    if (imguiCheck("Water walking", (m_filter.getIncludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_WATER_WALKING) != 0))
	{
        m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_WATER_WALKING);
		recalc();
	}
    if (imguiCheck("Water fording", (m_filter.getIncludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_WATER_FORDING) != 0))
    {
        m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_WATER_FORDING);
        recalc();
    }
    if (imguiCheck("Water swimming", (m_filter.getIncludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_WATER_SWIMMING) != 0))
    {
        m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_WATER_SWIMMING);
        recalc();
    }
    if (imguiCheck("Road", (m_filter.getIncludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_ROAD) != 0))
    {
        m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_ROAD);
        recalc();
    }
    if (imguiCheck("Forest", (m_filter.getIncludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_FOREST) != 0))
    {
        m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_FOREST);
        recalc();
    }
	if (imguiCheck("Door", (m_filter.getIncludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_DOOR) != 0))
	{
		m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_DOOR);
		recalc();
	}
    if (imguiCheck("Ladder", (m_filter.getIncludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_LADDER) != 0))
    {
        m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_LADDER);
        recalc();
    }
	imguiUnindent();

	imguiSeparator();
	imguiLabel("Exclude Flags");
	
    imguiIndent();
    if (imguiCheck("Walk", (m_filter.getExcludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_WALK) != 0))
    {
        m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_WALK);
        recalc();
    }
    if (imguiCheck("Water walking", (m_filter.getExcludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_WATER_WALKING) != 0))
    {
        m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_WATER_WALKING);
        recalc();
    }
    if (imguiCheck("Water fording", (m_filter.getExcludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_WATER_FORDING) != 0))
    {
        m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_WATER_FORDING);
        recalc();
    }
    if (imguiCheck("Water swimming", (m_filter.getExcludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_WATER_SWIMMING) != 0))
    {
        m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_WATER_SWIMMING);
        recalc();
    }
    if (imguiCheck("Road", (m_filter.getExcludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_ROAD) != 0))
    {
        m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_ROAD);
        recalc();
    }
    if (imguiCheck("Forest", (m_filter.getExcludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_FOREST) != 0))
    {
        m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_FOREST);
        recalc();
    }
    if (imguiCheck("Door", (m_filter.getExcludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_DOOR) != 0))
    {
        m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_DOOR);
        recalc();
    }
    if (imguiCheck("Ladder", (m_filter.getExcludeFlags() & common::SamplePolyFlags::SAMPLE_POLYFLAGS_LADDER) != 0))
    {
        m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ common::SamplePolyFlags::SAMPLE_POLYFLAGS_LADDER);
        recalc();
    }
	imguiUnindent();

	imguiSeparator();	
}

void NavMeshTesterTool::handleClick(const float* /*s*/, const float* p, bool shift)
{
	if (shift)
	{
		m_sposSet = true;
		dtVcopy(m_spos, p);
	}
	else
	{
		m_eposSet = true;
		dtVcopy(m_epos, p);
	}
	recalc();
}

void NavMeshTesterTool::handleStep()
{
}

void NavMeshTesterTool::handleToggle()
{
	// TODO: merge separate to a path iterator. Use same code in recalc() too.
	if (m_toolMode != TOOLMODE_PATHFIND_FOLLOW)
		return;
		
	if (!m_sposSet || !m_eposSet || !m_startRef || !m_endRef)
		return;
		
	static const float STEP_SIZE = 0.5f;
	static const float SLOP = 0.01f;

	if (m_pathIterNum == 0)
	{
		m_navQuery->findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, m_polys, &m_npolys, MAX_POLYS);
		m_nsmoothPath = 0;

		m_pathIterPolyCount = m_npolys;
		if (m_pathIterPolyCount)
			memcpy(m_pathIterPolys, m_polys, sizeof(dtPolyRef)*m_pathIterPolyCount); 
		
		if (m_pathIterPolyCount)
		{
			// Iterate over the path to find smooth path on the detail mesh surface.
			m_navQuery->closestPointOnPoly(m_startRef, m_spos, m_iterPos, 0);
			m_navQuery->closestPointOnPoly(m_pathIterPolys[m_pathIterPolyCount-1], m_epos, m_targetPos, 0);
			
			m_nsmoothPath = 0;
			
			dtVcopy(&m_smoothPath[m_nsmoothPath*3], m_iterPos);
			m_nsmoothPath++;
		}
	}
	
	dtVcopy(m_prevIterPos, m_iterPos);

	m_pathIterNum++;

	if (!m_pathIterPolyCount)
		return;

	if (m_nsmoothPath >= MAX_SMOOTH)
		return;

	// Move towards target a small advancement at a time until target reached or
	// when ran out of memory to store the path.

	// Find location to steer towards.
	float steerPos[3];
	unsigned char steerPosFlag;
	dtPolyRef steerPosRef;
		
	if (!getSteerTarget(m_navQuery, m_iterPos, m_targetPos, SLOP,
						m_pathIterPolys, m_pathIterPolyCount, steerPos, steerPosFlag, steerPosRef,
						m_steerPoints, &m_steerPointCount))
		return;
		
	dtVcopy(m_steerPos, steerPos);
	
	bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
	bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
		
	// Find movement delta.
	float delta[3], len;
	dtVsub(delta, steerPos, m_iterPos);
	len = sqrtf(dtVdot(delta,delta));
	// If the steer target is end of path or off-mesh link, do not move past the location.
	if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
		len = 1;
	else
		len = STEP_SIZE / len;
	float moveTgt[3];
	dtVmad(moveTgt, m_iterPos, delta, len);
		
	// Move
	float result[3];
	dtPolyRef visited[16];
	int nvisited = 0;
	m_navQuery->moveAlongSurface(m_pathIterPolys[0], m_iterPos, moveTgt, &m_filter,
								 result, visited, &nvisited, 16);
	m_pathIterPolyCount = fixupCorridor(m_pathIterPolys, m_pathIterPolyCount, MAX_POLYS, visited, nvisited);
	m_pathIterPolyCount = fixupShortcuts(m_pathIterPolys, m_pathIterPolyCount, m_navQuery);

	float h = 0;
	m_navQuery->getPolyHeight(m_pathIterPolys[0], result, &h);
	result[1] = h;
	dtVcopy(m_iterPos, result);
	
	// Handle end of path and off-mesh links when close enough.
	if (endOfPath && inRange(m_iterPos, steerPos, SLOP, 1.0f))
	{
		// Reached end of path.
		dtVcopy(m_iterPos, m_targetPos);
		if (m_nsmoothPath < MAX_SMOOTH)
		{
			dtVcopy(&m_smoothPath[m_nsmoothPath*3], m_iterPos);
			m_nsmoothPath++;
		}
		return;
	}
	else if (offMeshConnection && inRange(m_iterPos, steerPos, SLOP, 1.0f))
	{
		// Reached off-mesh connection.
		float startPos[3], endPos[3];
		
		// Advance the path up to and over the off-mesh connection.
		dtPolyRef prevRef = 0, polyRef = m_pathIterPolys[0];
		int npos = 0;
		while (npos < m_pathIterPolyCount && polyRef != steerPosRef)
		{
			prevRef = polyRef;
			polyRef = m_pathIterPolys[npos];
			npos++;
		}
		for (int i = npos; i < m_pathIterPolyCount; ++i)
			m_pathIterPolys[i-npos] = m_pathIterPolys[i];
		m_pathIterPolyCount -= npos;
				
		// Handle the connection.
		dtStatus status = m_navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos);
		if (dtStatusSucceed(status))
		{
			if (m_nsmoothPath < MAX_SMOOTH)
			{
				dtVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
				m_nsmoothPath++;
				// Hack to make the dotted path not visible during off-mesh connection.
				if (m_nsmoothPath & 1)
				{
					dtVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
					m_nsmoothPath++;
				}
			}
			// Move position at the other side of the off-mesh link.
			dtVcopy(m_iterPos, endPos);
			float eh = 0.0f;
			m_navQuery->getPolyHeight(m_pathIterPolys[0], m_iterPos, &eh);
			m_iterPos[1] = eh;
		}
	}
	
	// Store results.
	if (m_nsmoothPath < MAX_SMOOTH)
	{
		dtVcopy(&m_smoothPath[m_nsmoothPath*3], m_iterPos);
		m_nsmoothPath++;
	}

}

void NavMeshTesterTool::handleUpdate(const float /*dt*/)
{
	if (m_toolMode == TOOLMODE_PATHFIND_SLICED)
	{
		if (dtStatusInProgress(m_pathFindStatus))
		{
			m_pathFindStatus = m_navQuery->updateSlicedFindPath(1,0);
		}
		if (dtStatusSucceed(m_pathFindStatus))
		{
			m_navQuery->finalizeSlicedFindPath(m_polys, &m_npolys, MAX_POLYS);
			m_nstraightPath = 0;
			if (m_npolys)
			{
				// In case of partial path, make sure the end point is clamped to the last polygon.
				float epos[3];
				dtVcopy(epos, m_epos);
				if (m_polys[m_npolys-1] != m_endRef)
				m_navQuery->closestPointOnPoly(m_polys[m_npolys-1], m_epos, epos, 0);

				m_navQuery->findStraightPath(m_spos, epos, m_polys, m_npolys,
											 m_straightPath, m_straightPathFlags,
											 m_straightPathPolys, &m_nstraightPath, MAX_POLYS, DT_STRAIGHTPATH_ALL_CROSSINGS);
			}
			 
			m_pathFindStatus = DT_FAILURE;
		}
	}
}

void NavMeshTesterTool::reset()
{
	m_startRef = 0;
	m_endRef = 0;
	m_npolys = 0;
	m_nstraightPath = 0;
	m_nsmoothPath = 0;
	memset(m_hitPos, 0, sizeof(m_hitPos));
	memset(m_hitNormal, 0, sizeof(m_hitNormal));
	m_distanceToWall = 0;
}


void NavMeshTesterTool::recalc()
{
	if (!m_navMesh)
		return;
	
	if (m_sposSet)
		m_navQuery->findNearestPoly(m_spos, m_polyPickExt, &m_filter, &m_startRef, 0);
	else
		m_startRef = 0;
	
	if (m_eposSet)
		m_navQuery->findNearestPoly(m_epos, m_polyPickExt, &m_filter, &m_endRef, 0);
	else
		m_endRef = 0;
	m_ctx->log(RC_LOG_PROGRESS, "Recalc path for start: %llu, end: %llu poly's refs", m_startRef, m_endRef);
	m_pathFindStatus = DT_FAILURE;
	
	if (m_toolMode == TOOLMODE_PATHFIND_FOLLOW)
	{
		m_pathIterNum = 0;
		if (m_sposSet && m_eposSet && m_startRef && m_endRef)
		{
#ifdef DUMP_REQS
			m_ctx->log(RC_LOG_PROGRESS, "pi  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags()); 
#endif
			auto tp1 = std::chrono::steady_clock::now();

			m_navQuery->findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, m_polys, &m_npolys, MAX_POLYS);
			m_nsmoothPath = 0;
			if (m_npolys)
			{
				// Iterate over the path to find smooth path on the detail mesh surface.
				dtPolyRef polys[MAX_POLYS];
				memcpy(polys, m_polys, sizeof(dtPolyRef)*m_npolys); 
				int npolys = m_npolys;
				
				float iterPos[3], targetPos[3];
				m_navQuery->closestPointOnPoly(m_startRef, m_spos, iterPos, 0);
				m_navQuery->closestPointOnPoly(polys[npolys-1], m_epos, targetPos, 0);
				
				static const float STEP_SIZE = 0.5f;
				static const float SLOP = 0.01f;
				
				m_nsmoothPath = 0;
				
				dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
				m_nsmoothPath++;
				
				// Move towards target a small advancement at a time until target reached or
				// when ran out of memory to store the path.
				while (npolys && m_nsmoothPath < MAX_SMOOTH)
				{
					// Find location to steer towards.
					float steerPos[3];
					unsigned char steerPosFlag;
					dtPolyRef steerPosRef;
					
					if (!getSteerTarget(m_navQuery, iterPos, targetPos, SLOP,
										polys, npolys, steerPos, steerPosFlag, steerPosRef))
						break;
					
					bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
					bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
					
					// Find movement delta.
					float delta[3], len;
					dtVsub(delta, steerPos, iterPos);
					len = dtMathSqrtf(dtVdot(delta, delta));
					// If the steer target is end of path or off-mesh link, do not move past the location.
					if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
						len = 1;
					else
						len = STEP_SIZE / len;
					float moveTgt[3];
					dtVmad(moveTgt, iterPos, delta, len);
					
					// Move
					float result[3];
					dtPolyRef visited[16];
					int nvisited = 0;
					m_navQuery->moveAlongSurface(polys[0], iterPos, moveTgt, &m_filter,
												 result, visited, &nvisited, 16);

					npolys = fixupCorridor(polys, npolys, MAX_POLYS, visited, nvisited);
					npolys = fixupShortcuts(polys, npolys, m_navQuery);

					float h = 0;
					m_navQuery->getPolyHeight(polys[0], result, &h);
					result[1] = h;
					dtVcopy(iterPos, result);

					// Handle end of path and off-mesh links when close enough.
					if (endOfPath && inRange(iterPos, steerPos, SLOP, 1.0f))
					{
						// Reached end of path.
						dtVcopy(iterPos, targetPos);
						if (m_nsmoothPath < MAX_SMOOTH)
						{
							dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
							m_nsmoothPath++;
						}
						break;
					}
					else if (offMeshConnection && inRange(iterPos, steerPos, SLOP, 1.0f))
					{
						// Reached off-mesh connection.
						float startPos[3], endPos[3];
						
						// Advance the path up to and over the off-mesh connection.
						dtPolyRef prevRef = 0, polyRef = polys[0];
						int npos = 0;
						while (npos < npolys && polyRef != steerPosRef)
						{
							prevRef = polyRef;
							polyRef = polys[npos];
							npos++;
						}
						for (int i = npos; i < npolys; ++i)
							polys[i-npos] = polys[i];
						npolys -= npos;
						
						// Handle the connection.
						dtStatus status = m_navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos);
						if (dtStatusSucceed(status))
						{
							if (m_nsmoothPath < MAX_SMOOTH)
							{
								dtVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
								m_nsmoothPath++;
								// Hack to make the dotted path not visible during off-mesh connection.
								if (m_nsmoothPath & 1)
								{
									dtVcopy(&m_smoothPath[m_nsmoothPath*3], startPos);
									m_nsmoothPath++;
								}
							}
							// Move position at the other side of the off-mesh link.
							dtVcopy(iterPos, endPos);
							float eh = 0.0f;
							m_navQuery->getPolyHeight(polys[0], iterPos, &eh);
							iterPos[1] = eh;
						}
					}
					
					// Store results.
					if (m_nsmoothPath < MAX_SMOOTH)
					{
						dtVcopy(&m_smoothPath[m_nsmoothPath*3], iterPos);
						m_nsmoothPath++;
					}
				}
			}
			auto tp2 = std::chrono::steady_clock::now();
			uint32_t diffVal = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(tp2 - tp1).count());
			m_ctx->log(RC_LOG_PROGRESS, "Pathfinding follow, time delta microseconds: %u\n", diffVal);
		}
		else
		{
			m_npolys = 0;
			m_nsmoothPath = 0;
		}
	}
	else if (m_toolMode == TOOLMODE_PATHFIND_STRAIGHT)
	{
		if (m_sposSet && m_eposSet && m_startRef && m_endRef)
		{
#ifdef DUMP_REQS
			m_ctx->log(RC_LOG_PROGRESS, "ps  %f,%f,%f %f,%f,%f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags()); 
#endif
            auto tp1 = std::chrono::steady_clock::now();
            m_navQuery->findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, m_polys, &m_npolys, MAX_POLYS);
			m_nstraightPath = 0;
			if (m_npolys)
			{
				// In case of partial path, make sure the end point is clamped to the last polygon.
				float epos[3];
				dtVcopy(epos, m_epos);
				if (m_polys[m_npolys-1] != m_endRef)
					m_navQuery->closestPointOnPoly(m_polys[m_npolys-1], m_epos, epos, 0);
				
				m_navQuery->findStraightPath(m_spos, epos, m_polys, m_npolys,
											 m_straightPath, m_straightPathFlags,
											 m_straightPathPolys, &m_nstraightPath, MAX_POLYS, m_straightPathOptions);
			}
            auto tp2 = std::chrono::steady_clock::now();
			uint32_t diffVal = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(tp2 - tp1).count());
			m_ctx->log(RC_LOG_PROGRESS, "Pathfinding straight, time delta microseconds: %u\n", diffVal);
		}
		else
		{
			m_npolys = 0;
			m_nstraightPath = 0;
		}
	}
    else if (m_toolMode == TOOLMODE_PATHFIND_STRAIGHT_WITH_JUMPS)
    {
    //    if (m_sposSet && m_eposSet && m_startRef && m_endRef)
    //    {
    //        m_ctx->log(RC_LOG_PROGRESS, "Pathfinding with jumps start point: %f, %f, %f; end point: %f, %f, %f",
    //            m_spos[0], m_spos[1], m_spos[2], m_epos[0], m_epos[1], m_epos[2]);
    //        m_ctx->log(RC_LOG_PROGRESS, "Pathfinding with jumps poly start ref: %llu, poly end ref: %llu", m_startRef, m_endRef);
    //        auto tp1 = std::chrono::steady_clock::now();
    //        if (!findPathWithJumps(m_startRef, m_endRef, m_spos, m_epos)) {
    //            m_npolys = 0;
    //            m_nstraightPath = 0;
    //            m_ctx->log(RC_LOG_PROGRESS, "Error of pathfinding with jumps\n");
    //        } else {
    //            auto tp2 = std::chrono::steady_clock::now();
	//			  uint32_t diffVal = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(tp2 - tp1).count());
    //            m_ctx->log(RC_LOG_PROGRESS, "Pathfinding with jumps, time delta microseconds: %u", diffVal);
    //        }
    //    }
    }
	else if (m_toolMode == TOOLMODE_PATHFIND_SLICED)
	{
		if (m_sposSet && m_eposSet && m_startRef && m_endRef)
		{
#ifdef DUMP_REQS
			m_ctx->log(RC_LOG_PROGRESS, "ps  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags()); 
#endif
			m_npolys = 0;
			m_nstraightPath = 0;
			
			m_pathFindStatus = m_navQuery->initSlicedFindPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, DT_FINDPATH_ANY_ANGLE);
		}
		else
		{
			m_npolys = 0;
			m_nstraightPath = 0;
		}
	}
	else if (m_toolMode == TOOLMODE_RAYCAST)
	{
		m_nstraightPath = 0;
		if (m_sposSet && m_eposSet && m_startRef)
		{
#ifdef DUMP_REQS
			m_ctx->log(RC_LOG_PROGRESS, "rc  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], m_epos[0],m_epos[1],m_epos[2],
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags()); 
#endif
			auto tp1 = std::chrono::steady_clock::now();
			
			float t = 0;
			m_npolys = 0;
			m_nstraightPath = 2;
			m_straightPath[0] = m_spos[0];
			m_straightPath[1] = m_spos[1];
			m_straightPath[2] = m_spos[2];
			m_navQuery->raycast(m_startRef, m_spos, m_epos, &m_filter, &t, m_hitNormal, m_polys, &m_npolys, MAX_POLYS);
			if (t > 1)
			{
				// No hit
				dtVcopy(m_hitPos, m_epos);
				m_hitResult = false;
			}
			else
			{
				// Hit
				dtVlerp(m_hitPos, m_spos, m_epos, t);
				m_hitResult = true;
			}
			// Adjust height.
			if (m_npolys > 0)
			{
				float h = 0;
				m_navQuery->getPolyHeight(m_polys[m_npolys-1], m_hitPos, &h);
				m_hitPos[1] = h;
			}
			dtVcopy(&m_straightPath[3], m_hitPos);

			auto tp2 = std::chrono::steady_clock::now();
			uint32_t diffVal = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(tp2 - tp1).count());
			m_ctx->log(RC_LOG_PROGRESS, "Pathfinding raycast, time delta microseconds: %u\n", diffVal);
		}
	}
	else if (m_toolMode == TOOLMODE_DISTANCE_TO_WALL)
	{
		m_distanceToWall = 0;
		if (m_sposSet && m_startRef)
		{
#ifdef DUMP_REQS
			m_ctx->log(RC_LOG_PROGRESS, "dw  %f %f %f  %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], 100.0f,
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags()); 
#endif
			m_distanceToWall = 0.0f;
			m_navQuery->findDistanceToWall(m_startRef, m_spos, 100.0f, &m_filter, &m_distanceToWall, m_hitPos, m_hitNormal);
		}
	}
	else if (m_toolMode == TOOLMODE_FIND_POLYS_IN_CIRCLE)
	{
		if (m_sposSet && m_startRef && m_eposSet)
		{
			const float dx = m_epos[0] - m_spos[0];
			const float dz = m_epos[2] - m_spos[2];
			float dist = sqrtf(dx*dx + dz*dz);
#ifdef DUMP_REQS
			m_ctx->log(RC_LOG_PROGRESS, "fpc  %f %f %f  %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], dist,
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif
			m_navQuery->findPolysAroundCircle(m_startRef, m_spos, dist, &m_filter,
											  m_polys, m_parent, 0, &m_npolys, MAX_POLYS);

		}
	}
	else if (m_toolMode == TOOLMODE_FIND_POLYS_IN_SHAPE)
	{
		if (m_sposSet && m_startRef && m_eposSet)
		{
			const float nx = (m_epos[2] - m_spos[2])*0.25f;
			const float nz = -(m_epos[0] - m_spos[0])*0.25f;
			const float agentHeight = m_sample ? m_sample->getAgentHeight() : 0;

			m_queryPoly[0] = m_spos[0] + nx*1.2f;
			m_queryPoly[1] = m_spos[1] + agentHeight/2;
			m_queryPoly[2] = m_spos[2] + nz*1.2f;

			m_queryPoly[3] = m_spos[0] - nx*1.3f;
			m_queryPoly[4] = m_spos[1] + agentHeight/2;
			m_queryPoly[5] = m_spos[2] - nz*1.3f;

			m_queryPoly[6] = m_epos[0] - nx*0.8f;
			m_queryPoly[7] = m_epos[1] + agentHeight/2;
			m_queryPoly[8] = m_epos[2] - nz*0.8f;

			m_queryPoly[9] = m_epos[0] + nx;
			m_queryPoly[10] = m_epos[1] + agentHeight/2;
			m_queryPoly[11] = m_epos[2] + nz;
			
#ifdef DUMP_REQS
			m_ctx->log(RC_LOG_PROGRESS, "fpp  %f %f %f  %f %f %f  %f %f %f  %f %f %f  0x%x 0x%x\n",
				   m_queryPoly[0],m_queryPoly[1],m_queryPoly[2],
				   m_queryPoly[3],m_queryPoly[4],m_queryPoly[5],
				   m_queryPoly[6],m_queryPoly[7],m_queryPoly[8],
				   m_queryPoly[9],m_queryPoly[10],m_queryPoly[11],
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif
			m_navQuery->findPolysAroundShape(m_startRef, m_queryPoly, 4, &m_filter,
											 m_polys, m_parent, 0, &m_npolys, MAX_POLYS);
		}
	}
	else if (m_toolMode == TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD)
	{
		if (m_sposSet && m_startRef)
		{
#ifdef DUMP_REQS
			m_ctx->log(RC_LOG_PROGRESS, "fln  %f %f %f  %f  0x%x 0x%x\n",
				   m_spos[0],m_spos[1],m_spos[2], m_neighbourhoodRadius,
				   m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif
            m_navQuery->findLocalNeighbourhood(
                        m_startRef, m_spos, m_neighbourhoodRadius, &m_filter,
                        m_polys, m_parent, &m_npolys, MAX_POLYS);
		}
	}
}
/*
bool NavMeshTesterTool::findPathWithJumps(
    dtPolyRef startRef, dtPolyRef endRef, const float* spos, const float* epos
) {
    if (m_initError) {
        m_ctx->log(RC_LOG_PROGRESS, "Can't run findPathWithJumps, error of initialization");
        return false;
    }
    //m_collDet.disable();
    m_navQuery->findPathWithJumps(
        &m_collDet, startRef, endRef, spos, epos, &m_filter, JUMPING_SIZE,
        m_polysJumping, &m_infoPolysJumping, &m_npolys
    );
    if (!m_npolys) {
        m_ctx->log(RC_LOG_PROGRESS, "Error of pathfinding, dtNavMeshQuery::findPathWithJumps");
        return false;
    }
    m_collDet.clearCallsNum();

    float spos_coords[3];
    float epos_coords[3];
    int current_max_size = MAX_POLYS;
    int current_size = 0;
    m_nstraightPath = 0;
    m_npolys = 0;
    for (int i = 0, sz = m_infoPolysJumping.size(); i < sz; ++i)
    {
        InfoPathEntry& info = m_infoPolysJumping[i];
        if (MAX_POLYS - m_npolys <= 0)
            break;
        if (info.normal) {
            auto dat = (const NormalPathEntry*)info.data;
            assert(dat->actionType == NO_ACTION);
            dtPolyRef firstPolyRef = 0, endPolyRef = 0;
            if (i != 0) {
                dtVcopy(spos_coords, m_straightPathJumping[m_nstraightPath - 1].point);
                InfoPathEntry& prev_info = m_infoPolysJumping[i - 1];
                auto prev_dat = ((const JumpingPathEntry*)prev_info.data + prev_info.size - 1);
                firstPolyRef = prev_dat->polyRefTo;
                m_polys[m_npolys] = firstPolyRef;
                if (i + 1 != sz) {
                    auto next_dat = (const JumpingPathEntry*)m_infoPolysJumping[i + 1].data;
                    dtVcopy(epos_coords, next_dat->posFrom);
                    endPolyRef = next_dat->polyRefFrom;
                    m_polys[m_npolys + info.size + 1] = endPolyRef;
                } else {
                    dtVcopy(epos_coords, epos);
                }
            } else {
                dtVcopy(spos_coords, spos);
                if (sz == 1) {
                    dtVcopy(epos_coords, epos);
                } else {
                    auto next_dat = (const JumpingPathEntry*)m_infoPolysJumping[i + 1].data;
                    dtVcopy(epos_coords, next_dat->posFrom);
                    endPolyRef = next_dat->polyRefFrom;
                    m_polys[m_npolys + info.size] = endPolyRef;
                }
            }

            std::transform(
                dat,
                dat + info.size,
                m_polys + m_npolys + static_cast<bool>(firstPolyRef),
                [] (const NormalPathEntry& ref) {return ref.polyRef;}
            );
            if (endPolyRef == m_polys[m_npolys + info.size])
                endPolyRef = 0;
            m_navQuery->findStraightPath(
                spos_coords,
                epos_coords,
                m_polys + m_npolys,
                info.size + static_cast<bool>(firstPolyRef) + static_cast<bool>(endPolyRef),
                m_straightPath + m_nstraightPath * NCOORDS,
                m_straightPathFlags + m_nstraightPath,
                m_straightPathPolys + m_nstraightPath,
                &current_size,
                current_max_size,
                m_straightPathOptions
            );

            for (int j = 0; j < current_size; ++j) {
                m_straightPathJumping[m_nstraightPath + j].actionType = NO_ACTION;
                dtVcopy(
                    m_straightPathJumping[m_nstraightPath + j].point,
                    m_straightPath + (m_nstraightPath + j) * NCOORDS
                );
            }
            current_max_size -= current_size;
            m_nstraightPath += current_size;
            m_npolys += info.size + static_cast<bool>(firstPolyRef) + static_cast<bool>(endPolyRef);
            if (current_max_size < 2)
                break;
        } else {
            bool first = true;
            for (int j = 0; j < info.size; ++j) {
                auto dat = (const JumpingPathEntry*)info.data + j;
                assert(dat->actionType != NO_ACTION);
                if (!first) {
                    m_straightPathJumping[m_nstraightPath].actionType = dat->actionType;
                    dtVcopy(m_straightPathJumping[m_nstraightPath].point, dat->posFrom);
                    ++m_nstraightPath;
                    m_straightPathJumping[m_nstraightPath].actionType = NO_ACTION;
                    dtVcopy(m_straightPathJumping[m_nstraightPath].point, dat->posTo);
                    ++m_nstraightPath;
                    m_polys[m_npolys] = dat->polyRefFrom;
                    ++m_npolys;
                    current_max_size -= 2;
                } else {
                    m_straightPathJumping[m_nstraightPath - 1].actionType = dat->actionType;
                    dtVcopy(m_straightPathJumping[m_nstraightPath - 1].point, dat->posFrom);
                    m_straightPathJumping[m_nstraightPath].actionType = NO_ACTION;
                    dtVcopy(m_straightPathJumping[m_nstraightPath].point, dat->posTo);
                    ++m_nstraightPath;
                    first = false;
                    current_max_size -= 1;
                }
                if (current_max_size < 2)
                    break;
            }
            if (i + 1 == sz) {
                auto dat = (const JumpingPathEntry*)info.data + (info.size - 1);
                m_straightPathJumping[m_nstraightPath].actionType = NO_ACTION;
                dtVcopy(m_straightPathJumping[m_nstraightPath].point, epos);
                ++m_nstraightPath;
                m_polys[m_npolys] = dat->polyRefTo;
                ++m_npolys;
            }
        }
    }
    assert(m_straightPathJumping[m_nstraightPath - 1].actionType == NO_ACTION);

    return true;
}
*/

static void getPolyCenter(dtNavMesh* navMesh, dtPolyRef ref, float* center)
{
	center[0] = 0;
	center[1] = 0;
	center[2] = 0;
	
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	dtStatus status = navMesh->getTileAndPolyByRef(ref, &tile, &poly);
	if (dtStatusFailed(status))
		return;
		
	for (int i = 0; i < (int)poly->vertCount; ++i)
	{
		const float* v = &tile->verts[poly->verts[i]*3];
		center[0] += v[0];
		center[1] += v[1];
		center[2] += v[2];
	}
	const float s = 1.0f / poly->vertCount;
	center[0] *= s;
	center[1] *= s;
	center[2] *= s;
}

void NavMeshTesterTool::handleRender()
{
	duDebugDraw& dd = m_sample->getDebugDraw();
	
	static const unsigned int startCol = duRGBA(128,25,0,192);
	static const unsigned int endCol = duRGBA(51,102,0,129);
	static const unsigned int pathCol = duRGBA(0,0,0,64);
	
	const float agentRadius = m_sample->getAgentRadius();
	const float agentHeight = m_sample->getAgentHeight();
	const float agentClimb = m_sample->getAgentClimb();
	
	dd.depthMask(false);
	if (m_sposSet)
		drawAgent(m_spos, agentRadius, agentHeight, agentClimb, startCol);
	if (m_eposSet)
		drawAgent(m_epos, agentRadius, agentHeight, agentClimb, endCol);
	dd.depthMask(true);
	
	if (!m_navMesh)
	{
		return;
	}

    duDebugDrawNavMeshPolyPonts(&dd, *m_navMesh, m_startRef);
    duDebugDrawNavMeshPolyPonts(&dd, *m_navMesh, m_endRef);

	if (m_toolMode == TOOLMODE_PATHFIND_FOLLOW)
	{
		duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_startRef, startCol);
		duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_endRef, endCol);
		
		if (m_npolys)
		{
			for (int i = 0; i < m_npolys; ++i)
			{
				if (m_polys[i] == m_startRef || m_polys[i] == m_endRef)
					continue;
				duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
			}
		}
				
		if (m_nsmoothPath)
		{
			dd.depthMask(false);
			const unsigned int spathCol = duRGBA(0,0,0,220);
			dd.begin(DU_DRAW_LINES, 3.0f);
			for (int i = 0; i < m_nsmoothPath; ++i)
				dd.vertex(m_smoothPath[i*3], m_smoothPath[i*3+1]+0.1f, m_smoothPath[i*3+2], spathCol);
			dd.end();
			dd.depthMask(true);
		}
		
		if (m_pathIterNum)
		{
			duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_pathIterPolys[0], duRGBA(255,255,255,128));

			dd.depthMask(false);
			dd.begin(DU_DRAW_LINES, 1.0f);
			
			const unsigned int prevCol = duRGBA(255,192,0,220);
			const unsigned int curCol = duRGBA(255,255,255,220);
			const unsigned int steerCol = duRGBA(0,192,255,220);

			dd.vertex(m_prevIterPos[0],m_prevIterPos[1]-0.3f,m_prevIterPos[2], prevCol);
			dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3f,m_prevIterPos[2], prevCol);

			dd.vertex(m_iterPos[0],m_iterPos[1]-0.3f,m_iterPos[2], curCol);
			dd.vertex(m_iterPos[0],m_iterPos[1]+0.3f,m_iterPos[2], curCol);

			dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3f,m_prevIterPos[2], prevCol);
			dd.vertex(m_iterPos[0],m_iterPos[1]+0.3f,m_iterPos[2], prevCol);

			dd.vertex(m_prevIterPos[0],m_prevIterPos[1]+0.3f,m_prevIterPos[2], steerCol);
			dd.vertex(m_steerPos[0],m_steerPos[1]+0.3f,m_steerPos[2], steerCol);
			
			for (int i = 0; i < m_steerPointCount-1; ++i)
			{
				dd.vertex(m_steerPoints[i*3+0],m_steerPoints[i*3+1]+0.2f,m_steerPoints[i*3+2], duDarkenCol(steerCol));
				dd.vertex(m_steerPoints[(i+1)*3+0],m_steerPoints[(i+1)*3+1]+0.2f,m_steerPoints[(i+1)*3+2], duDarkenCol(steerCol));
			}
			
			dd.end();
			dd.depthMask(true);
		}
	}
    else if (
        m_toolMode == TOOLMODE_PATHFIND_STRAIGHT ||
        m_toolMode == TOOLMODE_PATHFIND_SLICED ||
        m_toolMode == TOOLMODE_PATHFIND_STRAIGHT_WITH_JUMPS
    ) {
        duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_startRef, startCol);
		duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_endRef, endCol);

        if (m_npolys)
		{
            for (int i = 0; i < m_npolys; ++i)
			{
                if (m_polys[i] == m_startRef || m_polys[i] == m_endRef)
                    continue;
                duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
			}
		}
		
		if (m_nstraightPath)
		{
            const unsigned int spathCol = duRGBA(64,16,0,220);
            const unsigned int offMeshCol = duRGBA(128,96,0,220);

            if (m_toolMode == TOOLMODE_PATHFIND_STRAIGHT_WITH_JUMPS) {
                //dd.depthMask(false);
                //dd.begin(DU_DRAW_LINES, 2.0f);
                //std::vector<int> jumpIndices;
                //for (int i = 0; i < m_nstraightPath - 1; ++i)
                //{
                //    if (m_straightPathJumping[i].actionType != NO_ACTION) {
                //        jumpIndices.emplace_back(i);
                //        continue;
                //    }

                //    unsigned int col;
                //    if (m_straightPathFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
                //        col = offMeshCol;
                //    else
                //        col = spathCol;
                //    dd.vertex(
                //        m_straightPathJumping[i].point[0],
                //        m_straightPathJumping[i].point[1] + 0.4f,
                //        m_straightPathJumping[i].point[2],
                //        col
                //    );
                //    dd.vertex(
                //        m_straightPathJumping[i + 1].point[0],
                //        m_straightPathJumping[i + 1].point[1] + 0.4f,
                //        m_straightPathJumping[i + 1].point[2],
                //        col
                //    );
                //}
                //dd.end();

                //for (int i : jumpIndices) {
                //    const float* v0 = m_straightPathJumping[i].point;
                //    const float* v1 = m_straightPathJumping[i + 1].point;
                //    auto& dd = m_sample->getDebugDraw();
                //    duDebugDrawArc(&dd, v0[0],v0[1],v0[2], v1[0],v1[1],v1[2],
                //                        0.25f, 0.0f, 0.4f, duRGBA(0,0,0,192), 2.0f);
                //}
            } else {
                dd.depthMask(false);
                dd.begin(DU_DRAW_LINES, 2.0f);
                for (int i = 0; i < m_nstraightPath-1; ++i)
                {
                    unsigned int col;
                    if (m_straightPathFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
                        col = offMeshCol;
                    else
                        col = spathCol;

                    dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2], col);
                    dd.vertex(m_straightPath[(i+1)*3], m_straightPath[(i+1)*3+1]+0.4f, m_straightPath[(i+1)*3+2], col);
                }
                dd.end();
            }

			dd.begin(DU_DRAW_POINTS, 6.0f);
			for (int i = 0; i < m_nstraightPath; ++i)
			{
				unsigned int col;
				if (m_straightPathFlags[i] & DT_STRAIGHTPATH_START)
					col = startCol;
				else if (m_straightPathFlags[i] & DT_STRAIGHTPATH_END)
					col = endCol;
				else if (m_straightPathFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
					col = offMeshCol;
				else
					col = spathCol;
				dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2], col);
			}
			dd.end();

			dd.depthMask(true);
		}
	}
	else if (m_toolMode == TOOLMODE_RAYCAST)
	{
		duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_startRef, startCol);
		
		if (m_nstraightPath)
		{
			for (int i = 1; i < m_npolys; ++i)
				duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
			
			dd.depthMask(false);
			const unsigned int spathCol = m_hitResult ? duRGBA(64,16,0,220) : duRGBA(240,240,240,220);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int i = 0; i < m_nstraightPath-1; ++i)
			{
				dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2], spathCol);
				dd.vertex(m_straightPath[(i+1)*3], m_straightPath[(i+1)*3+1]+0.4f, m_straightPath[(i+1)*3+2], spathCol);
			}
			dd.end();
			dd.begin(DU_DRAW_POINTS, 4.0f);
			for (int i = 0; i < m_nstraightPath; ++i)
				dd.vertex(m_straightPath[i*3], m_straightPath[i*3+1]+0.4f, m_straightPath[i*3+2], spathCol);
			dd.end();

			if (m_hitResult)
			{
				const unsigned int hitCol = duRGBA(0,0,0,128);
				dd.begin(DU_DRAW_LINES, 2.0f);
				dd.vertex(m_hitPos[0], m_hitPos[1] + 0.4f, m_hitPos[2], hitCol);
				dd.vertex(m_hitPos[0] + m_hitNormal[0]*agentRadius,
						  m_hitPos[1] + 0.4f + m_hitNormal[1]*agentRadius,
						  m_hitPos[2] + m_hitNormal[2]*agentRadius, hitCol);
				dd.end();
			}
			dd.depthMask(true);
		}
	}
	else if (m_toolMode == TOOLMODE_DISTANCE_TO_WALL)
	{
		duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_startRef, startCol);
		dd.depthMask(false);
		duDebugDrawCircle(&dd, m_spos[0], m_spos[1]+agentHeight/2, m_spos[2], m_distanceToWall, duRGBA(64,16,0,220), 2.0f);
		dd.begin(DU_DRAW_LINES, 3.0f);
		dd.vertex(m_hitPos[0], m_hitPos[1] + 0.02f, m_hitPos[2], duRGBA(0,0,0,192));
		dd.vertex(m_hitPos[0], m_hitPos[1] + agentHeight, m_hitPos[2], duRGBA(0,0,0,192));
		dd.end();
		dd.depthMask(true);
	}
	else if (m_toolMode == TOOLMODE_FIND_POLYS_IN_CIRCLE)
	{
		for (int i = 0; i < m_npolys; ++i)
		{
			duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
			dd.depthMask(false);
			if (m_parent[i])
			{
				float p0[3], p1[3];
				dd.depthMask(false);
				getPolyCenter(m_navMesh, m_parent[i], p0);
				getPolyCenter(m_navMesh, m_polys[i], p1);
				duDebugDrawArc(&dd, p0[0],p0[1],p0[2], p1[0],p1[1],p1[2], 0.25f, 0.0f, 0.4f, duRGBA(0,0,0,128), 2.0f);
				dd.depthMask(true);
			}
			dd.depthMask(true);
		}
		
		if (m_sposSet && m_eposSet)
		{
			dd.depthMask(false);
			const float dx = m_epos[0] - m_spos[0];
			const float dz = m_epos[2] - m_spos[2];
			const float dist = sqrtf(dx*dx + dz*dz);
			duDebugDrawCircle(&dd, m_spos[0], m_spos[1]+agentHeight/2, m_spos[2], dist, duRGBA(64,16,0,220), 2.0f);
			dd.depthMask(true);
		}
	}	
	else if (m_toolMode == TOOLMODE_FIND_POLYS_IN_SHAPE)
	{
		for (int i = 0; i < m_npolys; ++i)
		{
			duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
			dd.depthMask(false);
			if (m_parent[i])
			{
				float p0[3], p1[3];
				dd.depthMask(false);
				getPolyCenter(m_navMesh, m_parent[i], p0);
				getPolyCenter(m_navMesh, m_polys[i], p1);
				duDebugDrawArc(&dd, p0[0],p0[1],p0[2], p1[0],p1[1],p1[2], 0.25f, 0.0f, 0.4f, duRGBA(0,0,0,128), 2.0f);
				dd.depthMask(true);
			}
			dd.depthMask(true);
		}
		
		if (m_sposSet && m_eposSet)
		{
			dd.depthMask(false);
			const unsigned int col = duRGBA(64,16,0,220);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int i = 0, j = 3; i < 4; j=i++)
			{
				const float* p0 = &m_queryPoly[j*3];
				const float* p1 = &m_queryPoly[i*3];
				dd.vertex(p0, col);
				dd.vertex(p1, col);
			}
			dd.end();
			dd.depthMask(true);
		}
	}
	else if (m_toolMode == TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD)
	{
		for (int i = 0; i < m_npolys; ++i)
		{
			duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
			dd.depthMask(false);
			if (m_parent[i])
			{
				float p0[3], p1[3];
				dd.depthMask(false);
				getPolyCenter(m_navMesh, m_parent[i], p0);
				getPolyCenter(m_navMesh, m_polys[i], p1);
				duDebugDrawArc(&dd, p0[0],p0[1],p0[2], p1[0],p1[1],p1[2], 0.25f, 0.0f, 0.4f, duRGBA(0,0,0,128), 2.0f);
				dd.depthMask(true);
			}

			static const int MAX_SEGS = DT_VERTS_PER_POLYGON*4;
			float segs[MAX_SEGS*6];
			dtPolyRef refs[MAX_SEGS];
			memset(refs, 0, sizeof(dtPolyRef)*MAX_SEGS); 
			int nsegs = 0;
			m_navQuery->getPolyWallSegments(m_polys[i], &m_filter, segs, refs, &nsegs, MAX_SEGS);
			dd.begin(DU_DRAW_LINES, 2.0f);
			for (int j = 0; j < nsegs; ++j)
			{
				const float* s = &segs[j*6];
				
				// Skip too distant segments.
				float tseg;
				float distSqr = dtDistancePtSegSqr2D(m_spos, s, s+3, tseg);
				if (distSqr > dtSqr(m_neighbourhoodRadius))
					continue;
				
				float delta[3], norm[3], p0[3], p1[3];
				dtVsub(delta, s+3,s);
				dtVmad(p0, s, delta, 0.5f);
				norm[0] = delta[2];
				norm[1] = 0;
				norm[2] = -delta[0];
				dtVnormalize(norm);
				dtVmad(p1, p0, norm, agentRadius*0.5f);

				// Skip backfacing segments.
				if (refs[j])
				{
					unsigned int col = duRGBA(255,255,255,32);
					dd.vertex(s[0],s[1]+agentClimb,s[2],col);
					dd.vertex(s[3],s[4]+agentClimb,s[5],col);
				}
				else
				{
					unsigned int col = duRGBA(192,32,16,192);
					if (dtTriArea2D(m_spos, s, s+3) < 0.0f)
						col = duRGBA(96,32,16,192);
					
					dd.vertex(p0[0],p0[1]+agentClimb,p0[2],col);
					dd.vertex(p1[0],p1[1]+agentClimb,p1[2],col);

					dd.vertex(s[0],s[1]+agentClimb,s[2],col);
					dd.vertex(s[3],s[4]+agentClimb,s[5],col);
				}
			}
			dd.end();
			
			dd.depthMask(true);
		}
		
		if (m_sposSet)
		{
			dd.depthMask(false);
			duDebugDrawCircle(&dd, m_spos[0], m_spos[1]+agentHeight/2, m_spos[2], m_neighbourhoodRadius, duRGBA(64,16,0,220), 2.0f);
			dd.depthMask(true);
		}
	}
	
	if (m_nrandPoints > 0)
	{
		dd.begin(DU_DRAW_POINTS, 6.0f);
		for (int i = 0; i < m_nrandPoints; i++)
		{
			const float* p = &m_randPoints[i*3];
			dd.vertex(p[0],p[1]+0.1f,p[2], duRGBA(220,32,16,192));
		} 
		dd.end();
		
		if (m_randPointsInCircle && m_sposSet)
		{
			duDebugDrawCircle(&dd, m_spos[0], m_spos[1]+agentHeight/2, m_spos[2], m_randomRadius, duRGBA(64,16,0,220), 2.0f);
		}
	}
}

void NavMeshTesterTool::printNavmeshPolyId(double* proj, double* model, int* view, dtPolyRef ref) const
{
    const dtMeshTile* tile = 0;
    const dtPoly* poly = 0;
    if (dtStatusFailed(m_navMesh->getTileAndPolyByRef(ref, &tile, &poly))) {
        return;
    }
    float center[3];
    char buf[64];
    calcPolyCenter(tile, poly, center);
    std::sprintf(buf, "ref_id = %llu", ref);
    GLdouble x, y, z;
    if (gluProject(
        (GLdouble)center[0], (GLdouble)center[1], (GLdouble)center[2],
        model, proj, view, &x, &y, &z
    )) {
        imguiDrawText((int)x, (int)y, IMGUI_ALIGN_CENTER, buf, imguiRGBA(0,0,0,220));
    }
}

void NavMeshTesterTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
    printNavmeshPolyId(proj, model, view, m_startRef);
    printNavmeshPolyId(proj, model, view, m_endRef);

	// Draw start and end point labels
	if (m_sposSet && gluProject((GLdouble)m_spos[0], (GLdouble)m_spos[1], (GLdouble)m_spos[2],
								model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, "Start", imguiRGBA(0,0,0,220));
	}
	if (m_eposSet && gluProject((GLdouble)m_epos[0], (GLdouble)m_epos[1], (GLdouble)m_epos[2],
								model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y-25), IMGUI_ALIGN_CENTER, "End", imguiRGBA(0,0,0,220));
	}

    // ==================================================
    if (m_npolys && m_displayRefIdsInPath)
    {
        for (int i = 0; i < m_npolys; ++i) {
            printNavmeshPolyId(proj, model, view, m_polys[i]);
        }
    }
    for (int i = 0; i < m_nstraightPath - 1; ++i)
    {
        //if (m_straightPathJumping[i].actionType != NO_ACTION) {
        //    const float* v0 = m_straightPathJumping[i].point;
        //    const float* v1 = m_straightPathJumping[i + 1].point;
        //    float center[3] = {
        //        (v0[0] + v1[0]) * 0.5f,
        //        (v0[1] + v1[1]) * 0.5f,
        //        (v0[2] + v1[2]) * 0.5f
        //    };
        //    double x, y, z;
        //    if (gluProject((double)center[0], (double)center[1], (double)center[2],
        //                                model, proj, view, &x, &y, &z)
        //    ) {
        //        //const char* text;
        //        std::string text;
        //        if (m_straightPathJumping[i].actionType == JUMP_DOWN)
        //            text = "jump down";
        //        else if (m_straightPathJumping[i].actionType == JUMP_FORWARD)
        //            text = "jump forward";
        //        else if (m_straightPathJumping[i].actionType == CLIMB)
        //            text = "climb";
        //        else if (m_straightPathJumping[i].actionType == CLIMB_OVERLAPPED_POLYS)
        //            text = "climb overlapped";
        //        else
        //            text = "unknown";
        //        if (text != "unknown" && i != m_nstraightPath - 1) {
        //            const float* start = m_straightPathJumping[i].point;
        //            const float* end = m_straightPathJumping[i + 1].point;
        //            std::ostringstream oss;
        //            oss << ", d: " << dtVdist2D(start, end) << ", h: "
        //                << end[1] - start[1];
        //            text.append(oss.str());
        //        }
        //        imguiDrawText(
        //            (int)x, (int)y, IMGUI_ALIGN_CENTER, text.c_str(), imguiRGBA(0,0,0,220)
        //        );
        //    }
        //}
    }
    // ==================================================
	
	// Tool help
	const int h = view[3];
	imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "LMB+SHIFT: Set start location  LMB: Set end location", imguiRGBA(255,255,255,192));	
}

void NavMeshTesterTool::drawAgent(const float* pos, float r, float h, float c, const unsigned int col)
{
	duDebugDraw& dd = m_sample->getDebugDraw();
	
	dd.depthMask(false);
	
	// Agent dimensions.	
	duDebugDrawCylinderWire(&dd, pos[0]-r, pos[1]+0.02f, pos[2]-r, pos[0]+r, pos[1]+h, pos[2]+r, col, 2.0f);

	duDebugDrawCircle(&dd, pos[0],pos[1]+c,pos[2],r,duRGBA(0,0,0,64),1.0f);

	unsigned int colb = duRGBA(0,0,0,196);
	dd.begin(DU_DRAW_LINES);
	dd.vertex(pos[0], pos[1]-c, pos[2], colb);
	dd.vertex(pos[0], pos[1]+c, pos[2], colb);
	dd.vertex(pos[0]-r/2, pos[1]+0.02f, pos[2], colb);
	dd.vertex(pos[0]+r/2, pos[1]+0.02f, pos[2], colb);
	dd.vertex(pos[0], pos[1]+0.02f, pos[2]-r/2, colb);
	dd.vertex(pos[0], pos[1]+0.02f, pos[2]+r/2, colb);
	dd.end();
	
	dd.depthMask(true);
}
