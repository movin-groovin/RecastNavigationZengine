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

#ifndef NAVMESHTESTERTOOL_H
#define NAVMESHTESTERTOOL_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "InputGeom.h"
#include "SampleInterfaces.h"

#include <string>

/*
class collisionDetectorGeometryWorld: public collisionDetectorGeometryInterface
{
public:
    collisionDetectorGeometryWorld(InputGeom* inGeom, bool active):
        m_inGeom(inGeom), m_active(active) {}
    collisionDetectorGeometryWorld(const collisionDetectorGeometryWorld&) = delete;
    collisionDetectorGeometryWorld& operator=(const collisionDetectorGeometryWorld&) = delete;

    void disable() {m_active = false;}
    void enable() {m_active = true;}
    bool getActive() const {return m_active;}
    int getCallsNum () const {return m_callsNum;}
    void clearCallsNum () {m_callsNum = 0;}

    static void Vmul(float* v, const float m)
    {
        v[0] *= m;
        v[1] *= m;
        v[2] *= m;
    }

    static void Vadd(float* res, const float* v)
    {
        res[0] += v[0];
        res[1] += v[1];
        res[2] += v[2];
    }

    static void Vsub(float* res, const float* v)
    {
        res[0] -= v[0];
        res[1] -= v[1];
        res[2] -= v[2];
    }

    static float Vlen(const float* v)
    {
        return rcSqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    static void calcObbPoints(const geometry::OBB* b, float* obbPoints);

    static void calcObbExt(
        const float* src,
        const float* dst,
        const float deltaH,
        const float halfWidth,
        const float scale,
        geometry::OBBExt* be
    );

    bool detectJumpForwardCollisions(
        const float* src, const float* dst, float deltaH
    ) const;

    bool detectClimbCollisions(
        const float* src, const float* dst, float xzDist, float deltaH
    ) const;

    bool detectJumpDownCollisions(const float* src, const float* dst, float deltaH) const;

    bool detectCollision(const float* src, const float* dst) const;

private:
    InputGeom* m_inGeom;
    bool m_active;
    mutable int m_callsNum = 0;
};
*/

class NavMeshTesterTool : public SampleTool
{
private:
    static constexpr float POLY_PICK_Y = 40.f;

	BuildContext* m_ctx;
    bool m_initError = false;
    Sample* m_sample;
	
	dtNavMesh* m_navMesh;
	dtNavMeshQuery* m_navQuery;
	dtQueryFilter m_filter;
	dtStatus m_pathFindStatus;

	enum ToolMode
	{
		TOOLMODE_PATHFIND_FOLLOW,
		TOOLMODE_PATHFIND_STRAIGHT,
        TOOLMODE_PATHFIND_STRAIGHT_WITH_JUMPS,
		TOOLMODE_PATHFIND_SLICED,
		TOOLMODE_RAYCAST,
		TOOLMODE_DISTANCE_TO_WALL,
		TOOLMODE_FIND_POLYS_IN_CIRCLE,
		TOOLMODE_FIND_POLYS_IN_SHAPE,
		TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD,
	};
	
	ToolMode m_toolMode;

	int m_straightPathOptions;
	bool m_displayRefIdsInPath;
	
	static const int MAX_POLYS = 256;
	static const int MAX_SMOOTH = 2048;
	
    // ====================================
    static constexpr float JUMPING_RATIO = 0.1;
    static constexpr float NORMAL_RATIO = 1.0 - JUMPING_RATIO;
    //static const int JUMPING_SIZE =
    //    MAX_POLYS * JUMPING_RATIO * sizeof(JumpingPathEntry) +
    //    MAX_POLYS * NORMAL_RATIO * sizeof(NormalPathEntry);
    static const int NCOORDS = 3;
    //char m_polysJumping[JUMPING_SIZE];
    //StaticArrayInternal m_infoPolysJumping;
    //CalcedPathEntry m_straightPathJumping[MAX_POLYS];
    int m_straightWithJumpsPathOptions;
    //collisionDetectorGeometryWorld m_collDet;
    // ====================================

	dtPolyRef m_startRef;
	dtPolyRef m_endRef;
	dtPolyRef m_polys[MAX_POLYS];
	dtPolyRef m_parent[MAX_POLYS];
	int m_npolys;
	float m_straightPath[MAX_POLYS*3];
	unsigned char m_straightPathFlags[MAX_POLYS];
	dtPolyRef m_straightPathPolys[MAX_POLYS];
	int m_nstraightPath;
	float m_polyPickExt[3];
	float m_smoothPath[MAX_SMOOTH*3];
	int m_nsmoothPath;
	float m_queryPoly[4*3];

	static const int MAX_RAND_POINTS = 64;
	float m_randPoints[MAX_RAND_POINTS*3];
	int m_nrandPoints;
	bool m_randPointsInCircle;
	
	float m_spos[3];
	float m_epos[3];
	float m_hitPos[3];
	float m_hitNormal[3];
	bool m_hitResult;
	float m_distanceToWall;
	float m_neighbourhoodRadius;
	float m_randomRadius;
	bool m_sposSet;
	bool m_eposSet;

	int m_pathIterNum;
	dtPolyRef m_pathIterPolys[MAX_POLYS]; 
	int m_pathIterPolyCount;
	float m_prevIterPos[3], m_iterPos[3], m_steerPos[3], m_targetPos[3];
	
	static const int MAX_STEER_POINTS = 10;
	float m_steerPoints[MAX_STEER_POINTS*3];
	int m_steerPointCount;
	
public:
    NavMeshTesterTool(InputGeom* inGeom, BuildContext* ctx);
    ~NavMeshTesterTool();

	virtual int type() { return TOOL_NAVMESH_TESTER; }
	virtual void init(Sample* sample);
    void init(dtNavMesh* navMesh, dtNavMeshQuery* navQuery);
	virtual void reset();
	virtual void handleMenu();
	virtual void handleClick(const float* s, const float* p, bool shift);
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleUpdate(const float dt);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);

	void recalc();
	void drawAgent(const float* pos, float r, float h, float c, const unsigned int col);

private:
    //bool findPathWithJumps(
    //    dtPolyRef startRef, dtPolyRef endRef, const float* spos, const float* epos
    //);

private:
    void printNavmeshPolyId(double* proj, double* model, int* view, dtPolyRef ref) const;
};

#endif // NAVMESHTESTERTOOL_H
