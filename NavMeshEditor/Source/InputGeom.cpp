//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This /*so*/ftware is provided 'as-is', without any express or implied
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
#include <ctype.h>
#include <string.h>
#include <cstdlib>
#include <algorithm>
#include <utility>
#include <memory>
#include <chrono>
#include <unordered_map>
#include <cmath>
#include <cstdint>
#include <tuple>
#include <new>
#include <type_traits>
#include "Recast.h"
#include "InputGeom.h"
#include "MeshLoaderObj.h"
#include "DebugDraw.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "Sample.h"

static bool intersectSegmentTriangle(
	const float* sp, const float* sq, const float* a, const float* b, const float* c, float &t
) {
	float v, w;
    float ab[3], ac[3], qp[3], ap[3], norm[3], e[3];
	rcVsub(ab, b, a);
	rcVsub(ac, c, a);
	rcVsub(qp, sp, sq);
	
	// Compute triangle normal. Can be precalculated or cached if
	// intersecting multiple segments against the same triangle
	rcVcross(norm, ab, ac);
	
	// Compute denominator d. If d <= 0, segment is parallel to or points
	// away from triangle, so exit early
	float d = rcVdot(qp, norm);
	if (d /*<=*/== 0.0f) return false;
	float invD = 1.f / d;
	
	// Compute intersection t value of pq with plane of triangle. A ray
	// intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
	// dividing by d until intersection has been found to pierce triangle
	rcVsub(ap, sp, a);
	t = rcVdot(ap, norm) * invD;
	if (t < 0.0f) return false;
	if (t > /*d*/1.f) return false; // For segment; exclude this code line for a ray test
	
	// Compute barycentric coordinate components and test if within bounds
	rcVcross(e, qp, ap);
	v = rcVdot(ac, e) * invD;
	if (v < 0.0f || v > /*d*/1.f) return false;
	w = -rcVdot(ab, e) * invD;
	if (w < 0.0f || v + w > /*d*/1.f) return false;
	
	// Segment/ray intersects triangle. Perform delayed division
	//t /= d;
	
	return true;
}

static char* parseRow(char* buf, char* bufEnd, char* row, int len)
{
	bool start = true;
	bool done = false;
	int n = 0;
	while (!done && buf < bufEnd)
	{
		char c = *buf;
		buf++;
		// multirow
		switch (c)
		{
			case '\n':
				if (start) break;
				done = true;
				break;
			case '\r':
				break;
			case '\t':
			case ' ':
				if (start) break;
				// else falls through
			default:
				start = false;
				row[n++] = c;
				if (n >= len-1)
					done = true;
				break;
		}
	}
	row[n] = '\0';
	return buf;
}

InputGeom::InputGeom():
	m_ctx(0),
	m_meshExt(0),
	m_hasBuildSettings(false)
{
}

InputGeom::~InputGeom()
{
	delete m_meshExt;
}

bool InputGeom::loadMesh(
	rcContext* ctx,
	const char* navMeshName,
	const char* staticMesh,
	const char* vobsMesh,
    const char* markedMesh,
    float offsetSize,
    float bvhGridSize
) {
	if (m_meshExt)
	{
		delete m_meshExt;
		m_meshExt = nullptr;
	}

    m_meshExt = new(std::nothrow) rcMeshLoaderObjExt;
	if (!m_meshExt)
	{
        ctx->log(RC_LOG_ERROR, "loadMesh(dir): Out of memory 'm_mesh'");
		return false;
	}
#ifdef PRINT_STRUCTURE_STAT
    auto tp1 = std::chrono::steady_clock::now();
#endif
    ctx->log(
        RC_LOG_WARNING,
        "Loading static: %s, vobs: %s, marked: %s",
        staticMesh, vobsMesh, markedMesh
    );
    auto ret = m_meshExt->load(navMeshName, staticMesh, vobsMesh, markedMesh, offsetSize, true);
	if (!ret.isOk())
	{
		ctx->log(RC_LOG_ERROR, "loadMesh(dir): could not load mesh, "
				 "static: %s, vobs: %s, marked: %s, codes: 0 = %d, 1 = %d, 2 = %d, 3 = %d",
				 staticMesh, vobsMesh, markedMesh, (int)ret.code0, (int)ret.code1,
				 (int)ret.code2, (int)ret.code3);
		return false;
	}

#ifdef PRINT_STRUCTURE_STAT
    auto tp2 = std::chrono::steady_clock::now();
#endif
    if (m_space.load(ctx, m_meshExt, static_cast<int>(bvhGridSize)) != Grid2dBvh::SUCCESSFUL) {
        ctx->log(RC_LOG_ERROR, "loadMesh(dir): mesh bvh construction error");
		return false;
	}
    m_space.getBounds(m_meshBMin, m_meshBMax);

#ifdef PRINT_STRUCTURE_STAT
    auto tp3 = std::chrono::steady_clock::now();
    auto diffParsing = std::chrono::duration_cast<std::chrono::milliseconds>(tp2 - tp1).count();
    auto diffConstr = std::chrono::duration_cast<std::chrono::milliseconds>(tp3 - tp2).count();
    ctx->log(
        RC_LOG_WARNING,
        "loadMesh(dir), mesh parsing: %f sec, bvh construction: %f sec",
        (double)diffParsing / 1000, (double)diffConstr / 1000
    );
    m_space.printStat(ctx);
#endif
	m_ctx = ctx;

    return true;
}

bool InputGeom::loadFromDir(
    class rcContext* ctx, const char* filepath, float offsetSize, float bvhGridSize
) {
#ifdef WIN32
	static const char SEP = '\\';
#else
	static const char SEP = '/';
#endif
	char staticMesh[STR_SIZE];
	char vobsMesh[STR_SIZE];
	char markedMesh[STR_SIZE];
	char navMesh[STR_SIZE];

	int strLen = static_cast<int>(std::strlen(filepath));
	std::strcpy(staticMesh, filepath);
	staticMesh[strLen] = SEP;
	std::strcpy(vobsMesh, filepath);
	vobsMesh[strLen] = SEP;
	std::strcpy(markedMesh, filepath);
	markedMesh[strLen] = SEP;
	std::strcpy(navMesh, filepath);
	navMesh[strLen] = SEP;
	std::strcpy(m_baseMeshName, filepath);
	m_baseMeshName[STR_SIZE - 1] = '\0';

	strLen += 1;
	std::strcpy(staticMesh + strLen, "static_mesh.obj");
	staticMesh[STR_SIZE - 1] = '\0';
	std::strcpy(vobsMesh + strLen, "vobs_mesh.obj");
	vobsMesh[STR_SIZE - 1] = '\0';
	std::strcpy(markedMesh + strLen, "marked_mesh.obj");
	markedMesh[STR_SIZE - 1] = '\0';
	std::strcpy(navMesh + strLen, "navmesh.bin");
	navMesh[STR_SIZE - 1] = '\0';
	std::strcpy(m_markedMeshName, markedMesh);
	m_markedMeshName[STR_SIZE - 1] = '\0';

    return loadMesh(ctx, navMesh, staticMesh, vobsMesh, markedMesh, offsetSize, bvhGridSize);
}

static bool isectSegAABB(const float* sp, const float* sq,
						 const float* amin, const float* amax,
						 float& tmin, float& tmax)
{
	static const float EPS = 1e-6f;
	
	float d[3];
	d[0] = sq[0] - sp[0];
	d[1] = sq[1] - sp[1];
	d[2] = sq[2] - sp[2];
	tmin = 0.0;
	tmax = 1.0f;
	
	for (int i = 0; i < 3; i++)
	{
		if (fabsf(d[i]) < EPS)
		{
			if (sp[i] < amin[i] || sp[i] > amax[i])
				return false;
		}
		else
		{
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			if (tmin > tmax) return false;
		}
	}
	
	return true;
}

int InputGeom::getOverlappingRectCellIds(
	const float* min, const float* max, int* cellIds, int idsSize
) const {
    return m_space.getOverlappingRectCellIds(min, max, cellIds, idsSize);
}

const Grid2dBvh::TrianglesData& InputGeom::extractOverlappingRectData(
	int cellId
) const {
	return m_space.extractOverlappingRectData(cellId);
}

bool InputGeom::obbCollDetect(const OBBExt* be) const
{
#ifdef PRINT_TRI_VS_OBB_LATENCY
    static uint64_t callCnt = 0;
    static uint64_t nsCnt = 0;
    auto tp1 = std::chrono::steady_clock::now();
#endif
	bool ret = m_space.obbTriCollisionFirstHit(be);
#ifdef PRINT_TRI_VS_OBB_LATENCY
    auto tp2 = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
    std::printf("Time diff obb vs tri ns: %d, total nodes: %d, total leafes: %d, "
           "total polys: %d, res: %d\n",
           (int)diff, m_space.getNodesPerCall(), m_space.getLeafesPerCall(),
           m_space.getPolysPerCall(), (int)ret);
    m_space.clearStatPerCall();
    ++callCnt;
    nsCnt += diff;
    if (callCnt % 10 == 0) {
        std::printf("Ns obb vs tri per call +100: %zu\n", nsCnt / callCnt);
    }
#endif
    return ret;
}

bool InputGeom::raycastMesh(
    const float* src, const float* dst, float& tmin, bool nearestHit
) const {
#ifdef PRINT_TRI_VS_SEG_LATENCY_TOTAL
    static uint64_t callCnt = 0;
    static uint64_t nsCnt = 0;
    auto tp1 = std::chrono::steady_clock::now();
#endif
	bool ret;
	if (nearestHit)
		ret = m_space.segTriCollisionNearestHit(src, dst, tmin);
	else
		ret = m_space.segTriCollisionFirstHit(src, dst, tmin);
#ifdef PRINT_TRI_VS_SEG_LATENCY_TOTAL
    auto tp2 = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
    ++callCnt;
    nsCnt += diff;
    if (callCnt % PRINT_AFTER_N_POLYS == 0) {
        std::printf("Ns seg vs tri per call +%d: %zu ns\n", PRINT_AFTER_N_POLYS, nsCnt / callCnt);
    }
#endif
#ifdef PRINT_TRI_VS_SEG_LATENCY
    std::printf("Time diff seg vs tri ns: %d, total nodes: %d, total leafes: %d,"
           "total polys: %d\n", (int)diff, m_space.getNodesPerCall(),
           m_space.getLeafesPerCall(), m_space.getPolysPerCall());
    m_space.clearStatPerCall();
#endif
    return ret;
}

int InputGeom::getOffMeshConnectionCount() const {
	return m_space.getOffMeshData().offMeshNum;
}

const float* InputGeom::getOffMeshConnectionVerts() const {
	return m_space.getOffMeshData().offMeshVerts;
}

const float* InputGeom::getOffMeshConnectionRads() const {
	return m_space.getOffMeshData().offMeshRads;
}

const unsigned char* InputGeom::getOffMeshConnectionDirs() const {
	return m_space.getOffMeshData().offMeshDirs;
}

const unsigned char* InputGeom::getOffMeshConnectionAreas() const {
	return m_space.getOffMeshData().offMeshAreas;
}

const unsigned short* InputGeom::getOffMeshConnectionFlags() const {
	return m_space.getOffMeshData().offMeshFlags;
}

const unsigned int* InputGeom::getOffMeshConnectionId() const {
	return m_space.getOffMeshData().offMeshId;
}

void InputGeom::addOffMeshConnection(
	const float* spos, const float* epos, const float rad,
	unsigned char bidir, unsigned char area, unsigned short flags
) {
	int ret = m_space.addOffMeshConn(spos, epos, rad, bidir, area, flags);
	if (ret != Grid2dBvh::SUCCESSFUL) {
		m_ctx->log(RC_LOG_ERROR, "Error of addOffMeshConn, code: %d", ret);
	}
}

void InputGeom::deleteOffMeshConnection(int i)
{
	m_space.deleteOffMeshConn(i);
}

void InputGeom::drawOffMeshConnections(duDebugDraw* dd, bool hilight)
{
	auto getOffMeshConnNum = [this] () {
		return m_space.getOffMeshData().offMeshNum;
	};
	auto getOffMeshData = [this] () {
		const Grid2dBvh::OffMeshData& data = m_space.getOffMeshData();
		return std::make_tuple(
			data.offMeshVerts, data.offMeshRads, data.offMeshDirs
		);
	};

	unsigned int conColor = duRGBA(192,0,128,192);
	unsigned int baseColor = duRGBA(0,0,0,64);
	dd->depthMask(false);

	dd->begin(DU_DRAW_LINES, 2.0f);
	int offConnNum = getOffMeshConnNum();
	auto dat = getOffMeshData();
	const float* verts = std::get<0>(dat);
	const float* rads = std::get<1>(dat);
	const uint8_t* dirs = std::get<2>(dat);
	for (int i = 0; i < offConnNum; ++i)
	{
		const float* v = &verts[i*3*2];

		dd->vertex(v[0],v[1],v[2], baseColor);
		dd->vertex(v[0],v[1]+0.2f,v[2], baseColor);
		
		dd->vertex(v[3],v[4],v[5], baseColor);
		dd->vertex(v[3],v[4]+0.2f,v[5], baseColor);
		
		duAppendCircle(dd, v[0],v[1]+0.1f,v[2], rads[i], baseColor);
		duAppendCircle(dd, v[3],v[4]+0.1f,v[5], rads[i], baseColor);

		if (hilight)
		{
			duAppendArc(dd, v[0],v[1],v[2], v[3],v[4],v[5], 0.25f,
						(dirs[i]&1) ? 0.6f : 0.0f, 0.6f, conColor);
		}
	}	
	dd->end();

	dd->depthMask(true);
}

int InputGeom::getConvexVolumeCount() const {
	return m_space.getMarkedAreaSize();
}

const rcMeshLoaderObjExt::MarkedEntry* InputGeom::getConvexVolume(int i) const {
	return m_space.getMarkedArea(i);
}

void InputGeom::addConvexVolume(
	const float* verts,
	const int nverts,
	const float minh,
	const float maxh,
    unsigned char area
) {
	if (Grid2dBvh::SUCCESSFUL != m_space.addMarkedArea(verts, nverts, minh, maxh, area)) {
		m_ctx->log(RC_LOG_ERROR, "Error of addMarkedArea");
	}
}

void InputGeom::deleteConvexVolume(int i)
{
	m_space.deleteMarkedArea(i);
}

void InputGeom::totalDeleteMarkedAreas()
{
	m_space.totalDeleteMarkedAreas();
}

void InputGeom::drawConvexVolumes(struct duDebugDraw* dd, bool /*hilight*/)
{
	auto getVolCnt = [this] () {
		return m_space.getMarkedAreaSize();
	};
	auto getData = [this] (int idx) {
		const rcMeshLoaderObjExt::MarkedEntry& ma = *m_space.getMarkedArea(idx);
		return std::make_tuple(
			ma.vertsNum, static_cast<const float*>(ma.verts), ma.minh, ma.maxh, ma.area
		);
	};

	dd->depthMask(false);

	dd->begin(DU_DRAW_TRIS);
	
	int areasNum = getVolCnt();
	for (int i = 0; i < areasNum; ++i)
	{
		auto dat = getData(i);
		int nverts = std::get<0>(dat);
		const float* verts = std::get<1>(dat);
		float hmin = std::get<2>(dat);
		float hmax = std::get<3>(dat);
        int area = PolyAreaFlags::clearInhabitedFlag(std::get<4>(dat));
		unsigned int col = duTransCol(dd->areaToCol(area), 32);
		for (int j = 0, k = nverts-1; j < nverts; k = j++)
		{
			const float* va = &verts[k*3];
			const float* vb = &verts[j*3];

			dd->vertex(verts[0],hmax,verts[2], col);
			dd->vertex(vb[0],hmax,vb[2], col);
			dd->vertex(va[0],hmax,va[2], col);
			
			dd->vertex(va[0],hmin,va[2], duDarkenCol(col));
			dd->vertex(va[0],hmax,va[2], col);
			dd->vertex(vb[0],hmax,vb[2], col);

			dd->vertex(va[0],hmin,va[2], duDarkenCol(col));
			dd->vertex(vb[0],hmax,vb[2], col);
			dd->vertex(vb[0],hmin,vb[2], duDarkenCol(col));
		}
	}
	
	dd->end();

	dd->begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0; i < areasNum; ++i)
	{
		auto dat = getData(i);
		int nverts = std::get<0>(dat);
		const float* verts = std::get<1>(dat);
		float hmin = std::get<2>(dat);
		float hmax = std::get<3>(dat);
        int area = PolyAreaFlags::clearInhabitedFlag(std::get<4>(dat));
		unsigned int col = duTransCol(dd->areaToCol(area), 220);
		for (int j = 0, k = nverts-1; j < nverts; k = j++)
		{
			const float* va = &verts[k*3];
			const float* vb = &verts[j*3];
			dd->vertex(va[0],hmin,va[2], duDarkenCol(col));
			dd->vertex(vb[0],hmin,vb[2], duDarkenCol(col));
			dd->vertex(va[0],hmax,va[2], col);
			dd->vertex(vb[0],hmax,vb[2], col);
			dd->vertex(va[0],hmin,va[2], duDarkenCol(col));
			dd->vertex(va[0],hmax,va[2], col);
		}
	}
	dd->end();

	dd->begin(DU_DRAW_POINTS, 3.0f);
	for (int i = 0; i < areasNum; ++i)
	{
		auto dat = getData(i);
		int nverts = std::get<0>(dat);
		const float* verts = std::get<1>(dat);
		float hmin = std::get<2>(dat);
		float hmax = std::get<3>(dat);
		int area = std::get<4>(dat);
		unsigned int col = duDarkenCol(duTransCol(dd->areaToCol(area), 220));
		for (int j = 0; j < nverts; ++j)
		{
			dd->vertex(verts[j*3+0],verts[j*3+1]+0.1f,verts[j*3+2], col);
			dd->vertex(verts[j*3+0],hmin,verts[j*3+2], col);
			dd->vertex(verts[j*3+0],hmax,verts[j*3+2], col);
		}
	}
	dd->end();
	
	
	dd->depthMask(true);
}

int InputGeom::getVertCount() const
{
	return m_meshExt->getVertCount();
}

int InputGeom::getTriCount() const
{
	return m_meshExt->getTriCount();
}

const char* InputGeom::getNavMeshName() const
{
	return m_meshExt->getNavMeshName();
}

const char* InputGeom::getMarkedMeshName() const
{
	return m_markedMeshName;
}

const char* InputGeom::getBaseMeshName() const
{
	return m_baseMeshName;
}

static bool checkAabbsCollision(
    const float* aMin, const float* aMax, const float* bMin, const float* bMax
) {
    if (aMax[0] < bMin[0] || bMax[0] < aMin[0]) return false;
    if (aMax[1] < bMin[1] || bMax[1] < aMin[1]) return false;
    if (aMax[2] < bMin[2] || bMax[2] < aMin[2]) return false;
    return true;
}

static bool checkAabbsCollisionXZ(
    const float* aMin, const float* aMax, const float* bMin, const float* bMax
) {
    if (aMax[0] < bMin[0] || bMax[0] < aMin[0]) return false;
    if (aMax[2] < bMin[2] || bMax[2] < aMin[2]) return false;
    return true;
}

template <int N>
struct CoordComparerOnIds
{
    CoordComparerOnIds(const Aabb3D* boxes): boxes_(boxes) {}

    static_assert(N >= 0 && N <= 2, "Wrong axis number");
    bool operator() (const int l, const int r) const
    {
        return boxes_[l].min[N] < boxes_[r].min[N];
    }

    const Aabb3D* boxes_;
};

struct CoordComparer
{
    CoordComparer(int axisNum_, const Aabb3D* boxes_): axisNum(axisNum_), boxes(boxes_) {}

    bool operator() (const int l, const int r) const
    {
        return boxes[l].min[axisNum] < boxes[r].min[axisNum];
    }

    int axisNum;
    const Aabb3D* boxes;
};

template <typename F>
class ScopeExitImpl final {
public:
    ScopeExitImpl(F&& f): m_f(std::forward<F>(f)) {}
    ~ScopeExitImpl() noexcept(true) {
        try {
            if (m_active) m_f();
        } catch (...) {
            // TODO static compile check for noexception of 'f'
        }
    }
    void activate() const {m_active = true;}
    void deactivate() const {m_active = false;}

private:
    F m_f;
    mutable bool m_active = true;
};

Octree::Octree(int maxDepth, int minTrisInLeaf):
    m_maxDepth(maxDepth),
    m_minTrisInLeaf(minTrisInLeaf)
{}

Octree::~Octree() {}

bool Octree::Load(rcMeshLoaderObjExt* mesh)
{
    m_mesh = mesh;
    m_triIds = m_mesh->getTris();
    m_verts = m_mesh->getVerts();
    const float* verts = m_mesh->getVerts();
    const int* tris = m_mesh->getTris();
    m_polyNum = m_mesh->getTriCount();

    Aabb3D* bboxes = new(std::nothrow) Aabb3D[m_polyNum];
    if (!bboxes) {
        m_memInsufficient = true;
        return false;
    }
    for (int i = 0; i < m_maxDepth + 1; ++i) {
        m_constrTimeIds.push_back( std::vector<int>(m_polyNum, 0) );
    }
    memset(m_bmax, 0, sizeof(m_bmax));
    m_bmin[0] = m_bmin[1] = m_bmin[2] = std::numeric_limits<float>::max();
    std::vector<int>& zeroLevel = m_constrTimeIds[0];
    for (int i = 0; i < m_polyNum; ++i) {
        calcAabb(verts, &tris[i * 3], &bboxes[i]);
        bboxes[i].polyIndex = i;
        rcVmin(m_bmin, bboxes[i].min);
        rcVmax(m_bmax, bboxes[i].max);
        zeroLevel[i] = i;
    }
    rcVcopy(m_root.bmin, m_bmin);
    rcVcopy(m_root.bmax, m_bmax);
    float center[3] = {
        (m_bmin[0] + m_bmax[0]) * 0.5f,
        (m_bmin[1] + m_bmax[1]) * 0.5f,
        (m_bmin[2] + m_bmax[2]) * 0.5f
    };
    float halfSpan[3] = {
        (m_bmax[0] - m_bmin[0]) * 0.25f,
        (m_bmax[1] - m_bmin[1]) * 0.25f,
        (m_bmax[2] - m_bmin[2]) * 0.25f
    };
    float offset[3];
    float newCenter[3];
    for (int i = 0; i < 8; ++i) {
        offset[0] = (i & 1) ? halfSpan[0] : -halfSpan[0];
        offset[1] = (i & 2) ? halfSpan[1] : -halfSpan[1];
        offset[2] = (i & 4) ? halfSpan[2] : -halfSpan[2];
        newCenter[0] = center[0] + offset[0];
        newCenter[1] = center[1] + offset[1];
        newCenter[2] = center[2] + offset[2];
        m_root.childs[i] = LoadDo(
            1, newCenter, halfSpan, verts, tris, bboxes, m_polyNum
        );
    }
    m_averPolysInLeaf /= m_leafNum;
    delete [] bboxes;
    m_constrTimeIds.clear();
    return true;
}

Octree::Node* Octree::LoadDo(
    int depth, const float* center, const float* span, const float* verts,
    const int* tris, const Aabb3D* bboxes, int polysNum
) {
    Node* cur = new(std::nothrow) Node;
    if (!cur) {
        m_memInsufficient = true;
        return nullptr;
    }

    cur->bmin[0] = center[0] - span[0];// * 0.5f;
    cur->bmin[1] = center[1] - span[1];// * 0.5f;
    cur->bmin[2] = center[2] - span[2];// * 0.5f;
    cur->bmax[0] = center[0] + span[0];// * 0.5f;
    cur->bmax[1] = center[1] + span[1];// * 0.5f;
    cur->bmax[2] = center[2] + span[2];// * 0.5f;
    const auto& curLevel = m_constrTimeIds[depth - 1];
    auto& nextLevel = m_constrTimeIds[depth];
    int curPolysNum = 0;
    for (int i = 0, j = 0; i < polysNum; ++i) {
        const Aabb3D* box = &bboxes[curLevel[i]];
        if (checkAabbsCollision(cur->bmin, cur->bmax, box->min, box->max)) {
            ++curPolysNum;
            nextLevel[j++] = curLevel[i];
        }
    }
    if (!curPolysNum){
        delete cur;
        return nullptr;
    }

    if (depth == m_maxDepth || polysNum <= m_minTrisInLeaf) {
        if (m_curMaxDepthReached < depth) m_curMaxDepthReached = depth;
        cur->m_num = curPolysNum;
        cur->m_trianglePolys = new(std::nothrow) int [curPolysNum];
        if (!cur->m_trianglePolys) {
            m_memInsufficient = true;
            return nullptr;
        }
        for (int i = 0; i < curPolysNum; ++i) {
            cur->m_trianglePolys[i] = bboxes[nextLevel[i]].polyIndex;
        }
        ++m_leafNum;
        m_averPolysInLeaf += polysNum;
        if (m_minPolysInLeaf >= polysNum) m_minPolysInLeaf = polysNum;
        if (m_maxPolysInLeaf <= polysNum) m_maxPolysInLeaf = polysNum;
        return cur;
    }

    float childCenter[3] = {
        (cur->bmin[0] + cur->bmax[0]) * 0.5f,
        (cur->bmin[1] + cur->bmax[1]) * 0.5f,
        (cur->bmin[2] + cur->bmax[2]) * 0.5f
    };
    float childHalfSpan[3] = {
        (cur->bmax[0] - cur->bmin[0]) * 0.25f,
        (cur->bmax[1] - cur->bmin[1]) * 0.25f,
        (cur->bmax[2] - cur->bmin[2]) * 0.25f
    };
    float offset[3];
    float newCenter[3];
    for (int i = 0; i < 8; ++i) {
        offset[0] = (i & 1) ? childHalfSpan[0] : -childHalfSpan[0];
        offset[1] = (i & 2) ? childHalfSpan[1] : -childHalfSpan[1];
        offset[2] = (i & 4) ? childHalfSpan[2] : -childHalfSpan[2];
        newCenter[0] = childCenter[0] + offset[0];
        newCenter[1] = childCenter[1] + offset[1];
        newCenter[2] = childCenter[2] + offset[2];
        cur->childs[i] = LoadDo(
            depth + 1, newCenter, childHalfSpan, verts, tris, bboxes, curPolysNum
        );
    }

    return cur;
}

bool Octree::checkOverlapAabb(
    const float* start, const float* end, const float* bmin, const float* bmax
) {
    static const float EPSILON = 1e-6f;

    float tmin = 0;
    float tmax = 1;
    float d[3];
    d[0] = end[0] - start[0];
    d[1] = end[1] - start[1];
    d[2] = end[2] - start[2];

    for (int i = 0; i < 3; i++)
    {
        if (fabsf(d[i]) < EPSILON)
        {
            // Ray is parallel to slab. No hit if origin not within slab
            if (start[i] < bmin[i] || start[i] > bmax[i])
                return false;
        }
        else
        {
            // Compute intersection t value of ray with near and far plane of slab
            float ood = 1.0f / d[i];
            float t1 = (bmin[i] - start[i]) * ood;
            float t2 = (bmax[i] - start[i]) * ood;
            if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
            if (tmin > tmax) return false;
        }
    }
    return true;
}

bool Octree::detectSegmentPolyCollision(
    const float* start, const float* end, float& t, int n
) const {
    m_totalNodes = 0;
    m_leafNodes = 0;
    m_polys = 0;
    auto tp1 = std::chrono::steady_clock::now();
    bool res = detectSegmentPolyCollisionDo(start, end, &m_root, t);
    auto tp2 = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
    m_totalNodes += m_leafNodes;
    std::printf("%d = time diff ns: %d, total nodes: %d, total leafes: %d, total polys: %d\n",
           n, (int)diff, m_totalNodes, m_leafNodes, m_polys);
    return res;
}

bool Octree::detectSegmentPolyCollisionDo(
    const float* start, const float* end, const Node* cur, float& t
) const {
    if (!checkOverlapAabb(start, end, cur->bmin, cur->bmax)) return false;
    ++m_totalNodes;
    if(cur->isLeaf()) {
        ++m_leafNodes;
        for (int i = 0; i < cur->m_num; ++i) {
            const int* vIds = &m_triIds[cur->m_trianglePolys[i] * 3];
            ++m_polys;
            if (intersectSegmentTriangle(
                start,
                end,
                &m_verts[vIds[0] * 3],
                &m_verts[vIds[1] * 3],
                &m_verts[vIds[2] * 3],
                t
            )) return true;
        }
    } else {
        for (int i = 0; i < 8; ++i) {
            if (!cur->childs[i]) continue;
            if (detectSegmentPolyCollisionDo(start, end, cur->childs[i], t)) return true;
        }
    }
    return false;
}

void Octree::calcAabb(const float* verts, const int* triangle, Aabb3D* bbox)
{
    rcVcopy(bbox->min, &verts[triangle[0] * 3]);
    rcVcopy(bbox->max, &verts[triangle[0] * 3]);
    for (int i = 1; i < 3; ++i) {
        rcVmin(bbox->min, &verts[triangle[i] * 3]);
        rcVmax(bbox->max, &verts[triangle[i] * 3]);
    }
}


class BminBmaxSegmentTree
{
public:
    struct BminBmax
    {
        float min[3];
        float max[3];
    };

private:
    struct ResourceManager {
    public:
        static ResourceManager& Instance() {
            static ResourceManager obj;
            return obj;
        }

        BminBmax* alloc(int n) {
            if (m_n >= n) return m_data;
            free();
            m_data = new(std::nothrow) BminBmax[n];
            m_n = n;
            return m_data;
        }

        void free() {
            delete [] m_data;
            m_data = nullptr;
            m_n = 0;
        }

        BminBmax* getData() {return m_data;}
        int getSize() const {return m_n;}

    private:
        BminBmax* m_data = nullptr;
        int m_n = 0;
    };

public:
    BminBmaxSegmentTree() = default;
    ~BminBmaxSegmentTree() = default;
    BminBmaxSegmentTree(const BminBmaxSegmentTree&) = delete;
    const BminBmaxSegmentTree& operator=(const BminBmaxSegmentTree& ) = delete;

    static const ResourceManager& getResourceManager()
    {
        return ResourceManager::Instance();
    }
    static void freeResources()
    {
        ResourceManager::Instance().free();
    }

    bool calcTree(uint32_t i, uint32_t j, const Aabb3D* bboxes, const int* boxIds)
    {
        m_i = i;
        m_j = j;
        uint32_t nUpPow2 = 1 << (static_cast<uint32_t>(std::log2(j - i - 1)) + 1);
        assert(nUpPow2 >= j - i);
        m_curSize = nUpPow2 << 1;
        BminBmax* dat = ResourceManager::Instance().alloc(m_curSize);
        if (!dat) {
            m_curSize = 0;
            return false;
        }
        m_curDat = dat;

        dat->min[0] = dat->min[1] = dat->min[2] = std::numeric_limits<float>::max();
        dat->max[0] = dat->max[1] = dat->max[2] = -std::numeric_limits<float>::max();
        BminBmax* leafes = dat + nUpPow2;
        for (uint32_t k = i; k < j; ++k, ++leafes) {
            const Aabb3D* box = bboxes + boxIds[k];
            rcVcopy(leafes->min, box->min);
            rcVcopy(leafes->max, box->max);
        }
        while (leafes < dat + m_curSize) {
            leafes->min[0] = leafes->min[1] = leafes->min[2] =
                std::numeric_limits<float>::max();
            leafes->max[0] = leafes->max[1] = leafes->max[2] =
                -std::numeric_limits<float>::max();
            ++leafes;
        }
        uint32_t k = nUpPow2 >> 1;
        while (k) {
            for (uint32_t l = k, n = k << 1; l < n; ++l) {
                rcVcopy(dat[l].min, dat[l * 2].min);
                rcVmin(dat[l].min, dat[l * 2 + 1].min);
                rcVcopy(dat[l].max, dat[l * 2].max);
                rcVmax(dat[l].max, dat[l * 2 + 1].max);
            }
            k >>= 1;
        }

        return true;
    }

    void calcBbox(uint32_t i, uint32_t j, float* min, float* max) const
    {
        min[0] = min[1] = min[2] = std::numeric_limits<float>::max();
        max[0] = max[1] = max[2] = -std::numeric_limits<float>::max();
        uint32_t leafIndex = m_curSize >> 1;
        i = i - m_i + leafIndex;
        j = j - m_i + leafIndex;
        if (i == j) {
            rcVmin(min, m_curDat[i].min);
            rcVmax(max, m_curDat[i].max);
            return;
        }
        while (i < j) {
            if (i % 2 != 0) {
                rcVmin(min, m_curDat[i].min);
                rcVmax(max, m_curDat[i].max);
                ++i;
            }
            if (j % 2 == 0) {
                rcVmin(min, m_curDat[j].min);
                rcVmax(max, m_curDat[j].max);
                --j;
            }
            i >>= 1;
            j >>= 1;
            if (i == j) {
                rcVmin(min, m_curDat[i].min);
                rcVmax(max, m_curDat[i].max);
            }
        }
    }

private:
    int m_i = -1;
    int m_j = -1;
    uint32_t m_curSize = 0;
    const BminBmax* m_curDat = nullptr;
};

Grid2dBvh::Grid2dBvh() {
    m_worldMin[0] = m_worldMin[1] = m_worldMin[2] = std::numeric_limits<float>::max();
    m_worldMax[0] = m_worldMax[1] = m_worldMax[2] = -std::numeric_limits<float>::max();
}

Grid2dBvh::~Grid2dBvh () {
    release();
}

bool Grid2dBvh::isLoaded() const {
	return m_grid;
}

void Grid2dBvh::release() {
    delete [] m_grid;
    m_grid = nullptr;
    delete [] m_tris;
    m_tris = nullptr;
    delete [] m_triFlags;
    m_triFlags = nullptr;
    //delete [] m_verts;
	freeAlignedArr<float>(m_verts, 0);
    m_verts = nullptr;
	delete [] m_vobs;
	m_vobs = nullptr;
	delete [] m_vobsMeshes;
	m_vobsMeshes = nullptr;
    delete [] m_markedAreas.data;
    m_markedAreas.data = nullptr;
}

static void calcProjection(
	const float* points, int n, const float* axis, float (&minMax)[2]
) {
	assert(n >= 1);
	minMax[0] = rcVdot(points, axis);
	minMax[1] = minMax[0];
	for (int i = 1; i < n; ++i) {
		float val = rcVdot(points + i * 3, axis);
		minMax[0] = rcMin(minMax[0], val);
		minMax[1] = rcMax(minMax[1], val);
	}
}

static void makeAabbPoints(float* aabbPoints, const float* bmin, const float* bmax)
{
	float dx = bmax[0] - bmin[0];
	float dy = bmax[1] - bmin[1];
	float dz = bmax[2] - bmin[2];
	float* pos = aabbPoints;
	rcVcopy(pos, bmin); // 0
	pos += 3; //1
	rcVcopy(pos, bmin);
	pos[2] += dz;
	pos += 3; // 2
	rcVcopy(pos, bmin);
	pos[0] += dx;
	pos[2] += dz;
	pos += 3; // 3
	rcVcopy(pos, bmin);
	pos[0] += dx;
	pos += 3; // 4
	rcVcopy(pos, bmin);
	pos[1] += dy;
	pos += 3; // 5
	rcVcopy(pos, bmin);
	pos[1] += dy;
	pos[2] += dz;
	pos += 3; // 6
	rcVcopy(pos, bmin);
	pos[0] += dx;
	pos[1] += dy;
	pos[2] += dz;
	pos += 3; // 7
	rcVcopy(pos, bmin);
	pos[0] += dx;
	pos[1] += dy;
}

static bool intersectionAabbVsTriangle(
	const float* bmin, const float* bmax, const float* v0, const float* v1, const float* v2
) {
	// SAT
	float axis[3];
	float boxEdge0[3] = {1, 0, 0};
	float boxEdge1[3] = {0, 1, 0};
	float boxEdge2[3] = {0, 0, 1};
	float triEdge0[3], triEdge1[3], triEdge2[3];
	rcVsub(triEdge0, v0, v1);
	rcVsub(triEdge1, v1, v2);
	rcVsub(triEdge2, v2, v0);
	static const int AABB_NPOINTS = 8;
	static const int TRI_NPOINTS = 3;
	float triPoints[3 * TRI_NPOINTS];
	rcVcopy(triPoints, v0);
	rcVcopy(triPoints + 3, v1);
	rcVcopy(triPoints + 6, v2);
	float aabbPoints[3 * AABB_NPOINTS];
	makeAabbPoints(aabbPoints, bmin, bmax);
	float minMaxAabb[2];
	float minMaxTri[2];

	// 9 crosses of aabb edges - tri edges
	float boxEdges[3 * 3];
	rcVcopy(boxEdges, boxEdge0);
	rcVcopy(boxEdges + 3, boxEdge1);
	rcVcopy(boxEdges + 6, boxEdge2);
	float triEdges[3 * 3];
	rcVcopy(triEdges, triEdge0);
	rcVcopy(triEdges + 3, triEdge1);
	rcVcopy(triEdges + 6, triEdge2);
	for (int j = 0; j < 9; j += 3) {
		for (int i = 0; i < 9; i += 3) {
			rcVcross(axis, boxEdges + i, triEdges + j);
			calcProjection(aabbPoints, 8, axis, minMaxAabb);
			calcProjection(triPoints, 3, axis, minMaxTri);
			if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;
		}
	}

	// 3 box edges
	calcProjection(aabbPoints, 8, boxEdge0, minMaxAabb);
	calcProjection(triPoints, 3, boxEdge0, minMaxTri);
	if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;
	calcProjection(aabbPoints, 8, boxEdge1, minMaxAabb);
	calcProjection(triPoints, 3, boxEdge1, minMaxTri);
	if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;
	calcProjection(aabbPoints, 8, boxEdge2, minMaxAabb);
	calcProjection(triPoints, 3, boxEdge2, minMaxTri);
	if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;

	// tri normal
	rcVcross(axis, triEdge0, triEdge1);
	calcProjection(aabbPoints, 8, axis, minMaxAabb);
	calcProjection(triPoints, 3, axis, minMaxTri);
	if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;

	return true;
}

void Grid2dBvh::calc3DAabb(const float* verts, const int* triangle, Aabb3D* bbox, int vertsBlock)
{
    rcVcopy(bbox->min, &verts[triangle[0] * vertsBlock]);
    rcVcopy(bbox->max, &verts[triangle[0] * vertsBlock]);
    for (int i = 1; i < 3; ++i) {
        rcVmin(bbox->min, &verts[triangle[i] * vertsBlock]);
        rcVmax(bbox->max, &verts[triangle[i] * vertsBlock]);
    }
}

bool Grid2dBvh::checkTriangleBelongAabb (
    const float* bmin, const float* bmax, const int* triangle
) const {
    const float* v0 = &m_verts[triangle[0] * CUR_VERTS_BLOCK];
    const float* v1 = &m_verts[triangle[1] * CUR_VERTS_BLOCK];
    const float* v2 = &m_verts[triangle[2] * CUR_VERTS_BLOCK];
	return intersectionAabbVsTriangle(bmin, bmax, v0, v1, v2);
}

void Grid2dBvh::fillPolyFlags(
	PolyAreaFlags::FlagType* flagsTo,
	const PolyAreaFlags::FlagType* flagsFrom,
	int trisNum
) {
	for (int i = 0; i < trisNum; ++i)
    {
		flagsTo[i] = flagsFrom[i];
		if (flagsFrom[i].isVobPos) {
			continue;
		}
		uint8_t valueCollision = 0;
		switch (flagsFrom[i].polyFlags)
        {
			case PolyAreaFlags::WATER_COMMON:
			case PolyAreaFlags::WATER_SHALLOW:
			case PolyAreaFlags::WATER_MIDDLE:
			case PolyAreaFlags::WATER_DEEP:
				valueCollision = PolyFlagsCollision::WATER;
                break;
			case PolyAreaFlags::GROUND:
			case PolyAreaFlags::LADDER:
			case PolyAreaFlags::DOOR:
			case PolyAreaFlags::FOREST:
			case PolyAreaFlags::ROAD:
				valueCollision = PolyFlagsCollision::SOLID;
                break;
			case PolyAreaFlags::LAVA:
				valueCollision = PolyFlagsCollision::LAVA;
                break;
			default:
				//*((int*)nullptr) = 123;
				assert(1 != 1);
        }
		flagsTo[i].vobIdOrCollFlags = valueCollision;
	}
}

int Grid2dBvh::constructVobs(rcMeshLoaderObjExt* mesh)
{
	if (!m_moverNameToVob.init(mesh->getMoversCnt() + 1)) {
		return ERROR_NO_MEMORY;
	}
#ifdef PRINT_STRUCTURE_STAT
    m_bytesForData += static_cast<int>(m_moverNameToVob.getMemSize());
#endif
	m_vobsNum = mesh->getVobsCnt();
	if (!m_vobsNum) {
		return SUCCESSFUL;
	}
    m_vobs = new(std::nothrow) VobEntry[m_vobsNum];
	if (!m_vobs) {
		return ERROR_NO_MEMORY;
	}
	m_vobsMeshesNum = mesh->getVobMeshesCnt();
    m_vobsMeshes = new(std::nothrow) MeshEntry[m_vobsMeshesNum];
	if (!m_vobsMeshes) {
		return ERROR_NO_MEMORY;
	}

    std::unique_ptr<int[]> sizes(new(std::nothrow) int[m_cellsNum]);
	if (!sizes) {
		return ERROR_NO_MEMORY;
	}
	std::memset(sizes.get(), 0, sizeof(int) * m_cellsNum);
	const VobEntry* vobs = mesh->getVobs();
	for (int i = 0, n = mesh->getVobsCnt(); i < n; ++i) {
		const VobEntry& vobFrom = vobs[i];
		VobEntry& vobTo = m_vobs[i];
		if (!vobFrom.copy(vobTo)) {
			return ERROR_NO_MEMORY;
		}
		for (int j = 0; j < vobTo.posCnt; ++j) {
			rcMeshLoaderObjExt::Position& pos = vobTo.positions[j];
			float* mat = pos.invTrafo;
			mat[2] *= -1.f;
			mat[6] *= -1.f;
			mat[10] *= -1.f;
		}
		if (vobFrom.isMover()) {
            assert(vobFrom.vobName);
            m_moverNameToVob.put(vobFrom.vobName, i);
		}

		for (int j = 0, nPos = vobFrom.posCnt; j < nPos; ++j) {
			const auto& pos = vobFrom.positions[j];
			XzGridBorders ret = calcXzGridBorders(pos.aabbMin, pos.aabbMax);

			for (int k = ret.xiMin; k <= ret.xiMax; ++k) {
				for (int l = ret.ziMin; l <= ret.ziMax; ++l) {
					const int p = m_wszCellsZ * k + l;
					GridCell& cell = m_grid[p];
					for (int m = 0; m < cell.vobResidencesNum; ++m) {
						if (cell.vobResidence[m].vobIndex == i) {
							continue;
						}
					}

					assert(cell.vobResidencesNum <= sizes[p]);
					if (cell.vobResidencesNum == sizes[p]) {
						sizes[p] = sizes[p] * 2 + 1;
                        auto tmp = new(std::nothrow) GridCell::VobPosResidence[sizes[p]];
						if (!tmp) {
							return ERROR_NO_MEMORY;
						}
						std::memcpy(
							tmp,
							cell.vobResidence,
							cell.vobResidencesNum * sizeof(GridCell::VobPosResidence)
						);
						delete [] cell.vobResidence;
						cell.vobResidence = tmp;
					}
					cell.vobResidence[cell.vobResidencesNum].vobIndex = i;
					++cell.vobResidencesNum;
				}
			}
		}
	}

	//remove excess memory
	for (int i = 0; i < m_cellsNum; ++i) {
		GridCell& cell = m_grid[i];
		if (!cell.vobResidencesNum) {
			continue;
		}
        auto tmp = new(std::nothrow) GridCell::VobPosResidence[cell.vobResidencesNum];
		if (!tmp) {
			return ERROR_NO_MEMORY;
		}
		std::memcpy(
			tmp,
			cell.vobResidence,
			sizeof(GridCell::VobPosResidence) * cell.vobResidencesNum
		);
		delete [] cell.vobResidence;
		cell.vobResidence = tmp;
#ifdef PRINT_STRUCTURE_STAT
		m_bytesForData += sizeof(GridCell::VobPosResidence) * cell.vobResidencesNum;
#endif
	}

	int maxTrisNum = mesh->getMaxTrisPerVob();
    std::unique_ptr<Aabb3D[]> bboxes(new(std::nothrow) Aabb3D[maxTrisNum]);
    std::unique_ptr<int[]> boxIds(new(std::nothrow) int[maxTrisNum]);
	if (!bboxes || !boxIds) {
		return ERROR_NO_MEMORY;
	}
	const rcMeshLoaderObjExt::MeshEntry* vobsMesh = mesh->getVobMeshes();
	for (int i = 0; i < m_vobsMeshesNum; ++i) {
		const rcMeshLoaderObjExt::MeshEntry& vm = vobsMesh[i];
		rcMeshLoaderObjExt::MeshEntry& vmCp = m_vobsMeshes[i].mesh;
		if (vm.isEmpty()) {
			// TODO log
			continue;
		}
        if (!vm.copy(vmCp)) {
			return ERROR_NO_MEMORY;
		}
        int* tris = vm.tris.get();
		const int trisNum = vmCp.triCount;
		for (int j = 0; j < trisNum; ++j)
		{
            calc3DAabb(vm.verts, tris + j * 3, bboxes.get() + j, REGULAR_VERTS_BLOCK);
			bboxes[j].polyIndex = j;
			boxIds[j] = j;
		}
		auto dat = makeBvh(bboxes.get(), boxIds.get(), trisNum);
		if (!dat.first) {
			return ERROR_NO_MEMORY;
		}
		m_vobsMeshes[i].childs = dat.first;
		m_vobsMeshes[i].childsNumber = dat.second;
		fillPolyFlags(vmCp.flags.get(), vm.flags.get(), trisNum);
	}

#ifdef PRINT_STRUCTURE_STAT
	m_bytesPerConstruction +=
		sizeof(int) * m_cellsNum + (sizeof (Aabb3D) + sizeof(int)) * maxTrisNum;
	m_bytesForData += sizeof(VobEntry) * m_vobsNum + sizeof(MeshEntry) * m_vobsMeshesNum;
#endif

	return SUCCESSFUL;
}

void Grid2dBvh::transformVertex(const float* vertex, const float* trafo, float* vertexNew)
{
	// 0: 0-3
	// 0: 4-7
	// 0: 8-11
	// 0: 12-15
	vertexNew[0] =
		trafo[0]*vertex[0] + trafo[1]*vertex[1] + trafo[2]*vertex[2] + trafo[3]/**1.f*/;
	vertexNew[1] =
		trafo[4]*vertex[0] + trafo[5]*vertex[1] + trafo[6]*vertex[2] + trafo[7]/**1.f*/;
	vertexNew[2] =
		trafo[8]*vertex[0] + trafo[9]*vertex[1] + trafo[10]*vertex[2] + trafo[11]/**1.f*/;
}

void Grid2dBvh::transformVertex(
	const float* vertex, const rcMeshLoaderObjExt::Position* pos, float* vertexNew
) {
	if (pos)
		transformVertex(vertex, pos->trafo, vertexNew);
	else
		rcVcopy(vertexNew, vertex);
}

void Grid2dBvh::transformDirection(const float* normal, const float* trafo, float* normalNew)
{
	// 0: 0-3
	// 0: 4-7
	// 0: 8-11
	// 0: 12-15
	normalNew[0] =
		trafo[0]*normal[0] + trafo[1]*normal[1] + trafo[2]*normal[2]/* + trafo[3]*0.f*/;
	normalNew[1] =
		trafo[4]*normal[0] + trafo[5]*normal[1] + trafo[6]*normal[2]/* + trafo[7]*0.f*/;
	normalNew[2] =
		trafo[8]*normal[0] + trafo[9]*normal[1] + trafo[10]*normal[2]/* + trafo[11]*0.f*/;
	if (normalNew[1] < 0.f) {
		normalNew[0] *= -1.f;
		normalNew[1] *= -1.f;
		normalNew[2] *= -1.f;
	}
}

int Grid2dBvh::constructRenderingData(rcMeshLoaderObjExt* mesh)
{
	if (!mesh->isEnabledRendering())
	{
		return SUCCESSFUL;
	}

	m_mesh = mesh;
	m_renderingData.vertsNum = mesh->getVertCountStatic();
	m_renderingData.trisNum = mesh->getTriCountStatic();
	int vobsVertsNum = 0, vobsTrisNum = 0;
	for (int i = 0; i < m_vobsNum; ++i)
	{
		const VobEntry& vob = m_vobs[i];
		const MeshEntry& vmesh = m_vobsMeshes[vob.meshIndex];
		vobsVertsNum += vmesh.mesh.vertCount;
		vobsTrisNum += vmesh.mesh.triCount;
	}
	m_renderingData.vertsNum += vobsVertsNum;
	m_renderingData.trisNum += vobsTrisNum;
	m_renderingData.vertsNumCurrent = m_renderingData.vertsNum;
	m_renderingData.trisNumCurrent = m_renderingData.trisNum;
    m_renderingData.verts = new(std::nothrow) float[3 * m_renderingData.vertsNum];
    m_renderingData.tris = new(std::nothrow) int[3 * m_renderingData.trisNum];
    m_renderingData.normals = new(std::nothrow) float[3 * m_renderingData.trisNum];
    m_renderingData.triFlags = new(std::nothrow) uint8_t[m_renderingData.trisNum];
	if (
		!m_renderingData.verts || !m_renderingData.tris ||
		!m_renderingData.normals || !m_renderingData.triFlags
	) {
		return ERROR_NO_MEMORY;
	}

	// copy rendering data
	int vertsPos = mesh->getVertCountStatic() * 3;
	int deltaVerts = mesh->getVertCountStatic();
	int trisPos = mesh->getTriCountStatic() * 3;
	int flagsPos = mesh->getTriCountStatic();

    for (int i = 0, vPos = 0; i < vertsPos; i += 3, vPos += CUR_VERTS_BLOCK) {
		rcVcopy(m_renderingData.verts + i, m_verts + vPos);
	}
	std::memcpy(m_renderingData.tris, m_tris, trisPos * sizeof(int));
	for (int i = 0; i < flagsPos; ++i) {
        m_renderingData.triFlags[i] =
            m_triFlags[i].polyFlags |
            m_triFlags[i].isTriangle << PolyAreaFlags::IS_TRI_POS;
	}
	std::memcpy(m_renderingData.normals, mesh->getNormals(), trisPos * sizeof(float));

	for (int i = 0; i < m_vobsNum; ++i)
	{
		VobEntry& vob = m_vobs[i];
		const auto& vobMesh = m_vobsMeshes[vob.meshIndex].mesh;
		const auto& vobPos = vob.positions[vob.activePosIndex];
		const float* verts = vobMesh.verts;
		const int* tris = vobMesh.tris.get();
		const PolyAreaFlags::FlagType* flags = vobMesh.flags.get();
		const float* normals = vobMesh.normals.get();
		vob.vertsPosRendering = vertsPos / 3;

		for (int j = 0; j < vobMesh.vertCount; ++j, vertsPos += 3) {
			transformVertex(
                verts + j * CUR_VERTS_BLOCK, vobPos.trafo, m_renderingData.verts + vertsPos
			);
			(m_renderingData.verts + vertsPos)[2] *= -1.f;
		}
		for (
			int j = 0, m = vobMesh.triCount * 3;
			j < m;
			j += 3, trisPos += 3, ++flagsPos, ++flags
		) {
            m_renderingData.tris[trisPos] = tris[j] / CUR_VERTS_BLOCK + deltaVerts;
            m_renderingData.tris[trisPos + 1] = tris[j + 1] / CUR_VERTS_BLOCK + deltaVerts;
            m_renderingData.tris[trisPos + 2] = tris[j + 2] / CUR_VERTS_BLOCK + deltaVerts;
			transformDirection(
				normals + j, vobPos.trafo, m_renderingData.normals + trisPos
            );
            m_renderingData.triFlags[flagsPos] =
                flags[0].polyFlags |
                flags[0].isTriangle << PolyAreaFlags::IS_TRI_POS;
		}
		deltaVerts += vobMesh.vertCount;
	}

	return SUCCESSFUL;
}

int Grid2dBvh::constructOverlappingRectData(
	std::unique_ptr<std::pair<int/*verts num*/, int/*tris num*/>[]> trisVertsPerCellStatic
) {
	int maxVerts = 0, maxTris = 0;
	for (int i = 0; i < m_cellsNum; ++i) {
		const GridCell& cell = m_grid[i];
		int vnum = 0, tnum = 0;
		std::tie(vnum, tnum) = trisVertsPerCellStatic[i];
		for (int j = 0; j < cell.vobResidencesNum; ++j) {
			int vobIndex = cell.vobResidence[j].vobIndex;
			const VobEntry& vob = m_vobs[vobIndex];
			const auto& mesh = m_vobsMeshes[vob.meshIndex].mesh;
			trisVertsPerCellStatic[i].first += mesh.vertCount;
			trisVertsPerCellStatic[i].second += mesh.triCount;
		}
		if (trisVertsPerCellStatic[i].first > maxVerts)
			maxVerts = trisVertsPerCellStatic[i].first;
		if (trisVertsPerCellStatic[i].second > maxTris)
			maxTris = trisVertsPerCellStatic[i].second;
	}

	m_overlappingRectData.vertsNum = maxVerts;
	m_overlappingRectData.trisNum = maxTris;
	// TODO replace with more clever algo
	if (maxTris * 3 > maxVerts) {
		m_overlappingRectData.vertsNum = maxTris * 3;
	}
    m_overlappingRectData.verts = new(std::nothrow) float[m_overlappingRectData.vertsNum * 3];
    m_overlappingRectData.tris = new(std::nothrow) int[m_overlappingRectData.trisNum * 3];
    m_overlappingRectData.triFlags = new(std::nothrow) uint8_t[m_overlappingRectData.trisNum];
    //m_overlappingRectData.normals = new(std::nothrow) float[m_overlappingRectData.trisNum * 3];
	if (
		!m_overlappingRectData.verts || !m_overlappingRectData.tris ||
		!m_overlappingRectData.triFlags/* || !m_overlappingRectData.normals*/
	) {
		return ERROR_NO_MEMORY;
	}
#ifdef PRINT_STRUCTURE_STAT
	m_bytesForData +=
		sizeof(float) * m_overlappingRectData.vertsNum * 3 +
		sizeof(int) * m_overlappingRectData.trisNum * 3 +
		sizeof(uint8_t) * m_overlappingRectData.trisNum;
#endif

	return SUCCESSFUL;
}

int Grid2dBvh::getOverlappingRectCellIds(
	const float* min, const float* max, int* cellIds, int idsSize
) const {
	XzGridBorders ret = calcXzGridBorders(min, max);
	int k = 0;
	for (int i = ret.xiMin; i <= ret.xiMax; ++i) {
		for (int j = ret.ziMin; j <= ret.ziMax; ++j) {
			if (k < idsSize) {
				cellIds[k] = m_wszCellsZ * i + j;
			}
			++k;
		}
	}
	return k;
}

int Grid2dBvh::getOverlappingRectMarkedAreaIds(
    const float* min, const float* max, int* markedAreaIds, int idsSize
) const {
    XzGridBorders ret = calcXzGridBorders(min, max);
    int n = 0;
    for (int i = ret.xiMin; i <= ret.xiMax; ++i) {
        for (int j = ret.ziMin; j <= ret.ziMax; ++j) {
            const GridCell& cell = m_grid[m_wszCellsZ * i + j];
            for (int k = 0; k < cell.markedNum; ++k) {
                if (n < idsSize) {
                    markedAreaIds[n] = cell.markedIndices[k];
                }
                ++n;
            }
        }
    }
    return n;
}

void Grid2dBvh::copyDataFromBvh (
	Grid2dBvh::TrianglesData& resData,
	int& trisPos,
	int& vertsPos,
	const BvhNode* curNode,
	const BvhNode* endNode,
	const rcMeshLoaderObjExt::Position* pos,
	const float* verts,
	const int* tris,
	const PolyAreaFlags::FlagType* triFlags
) {
	while (curNode < endNode) {
		if (curNode->triId >= 0) {
			PolyAreaFlags::FlagType flag = triFlags[curNode->triId / 3];
			if (
				flag.isVobPos &&
				(flag.polyFlags != PolyAreaFlags::DOOR) &&
				(flag.polyFlags != PolyAreaFlags::LADDER)
			) {
				++curNode;
				continue;
			}
			const int* vIds = tris + curNode->triId;
			const float* v1 = verts + vIds[0];
			const float* v2 = verts + vIds[1];
			const float* v3 = verts + vIds[2];
			resData.triFlags[trisPos / 3] =
				// need not is tri flag for navmesh generation
				static_cast<uint8_t>(flag.isInhabited) << PolyAreaFlags::INHABITED_POS |
				flag.polyFlags;
			transformVertex(v1, pos, resData.verts + vertsPos);
			if (pos)
				(resData.verts + vertsPos)[2] *= -1.f;
			resData.tris[trisPos++] = vertsPos / 3;
			vertsPos += 3;
			transformVertex(v2, pos, resData.verts + vertsPos);
			if (pos)
				(resData.verts + vertsPos)[2] *= -1.f;
			resData.tris[trisPos++] = vertsPos / 3;
			vertsPos += 3;
			transformVertex(v3, pos, resData.verts + vertsPos);
			if (pos)
				(resData.verts + vertsPos)[2] *= -1.f;
			resData.tris[trisPos++] = vertsPos / 3;
			vertsPos += 3;
		}
		++curNode;
	}
}

const Grid2dBvh::TrianglesData& Grid2dBvh::getEmptyOverlappingRectData() const
{
	return m_overlappingRectData;
}

const Grid2dBvh::TrianglesData& Grid2dBvh::extractOverlappingRectData(int cellId) const
{
	extractOverlappingRectData(cellId, m_overlappingRectData);
	return m_overlappingRectData;
}

void Grid2dBvh::extractOverlappingRectData(
	int cellId, Grid2dBvh::TrianglesData& customData
) const {
	int trisPos = 0;
	int vertsPos = 0;
	const GridCell& grid = m_grid[cellId];
	const BvhNode* curNode = grid.childs;
	const BvhNode* endNode = curNode + grid.childsNumber;

	copyDataFromBvh(
		customData, trisPos, vertsPos, curNode, endNode, nullptr, m_verts, m_tris, m_triFlags
	);
	for (int k = 0; k < grid.vobResidencesNum; ++k) {
		const VobEntry& vob = m_vobs[grid.vobResidence[k].vobIndex];
		if (vob.isDoor() || vob.isLadder()) {
			continue;
		}
		const MeshEntry& vMesh = m_vobsMeshes[vob.meshIndex];
		const rcMeshLoaderObjExt::MeshEntry& realMesh = vMesh.mesh;
		assert(vob.activePosIndex < vob.posCnt);
		const rcMeshLoaderObjExt::Position* pos = &vob.positions[vob.activePosIndex];
		curNode = vMesh.childs;
		endNode = curNode + vMesh.childsNumber;
		// TODO replace to explicit extracting from mesh of resident vobs
		copyDataFromBvh(
			customData, trisPos, vertsPos, curNode, endNode, pos,
			realMesh.verts, realMesh.tris.get(), realMesh.flags.get()
		);
	}
	customData.vertsNumCurrent = vertsPos / 3;
	customData.trisNumCurrent = trisPos / 3;
}

const Grid2dBvh::TrianglesData& Grid2dBvh::getRenderingData() const
{
	return m_renderingData;
}

void Grid2dBvh::moverStateUpdate(const char* name, int stateId)
{
	static const int MOVERS_NUM = 32;
	size_t n = 0;
	int moverIds[MOVERS_NUM];
	if (!(n = m_moverNameToVob.find<MOVERS_NUM>(name))) {
		// TODO, debug log
		return;
	}
	if (n != m_moverNameToVob.get(name, moverIds)) {
		// TODO, debug log
		return;
	}

	for (size_t i = 0; i < n; ++i) {
		VobEntry& vob = m_vobs[moverIds[i]];
		assert(stateId < vob.posCnt);
		const auto& posOld = vob.positions[vob.activePosIndex];
		for (int j = 0; j < rcMeshLoaderObjExt::Position::POS_TRIS_NUM; ++j) {
			m_triFlags[posOld.aabbTris[i]].isActiveVobPos = false;
		}
		vob.activePosIndex = stateId;
		const auto& posNew = vob.positions[vob.activePosIndex];
		for (int j = 0; j < rcMeshLoaderObjExt::Position::POS_TRIS_NUM; ++j) {
			m_triFlags[posNew.aabbTris[i]].isActiveVobPos = true;
		}

		const auto& vobMesh = m_vobsMeshes[vob.meshIndex].mesh;
		const float* verts = vobMesh.verts;
		float* resVerts = &m_renderingData.verts[vob.vertsPosRendering * 3];
		for (int j = 0, n = vobMesh.vertCount * 3; j < n; j += 3, resVerts += 3) {
			transformVertex(verts + j, posNew.trafo, resVerts);
		}
	}
}

Grid2dBvh::XzGridBorders Grid2dBvh::calcXzGridBorders(const float* min, const float* max) const
{
	int xiMin = static_cast<int>((min[0] - m_worldMin[0]) * m_cellSizeInv);
	int xiMax = static_cast<int>((max[0] - m_worldMin[0]) * m_cellSizeInv);
	if (xiMin > xiMax) std::swap(xiMin, xiMax);
	int ziMin = static_cast<int>((min[2] - m_worldMin[2]) * m_cellSizeInv);
	int ziMax = static_cast<int>((max[2] - m_worldMin[2]) * m_cellSizeInv);
	if (ziMin > ziMax) std::swap(ziMin, ziMax);
	if (xiMin < 0) xiMin = 0;
	if (ziMin < 0) ziMin = 0;
	if (xiMax >= m_wszCellsX) xiMax = m_wszCellsX - 1;
	if (ziMax >= m_wszCellsZ) ziMax = m_wszCellsZ - 1;
	return {xiMin, xiMax, ziMin, ziMax};
}

static void calcProjectionXz(
	const float* points, int n, const float* axis, float (&minMax)[2]
) {
	assert(n >= 1);
	minMax[0] = rcVdotXz(points, axis);
	minMax[1] = minMax[0];
	for (int i = 1; i < n; ++i) {
		float val = rcVdotXz(points + i * 3, axis);
		minMax[0] = rcMin(minMax[0], val);
		minMax[1] = rcMax(minMax[1], val);
	}
}

static bool checkPolyVsPolyXz(
	const float* first, int firstNum, const float* second, int secondNum
) {
	float e[3], norm[3];
	float minMaxFirst[2], minMaxSecond[2];

	// SAT
    for (int i = firstNum - 1, j = 0; i != firstNum - 1; i = j, ++j) {
		rcVsub(e, first + i * 3, first + j * 3);
		norm[0] = e[2];
		norm[1] = 0.f;
		norm[2] = -e[0];
		calcProjectionXz(first, firstNum, norm, minMaxFirst);
		calcProjectionXz(second, secondNum, norm, minMaxSecond);
		if (minMaxFirst[1] < minMaxSecond[0] || minMaxSecond[1] < minMaxFirst[0]) {
			return false;
		}
	}
    for (int i = secondNum - 1, j = 0; i != secondNum - 1; i = j, ++j) {
		rcVsub(e, second + i * 3, second + j * 3);
		norm[0] = e[2];
		norm[1] = 0.f;
		norm[2] = -e[0];
        calcProjectionXz(first, firstNum, norm, minMaxFirst);
		calcProjectionXz(second, secondNum, norm, minMaxSecond);
		if (minMaxFirst[1] < minMaxSecond[0] || minMaxSecond[1] < minMaxFirst[0]) {
			return false;
		}
	}

	return true;
}

int Grid2dBvh::linkMarkedAreaWithGrid(
	int vertsNum, const float* verts, int eInd, MarkedEntry& e
) {
    static const int IDS_DELTA = 4;
    float bbox[3 * 4];
	float min[3], max[3];
	rcVcopy(min, verts);
	rcVcopy(max, verts);
	for (int j = 3; j < vertsNum * 3; j += 3) {
		rcVmin(min, verts + j);
		rcVmax(max, verts + j);
	}
	for (int i = 0; i < 4; ++i) {
		rcVcopy(bbox + i * 3, min);
	}
	float dx = max[0] - min[0];
	float dz = max[2] - min[2];
	(bbox + 3)[0] += dx;
	(bbox + 6)[0] += dx;
	(bbox + 6)[2] += dz;
	(bbox + 9)[2] += dz;

	const auto& minMax = calcXzGridBorders(min, max);
	for (int k = minMax.xiMin; k <= minMax.xiMax; ++k) {
		for (int l = minMax.ziMin; l <= minMax.ziMax; ++l) {
			int index = m_wszCellsZ * k + l;
			GridCell& cell = m_grid[index];
			if (checkPolyVsPolyXz(bbox, 4, verts, vertsNum)) {
				if (cell.markedSize == cell.markedNum) {
                    cell.markedSize += IDS_DELTA;
                    int* newIndices = new(std::nothrow) int[cell.markedSize];
					if (!newIndices) {
						return ERROR_NO_MEMORY;
					}
#ifdef PRINT_STRUCTURE_STAT
                    m_bytesForData += sizeof(int) * IDS_DELTA;
#endif
					std::memcpy(newIndices, cell.markedIndices, sizeof(int) * cell.markedNum);
					delete [] cell.markedIndices;
					cell.markedIndices = newIndices;
				}
				cell.markedIndices[cell.markedNum] = eInd;
				cell.markedNum += 1;

				if (e.idsNum == e.idsSize) {
                    e.idsSize += IDS_DELTA;
                    int* newIds = new(std::nothrow) int[e.idsSize];
					if (!newIds) {
						return ERROR_NO_MEMORY;
					}
#ifdef PRINT_STRUCTURE_STAT
                    m_bytesForData += sizeof(int) * IDS_DELTA;
#endif
					std::memcpy(newIds, e.gridIds, sizeof(int) * e.idsNum);
					delete [] e.gridIds;
					e.gridIds = newIds;
				}
				e.gridIds[e.idsNum] = index;
				e.idsNum += 1;
			}
		}
	}

	return SUCCESSFUL;
}

int Grid2dBvh::constructMarkedAreas(rcMeshLoaderObjExt* mesh)
{
	const int mNum = mesh->getMarkedCount();
	const rcMeshLoaderObjExt::MarkedEntry* marked = mesh->getMarked();
	return constructMarkedAreas(mNum, marked);
}

int Grid2dBvh::constructMarkedAreas(int mNum, const rcMeshLoaderObjExt::MarkedEntry* marked)
{
	int vertsNum = 0;
	for (int i = 0; i < mNum; ++i) {
		vertsNum += marked[i].vertsNum;
	}
    MarkedEntry* data = new(std::nothrow) MarkedEntry[mNum];
	if (!data) {
		return ERROR_NO_MEMORY;
	}
#ifdef PRINT_STRUCTURE_STAT
    m_bytesForData += sizeof(MarkedEntry) * mNum;
#endif

	for (int i = 0; i < mNum; ++i) {
		const rcMeshLoaderObjExt::MarkedEntry& from = marked[i];
		rcMeshLoaderObjExt::MarkedEntry& to = data[i].data;
		to.minh = from.minh;
		to.maxh = from.maxh;
		to.area = from.area;
		to.vertsNum = from.vertsNum;
        to.verts = new(std::nothrow) float[3 * to.vertsNum];
		if (!to.verts) {
			delete [] data;
			return ERROR_NO_MEMORY;
		}
#ifdef PRINT_STRUCTURE_STAT
        m_bytesForData += sizeof(float) * 3 * to.vertsNum;
#endif
		std::memcpy(to.verts, from.verts, 3 * sizeof(float) * to.vertsNum);
		int ret = linkMarkedAreaWithGrid(to.vertsNum, to.verts, i, data[i]);
		if (ret != SUCCESSFUL) {
			delete [] data;
			return ret;
		}
	}
    m_markedAreas.num = mNum;
    m_markedAreas.size = mNum;
    m_markedAreas.data = data;

	return SUCCESSFUL;
}

const rcMeshLoaderObjExt::MarkedEntry* Grid2dBvh::getMarkedArea(int i) const
{
    assert(i < m_markedAreas.num);
    return &m_markedAreas.data[i].data;
}

int Grid2dBvh::getMarkedAreaSize() const
{
    return m_markedAreas.num;
}

int Grid2dBvh::addMarkedArea(
	const float* verts,
	const int nverts,
	const float minh,
	const float maxh,
	int area
) {
    if (m_markedAreas.num == m_markedAreas.size) {
        m_markedAreas.size += 4;
        MarkedEntry* marked = new(std::nothrow) MarkedEntry[m_markedAreas.size];
		if (!marked) {
			return ERROR_NO_MEMORY;
		}
        for (int i = 0; i < m_markedAreas.num; ++i) {
            int ret = m_markedAreas.data[i].copy(marked[i]);
			if (ret != SUCCESSFUL) {
				delete [] marked;
				return ret;
			}
		}
        delete [] m_markedAreas.data;
        m_markedAreas.data = marked;
	}

    MarkedEntry& newArea = m_markedAreas.data[m_markedAreas.num];
	rcMeshLoaderObjExt::MarkedEntry& data = newArea.data;
    data.verts = new(std::nothrow) float[3 * nverts];
	if (!data.verts) {
		return ERROR_NO_MEMORY;
	}
	data.vertsNum = nverts;
	std::memcpy(data.verts, verts, sizeof(float) * 3 * data.vertsNum);
	data.minh = minh;
	data.maxh = maxh;
	data.area = area;
    int ret = linkMarkedAreaWithGrid(data.vertsNum, data.verts, m_markedAreas.num, newArea);
	if (ret != SUCCESSFUL) {
		return ret;
	}
    ++m_markedAreas.num;

	return SUCCESSFUL;
}

void Grid2dBvh::totalDeleteMarkedAreas()
{
    m_markedAreas.num = 0;
    m_markedAreas.size = 0;
    delete [] m_markedAreas.data;
    m_markedAreas.data = nullptr;
	for (int i = 0; i < m_cellsNum; ++i) {
		m_grid[i].markedNum = 0;
	}
}

void Grid2dBvh::deleteMarkedArea(int n)
{
    assert(n < m_markedAreas.num);
    MarkedEntry& marked = m_markedAreas.data[n];
	for (int i = 0; i < marked.idsNum; ++i) {
		int id = marked.gridIds[i];
		GridCell& grid = m_grid[id];
		for (int j = 0; j < grid.markedNum; ++j) {
			if (grid.markedIndices[j] != n) {
				continue;
			}
			std::memmove(
				grid.markedIndices + j,
				grid.markedIndices + j + 1,
				sizeof(int) * (grid.markedNum - j - 1)
			);
		}
		--grid.markedNum;
	}
	marked.~MarkedEntry();
    // TODO make shallow copying
	std::memmove(
        m_markedAreas.data + n,
        m_markedAreas.data + n + 1,
        sizeof(MarkedEntry) * (m_markedAreas.num - n - 1)
	);
    --m_markedAreas.num;
    std::memset(m_markedAreas.data + m_markedAreas.num, 0, sizeof(MarkedEntry));
}

int Grid2dBvh::constructOffmeshesOnLadders()
{
	static const int NUM_VERTS = 4;
	int num = 0;

	for (int i = 0; i < m_vobsNum; ++i)
	{
		VobEntry& vob = m_vobs[i];
		const MeshEntry& mesh = m_vobsMeshes[vob.meshIndex];
		if (!mesh.mesh.isEmpty() && vob.isLadder()) {
			++num;
		}
    }
    m_offMeshConns.offMeshSize = num;
    m_offMeshConns.offMeshVerts = new(std::nothrow) float[3 * 2 * num];
    m_offMeshConns.offMeshRads = new(std::nothrow) float[num];
    m_offMeshConns.offMeshDirs = new(std::nothrow) uint8_t[num];
    m_offMeshConns.offMeshAreas = new(std::nothrow) uint8_t[num];
    m_offMeshConns.offMeshFlags = new(std::nothrow) uint16_t[num];
    m_offMeshConns.offMeshId = new(std::nothrow) uint32_t[num];
	if (
		!m_offMeshConns.offMeshVerts || !m_offMeshConns.offMeshRads ||
		!m_offMeshConns.offMeshDirs || !m_offMeshConns.offMeshAreas ||
		!m_offMeshConns.offMeshFlags || !m_offMeshConns.offMeshId
	) {
		return ERROR_NO_MEMORY;
	}
#ifdef PRINT_STRUCTURE_STAT
    m_bytesForData += sizeof(float) * 3 * 2 * num + sizeof(float) * num +
                      sizeof(uint8_t) * num + sizeof(uint8_t) * num +
                      sizeof(uint16_t) * num + sizeof(uint32_t) * num;
#endif

    for (int i = 0; i < m_vobsNum; ++i)
	{
		VobEntry& vob = m_vobs[i];
		const MeshEntry& mesh = m_vobsMeshes[vob.meshIndex];
		if (mesh.mesh.isEmpty()) {
			//assert(1 != 1);
			// TODO log
			continue;
		}
		if (!vob.isLadder()) {
			continue;
		}

		const rcMeshLoaderObjExt::MeshEntry& realMesh = mesh.mesh;
		const float* verts = realMesh.verts;
		const rcMeshLoaderObjExt::Position& vobPos = vob.positions[vob.activePosIndex];
		int nVerts = realMesh.vertCount;
        std::unique_ptr<std::pair<float, int>[]> dat(
            new(std::nothrow) std::pair<float, int>[nVerts]
        );
		float vert[3];
		for (int j = 0; j < nVerts; ++j) {
            transformVertex(verts + j * CUR_VERTS_BLOCK, vobPos.trafo, vert);
			//vert[2] *= -1.f;
			dat[j].first = vert[1];
			dat[j].second = j;
		}
		std::sort(
			dat.get(),
			dat.get() + nVerts,
			[] (const std::pair<float, int>& v1, const std::pair<float, int>& v2) {
				return v1.first < v2.first;
			}
		);
		float low[3] = {0.f, 0.f, 0.f};
		float high[3] = {0.f, 0.f, 0.f};
		for (int j = 0, k = nVerts - NUM_VERTS; j < NUM_VERTS; ++j, ++k) {
            transformVertex(verts + dat[j].second * CUR_VERTS_BLOCK, vobPos.trafo, vert);
			vert[2] *= -1.f;
			low[0] += vert[0];
			low[1] += vert[1];
			low[2] += vert[2];
            transformVertex(verts + dat[k].second * CUR_VERTS_BLOCK, vobPos.trafo, vert);
			vert[2] *= -1.f;
			high[0] += vert[0];
			high[1] += vert[1];
			high[2] += vert[2];
		}
		low[0] *= 0.25f;
		low[1] = low[1] * 0.25f + 10.f;
		low[2] *= 0.25f;
		high[0] *= 0.25f;
		high[1] *= 0.25f;
		high[2] *= 0.25f;
        rcVcopy(m_offMeshConns.offMeshVerts + m_offMeshConns.offMeshNum * 3 * 2, low);
        rcVcopy(m_offMeshConns.offMeshVerts + m_offMeshConns.offMeshNum * 3 * 2 + 3, high);
        m_offMeshConns.offMeshRads[m_offMeshConns.offMeshNum] = 40.f;
        m_offMeshConns.offMeshDirs[m_offMeshConns.offMeshNum] = 1;
        m_offMeshConns.offMeshAreas[m_offMeshConns.offMeshNum] = SAMPLE_POLYAREA_GROUND;
        m_offMeshConns.offMeshFlags[m_offMeshConns.offMeshNum] = SAMPLE_POLYFLAGS_WALK;
        m_offMeshConns.offMeshId[m_offMeshConns.offMeshNum] = 1000 + m_offMeshConns.offMeshNum;
		m_offMeshConns.offMeshNum += 1;
	}

	return SUCCESSFUL;
}

const Grid2dBvh::OffMeshData& Grid2dBvh::getOffMeshData() const
{
	return m_offMeshConns;
}

int Grid2dBvh::addOffMeshConn(
	const float* spos,
	const float* epos,
	const float rad,
	unsigned char bidir,
	unsigned char area,
	unsigned short flags
) {
	static const int INCREMENT_VAL = 10;
	int curNum = m_offMeshConns.offMeshNum;

	if (m_offMeshConns.offMeshSize == curNum) {
		m_offMeshConns.offMeshSize += INCREMENT_VAL;
		int curSize = m_offMeshConns.offMeshSize;
        float* offMeshVerts = new(std::nothrow) float[2 * 3 * curSize];
        float* offMeshRads = new(std::nothrow) float[curSize];
        uint8_t* offMeshDirs = new(std::nothrow) uint8_t[curSize];
        uint8_t* offMeshAreas = new(std::nothrow) uint8_t[curSize];
        uint16_t* offMeshFlags = new(std::nothrow) uint16_t[curSize];
        uint32_t* offMeshId = new(std::nothrow) uint32_t[curSize];
		if (
			!offMeshVerts || !offMeshRads || !offMeshDirs ||
			!offMeshAreas || !offMeshFlags || !offMeshId
		) {
            delete [] offMeshVerts;
            delete [] offMeshRads;
            delete [] offMeshDirs;
            delete [] offMeshAreas;
            delete [] offMeshFlags;
            delete [] offMeshId;
            return ERROR_NO_MEMORY;
		}
		std::memcpy(offMeshVerts, m_offMeshConns.offMeshVerts, sizeof(float) * 2 * 3 * curNum);
		std::memcpy(offMeshRads, m_offMeshConns.offMeshRads, sizeof(float) * curNum);
		std::memcpy(offMeshDirs, m_offMeshConns.offMeshDirs, sizeof(float) * curNum);
		std::memcpy(offMeshAreas, m_offMeshConns.offMeshAreas, sizeof(uint8_t) * curNum);
		std::memcpy(offMeshFlags, m_offMeshConns.offMeshFlags, sizeof(uint16_t) * curNum);
		std::memcpy(offMeshId, m_offMeshConns.offMeshId, sizeof(uint32_t) * curNum);
		m_offMeshConns.offMeshVerts = offMeshVerts;
		m_offMeshConns.offMeshRads = offMeshRads;
		m_offMeshConns.offMeshDirs = offMeshDirs;
		m_offMeshConns.offMeshAreas = offMeshAreas;
		m_offMeshConns.offMeshFlags = offMeshFlags;
		m_offMeshConns.offMeshId = offMeshId;
	}
	rcVcopy(m_offMeshConns.offMeshVerts + curNum * 2 * 3, spos);
	rcVcopy(m_offMeshConns.offMeshVerts + curNum * 2 * 3 + 3, epos);
	m_offMeshConns.offMeshRads[curNum] = rad;
	m_offMeshConns.offMeshDirs[curNum] = bidir;
	m_offMeshConns.offMeshAreas[curNum] = area;
	m_offMeshConns.offMeshFlags[curNum] = flags;
	m_offMeshConns.offMeshId[curNum] = 1000 + curNum;
	m_offMeshConns.offMeshNum += 1;

	return SUCCESSFUL;
}

void Grid2dBvh::deleteOffMeshConn(int i)
{
	assert(m_offMeshConns.offMeshNum < i);
	int nShift = m_offMeshConns.offMeshNum - 1 - i;
	std::memmove(
		m_offMeshConns.offMeshVerts + 2 * 3 * i,
		m_offMeshConns.offMeshVerts + 2 * 3 * (i + 1),
		sizeof(float) * 2 * 3 * nShift
	);
	std::memmove(
		m_offMeshConns.offMeshRads + i,
		m_offMeshConns.offMeshRads + (i + 1),
		sizeof(float) * nShift
	);
	std::memmove(
		m_offMeshConns.offMeshDirs + i,
		m_offMeshConns.offMeshDirs + (i + 1),
		sizeof(uint8_t) * nShift
	);
	std::memmove(
		m_offMeshConns.offMeshAreas + i,
		m_offMeshConns.offMeshAreas + (i + 1),
		sizeof(uint8_t) * nShift
	);
	std::memmove(
		m_offMeshConns.offMeshFlags + i,
		m_offMeshConns.offMeshFlags + (i + 1),
		sizeof(uint16_t) * nShift
	);
	std::memmove(
		m_offMeshConns.offMeshId + i,
		m_offMeshConns.offMeshId + (i + 1),
		sizeof(uint32_t) * nShift
	);
	m_offMeshConns.offMeshNum -= 1;
}

int Grid2dBvh::load(rcContext* ctx, rcMeshLoaderObjExt* mesh, int cellSize)
{
    int ret = loadInternal(ctx, mesh, cellSize);
	if (ret != SUCCESSFUL) {
		release();
	}
	return ret;
}

void Grid2dBvh::getBounds(float* bMin, float* bMax) const
{
    rcVcopy(bMin, m_worldMin);
    rcVcopy(bMax, m_worldMax);
}

int Grid2dBvh::loadInternal(rcContext* ctx, rcMeshLoaderObjExt* mesh, int cellSize)
{
	m_cellSize = cellSize;
	m_cellSizeInv = 1.0f / m_cellSize;
#ifdef USAGE_SSE_1_0
    m_cellSizeInvVec =
        _mm_setr_ps(m_cellSizeInv, m_cellSizeInv, m_cellSizeInv, m_cellSizeInv);
#endif
	const float* vertsCp = mesh->getVerts();
	m_vertsNum = mesh->getVertCount();
	m_verts = allocAlignedArr<float>(m_vertsNum * CUR_VERTS_BLOCK, 16);
    if (!m_verts) {
		return ERROR_NO_MEMORY;
    }
    for (int i = 0; i < m_vertsNum; ++i) {
        std::memcpy(m_verts + CUR_VERTS_BLOCK * i, vertsCp + 3 * i, sizeof(float) * 3);
#ifdef USAGE_SSE_1_0
        m_verts[CUR_VERTS_BLOCK * i + 3] = 0.f;
#endif
    }
	m_trisNum = mesh->getTriCount();
    m_tris = new(std::nothrow) int[m_trisNum * 3];
    m_triFlags = new(std::nothrow) PolyAreaFlags::FlagType[m_trisNum];
    if (!m_tris || !m_triFlags) {
		return ERROR_NO_MEMORY;
    }
	std::memcpy(m_tris, mesh->getTris(), 3 * m_trisNum * sizeof(int));
	fillPolyFlags(m_triFlags, mesh->getFlags(), m_trisNum);
#ifdef PRINT_STRUCTURE_STAT
    m_bytesForData += sizeof(float) * CUR_VERTS_BLOCK * m_vertsNum +
                      sizeof(int) * 3 * m_trisNum +
					  sizeof(uint32_t) * m_trisNum;
#endif
    std::unique_ptr<Aabb3D[]> bboxes(new(std::nothrow) Aabb3D[m_trisNum]);
    std::unique_ptr<int[]> boxIds(new(std::nothrow) int[m_trisNum]);
    if (!bboxes || !boxIds) {
		return ERROR_NO_MEMORY;
    }
#ifdef PRINT_STRUCTURE_STAT
    m_bytesPerConstruction += m_trisNum * (sizeof(Aabb3D) + sizeof(int));
#endif
    for (int i = 0; i < m_trisNum; ++i)
    {
        calc3DAabb(m_verts, m_tris + i * 3, bboxes.get() + i, CUR_VERTS_BLOCK);
        rcVmin(m_worldMin, bboxes[i].min);
        rcVmax(m_worldMax, bboxes[i].max);
        bboxes[i].polyIndex = i;
        boxIds[i] = i;
    }
#ifdef USAGE_SSE_1_0
	m_worldMinVecXzXz = _mm_setr_ps(m_worldMin[0], m_worldMin[2], m_worldMin[0], m_worldMin[2]);
#endif
	rcVsub(m_worldSize, m_worldMax, m_worldMin);
	m_wszCellsX = static_cast<int>(std::ceil(m_worldSize[0] / m_cellSize));
	m_wszCellsY = 0;
	m_wszCellsZ = static_cast<int>(std::ceil(m_worldSize[2] / m_cellSize));
	m_cellsNum = m_wszCellsX * m_wszCellsZ;
    m_grid = new(std::nothrow) GridCell[m_cellsNum];
    if (!m_grid) {
		return ERROR_NO_MEMORY;
    }
#ifdef PRINT_STRUCTURE_STAT
    ctx->log(
        RC_LOG_WARNING,
        "World size bbox, dx: %f, dy: %f, dz: %f",
        m_worldSize[0], m_worldSize[1], m_worldSize[2]
    );
    ctx->log(
        RC_LOG_WARNING,
        "Bmin x: %f, y: %f, z: %f; "
        "Bmax x: %f, y: %f, z: %f",
        m_worldMin[0], m_worldMin[1], m_worldMin[2],
        m_worldMax[0], m_worldMax[1], m_worldMax[2]
    );
    ctx->log(
        RC_LOG_WARNING,
        "Size of vertices, polys and flags: %f\n",
        static_cast<float>(m_bytesForData) / (1024 * 1024)
    );
    m_bytesForData += m_cellsNum * sizeof(GridCell);
#endif
	if (SUCCESSFUL != constructVobs(mesh)) {
		return ERROR_NO_MEMORY;
	}
	if (SUCCESSFUL != constructRenderingData(mesh)) {
		return ERROR_NO_MEMORY;
	}
	if (SUCCESSFUL != constructMarkedAreas(mesh)) {
		return ERROR_NO_MEMORY;
	}
	if (SUCCESSFUL != constructOffmeshesOnLadders()) {
		return ERROR_NO_MEMORY;
	}

    // split by x axis
	std::unique_ptr<CellBoundingPair[]> cellMinMax(new(std::nothrow) CellBoundingPair[m_wszCellsX]);
	if (!cellMinMax) {
		return ERROR_NO_MEMORY;
	}
    std::unique_ptr<std::unique_ptr<int[]>[]>  xSplit(
        new(std::nothrow) std::unique_ptr<int[]>[m_wszCellsX]);
    if (!xSplit) {
		return ERROR_NO_MEMORY;
    }
#ifdef PRINT_STRUCTURE_STAT
    m_bytesPerConstruction += m_wszCellsX * (sizeof(std::unique_ptr<int[]>) + sizeof(CellBoundingPair));
#endif
    static const int PRIMARY_SIZE_X = 256;
    for (int i = 0; i < m_wszCellsX; ++i) {
        xSplit[i].reset(new(std::nothrow) int [PRIMARY_SIZE_X]);
        if (!xSplit[i]) {
			return ERROR_NO_MEMORY;
        }
        xSplit[i][0] = 2; // number of ids + shift
        xSplit[i][1] = PRIMARY_SIZE_X; // slab size
        float cellMin[3] = {
			m_worldMin[0] + i * m_cellSize, m_worldMin[1], m_worldMin[2]
        };
        float cellMax[3] = {
			m_worldMin[0] + (i + 1) * m_cellSize, m_worldMax[1], m_worldMax[2]
        };
        rcVcopy(cellMinMax[i].min, cellMin);
        rcVcopy(cellMinMax[i].max, cellMax);
    }
    std::sort(boxIds.get(), boxIds.get() + m_trisNum, CoordComparerOnIds<0>(bboxes.get()));
    for (int i = 0, j = 0; j < m_trisNum; /*++j*/)
    {
        int cellStart = i;
        while(true)
        {
            if (cellStart >= m_wszCellsX) {
                ++j;
                break;
            }
            bool aabbColl = checkAabbsCollision(
                cellMinMax[cellStart].min,
                cellMinMax[cellStart].max,
                bboxes[boxIds[j]].min,
                bboxes[boxIds[j]].max
            );
            if (!aabbColl) {
                if (cellStart == i) {
                    ++i;
                } else {
                    ++j;
                }
                break;
            }
            bool triColl = checkTriangleBelongAabb(
                cellMinMax[cellStart].min,
                cellMinMax[cellStart].max,
                &m_tris[bboxes[boxIds[j]].polyIndex * 3]
            );
            if (triColl) {
                if (xSplit[cellStart][0] == xSplit[cellStart][1]) {
                    xSplit[cellStart][1] += PRIMARY_SIZE_X;
                    int* tmp = new(std::nothrow) int [xSplit[cellStart][1]];
                    if (!tmp) {
						return ERROR_NO_MEMORY;
                    }
                    std::memcpy(
                        tmp, xSplit[cellStart].get(), xSplit[cellStart][0] * sizeof(int)
                    );
                    xSplit[cellStart].reset(tmp);
                }
                xSplit[cellStart][xSplit[cellStart][0]++] = boxIds[j];
            }
            ++cellStart;
        }
    }
#ifdef PRINT_STRUCTURE_STAT
    for (int i = 0; i < m_wszCellsX; ++i)
        m_bytesPerConstruction += xSplit[i][1] * sizeof(int);
#endif
    // split by z axis
    std::unique_ptr<
        std::unique_ptr<std::unique_ptr<int[]>[]>[]
    > zSplit(
        new(std::nothrow) std::unique_ptr<std::unique_ptr<int[]>[]>[m_wszCellsX]
    );
    if (!zSplit) {
		return ERROR_NO_MEMORY;
    }
    for (int i = 0; i < m_wszCellsX; ++i)
    {
		std::unique_ptr<CellBoundingPair[]> cellMinMax(new(std::nothrow) CellBoundingPair[m_wszCellsZ]); // TODO replace out
		if (!cellMinMax) {
			return ERROR_NO_MEMORY;
		}
        zSplit[i].reset(new(std::nothrow) std::unique_ptr<int[]>[m_wszCellsZ]);
        if (!zSplit[i]) {
			return ERROR_NO_MEMORY;
        }
        static const int PRIMARY_SIZE_Z = 64;
        int* boxIds = xSplit[i].get();
        for (int j = 0; j < m_wszCellsZ; ++j)
        {
            zSplit[i][j].reset(new(std::nothrow) int [PRIMARY_SIZE_Z]);
            if (!zSplit[i][j]) {
				return ERROR_NO_MEMORY;
            }
            zSplit[i][j][0] = 2; // number of ids + shift
            zSplit[i][j][1] = PRIMARY_SIZE_Z; // slab size
            float cellMin[3] = {
				m_worldMin[0] + i * m_cellSize,
				m_worldMin[1],
				m_worldMin[2] + j * m_cellSize
            };
            float cellMax[3] = {
				m_worldMin[0] + (i + 1) * m_cellSize,
                m_worldMax[1],
				m_worldMin[2] + (j + 1) * m_cellSize
            };
            rcVcopy(cellMinMax[j].min, cellMin);
            rcVcopy(cellMinMax[j].max, cellMax);
        }
        std::sort(boxIds + 2, boxIds + boxIds[0], CoordComparerOnIds<2>(bboxes.get()));
        for (int j = 0, k = 2; k < boxIds[0]; /*++k*/)
        {
            int cellStart = j;
            while(true)
            {
                if (cellStart >= m_wszCellsZ) {
                    ++k;
                    break;
                }
                bool aabbColl = checkAabbsCollision(
                    cellMinMax[cellStart].min,
                    cellMinMax[cellStart].max,
                    bboxes[boxIds[k]].min,
                    bboxes[boxIds[k]].max
                );
                if (!aabbColl) {
                    if (j == cellStart) {
                        ++j;
                    } else {
                        ++k;
                    }
                    break;
                }
                bool triColl = checkTriangleBelongAabb(
                    cellMinMax[cellStart].min,
                    cellMinMax[cellStart].max,
                    &m_tris[bboxes[boxIds[k]].polyIndex * 3]
                );
                if (triColl) {
                    if (zSplit[i][cellStart][0] == zSplit[i][cellStart][1]) {
                        zSplit[i][cellStart][1] += PRIMARY_SIZE_Z;
                        int* tmp = new(std::nothrow) int [zSplit[i][cellStart][1]];
                        if (!tmp) {
							return ERROR_NO_MEMORY;
                        }
                        std::memcpy(
                            tmp,
                            zSplit[i][cellStart].get(),
                            zSplit[i][cellStart][0] * sizeof(int)
                        );
                        zSplit[i][cellStart].reset(tmp);
                    }
                    zSplit[i][cellStart][zSplit[i][cellStart][0]++] = boxIds[k];
                }
                ++cellStart;
            }
        }
    }
#ifdef PRINT_STRUCTURE_STAT
    m_bytesPerConstruction +=
        m_wszCellsX * sizeof(std::unique_ptr<std::unique_ptr<int[]>[]>);
    for (int i = 0; i < m_wszCellsX; ++i) {
        m_bytesPerConstruction += m_wszCellsZ * sizeof(std::unique_ptr<int[]>);
        for (int j = 0; j < m_wszCellsZ; ++j) {
            m_bytesPerConstruction += zSplit[i][j][1] * sizeof(int);
        }
    }
#endif

    std::unique_ptr<std::pair<int, int>[]> trisVertsPerCell(
        new(std::nothrow) std::pair<int, int>[m_cellsNum]
    );
	if (!trisVertsPerCell) {
		return ERROR_NO_MEMORY;
	}
	std::memset(trisVertsPerCell.get(), 0, sizeof(int) * m_cellsNum);
    for (int i = 0; i < m_wszCellsX; ++i) {
        for (int j = 0; j < m_wszCellsZ; ++j) {
            int trisNum = zSplit[i][j][0] - 2;
            std::pair<Grid2dBvh::BvhNode*, int> ret;
            if (trisNum)
                ret = makeBvh(bboxes.get(), zSplit[i][j].get() + 2, trisNum);
			int pos = i * m_wszCellsZ + j;
            if (!trisNum || ret.first) {
				GridCell* cell = &m_grid[pos];
                float cellMin[3] = {
					m_worldMin[0] + i * m_cellSize,
					m_worldMin[1],
					m_worldMin[2] + j * m_cellSize
                };
                float cellMax[3] = {
					m_worldMin[0] + (i + 1) * m_cellSize,
                    m_worldMax[1],
					m_worldMin[2] + (j + 1) * m_cellSize
                };
                rcVcopy(cell->bmin, cellMin);
                rcVcopy(cell->bmax, cellMax);
                cell->childsNumber = ret.second;
                cell->childs = ret.first;
#ifdef PRINT_STRUCTURE_STAT
                if (trisNum > m_maxTrisInGridCell) m_maxTrisInGridCell = trisNum;
#endif
            } else {
				return ERROR_NO_MEMORY;
            }
			trisVertsPerCell[pos].first = trisNum * 3; // verts
			trisVertsPerCell[pos].second = trisNum; // tris
        }
    }
    for (int i = 0; i < 3 * m_trisNum; ++i) {
        m_tris[i] *= CUR_VERTS_BLOCK;
    }
	if (SUCCESSFUL != constructOverlappingRectData(std::move(trisVertsPerCell))) {
		return ERROR_NO_MEMORY;
	}
#ifdef PRINT_STRUCTURE_STAT
    m_bytesPerConstruction +=
        sizeof(BminBmaxSegmentTree::BminBmax) *
            BminBmaxSegmentTree::getResourceManager().getSize();
#endif
    BminBmaxSegmentTree::freeResources();

	return SUCCESSFUL;
}

void Grid2dBvh::subdivideMedian(
    const Aabb3D* bboxes, int* boxIds, BvhNode* bnodes, int i, int j, int& curNodeNum
#ifdef PRINT_STRUCTURE_STAT
    ,int depth, int& maxBoxesInGridCell
#endif
) const {
#ifdef PRINT_STRUCTURE_STAT
    if (m_maxDepth < depth) m_maxDepth = depth;
    if (m_curDepth < depth) m_curDepth = depth;
#endif
    int n = j - i;
    int prevCurNodeNum = curNodeNum;
    BvhNode* curNode = &bnodes[curNodeNum++];
    if (n == 1) {
        rcVcopy(curNode->min, bboxes[boxIds[i]].min);
        rcVcopy(curNode->max, bboxes[boxIds[i]].max);
        curNode->triId = bboxes[boxIds[i]].polyIndex * 3;
#ifdef PRINT_STRUCTURE_STAT
        ++m_totalNodes;
        ++m_leafNodes;
#endif
    } else {
        rcVcopy(curNode->min, bboxes[boxIds[i]].min);
        rcVcopy(curNode->max, bboxes[boxIds[i]].max);
        for (int k = i + 1; k < j; ++k) {
            rcVmin(curNode->min, bboxes[boxIds[k]].min);
            rcVmax(curNode->max, bboxes[boxIds[k]].max);
        }
        float span[3] = {
            curNode->max[0] - curNode->min[0],
            curNode->max[1] - curNode->min[1],
            curNode->max[2] - curNode->min[2]
        };
        int maxAxis = 0;
        if (span[1] > span[maxAxis])
            maxAxis = 1;
        if (span[2] > span[maxAxis])
            maxAxis = 2;
        if (maxAxis == 0)
            std::sort(boxIds + i, boxIds + j, CoordComparerOnIds<0>(bboxes));
        else if (maxAxis == 1)
            std::sort(boxIds + i, boxIds + j, CoordComparerOnIds<1>(bboxes));
        else if (maxAxis == 2)
            std::sort(boxIds + i, boxIds + j, CoordComparerOnIds<2>(bboxes));
        int k = i + n / 2;
        subdivideMedian(bboxes, boxIds, bnodes, i, k, curNodeNum
#ifdef PRINT_STRUCTURE_STAT
            ,depth + 1, maxBoxesInGridCell
#endif
        );
        subdivideMedian(bboxes, boxIds, bnodes, k, j, curNodeNum
#ifdef PRINT_STRUCTURE_STAT
            ,depth + 1, maxBoxesInGridCell
#endif
        );
        int delta = curNodeNum - prevCurNodeNum;
        curNode->triId = -delta;
#ifdef PRINT_STRUCTURE_STAT
        ++m_totalNodes;
        ++m_internalNodes;
        ++maxBoxesInGridCell;
#endif
    }
}

float Grid2dBvh::calcHalfSurfaceArea(const float* bboxDiff)
{
    return bboxDiff[0] * bboxDiff[1] + bboxDiff[1] * bboxDiff[2] + bboxDiff[2] * bboxDiff[0];
}

float Grid2dBvh::calcPartSahValue(const float* diffTotal, const float* bboxDiff, const int n)
{
    return COST_CHECK_TRI * n *
        (calcHalfSurfaceArea(bboxDiff) / calcHalfSurfaceArea(diffTotal));
}

float Grid2dBvh::calcSah(
    const BminBmaxSegmentTree& tree,
    int i,
    int mid,
    int j,
    float* totalMin,
    float* totalMax
) {
    float bmin[3], bmax[3], diffLeft[3], diffRight[3], diffTotal[3];

    tree.calcBbox(i, mid - 1, bmin, bmax);
    rcVcopy(totalMin, bmin);
    rcVcopy(totalMax, bmax);
    rcVsub(diffLeft, bmax, bmin);

    tree.calcBbox(mid, j - 1, bmin, bmax);
    rcVmin(totalMin, bmin);
    rcVmax(totalMax, bmax);
    rcVsub(diffRight, bmax, bmin);

    rcVsub(diffTotal, totalMax, totalMin);
    return COST_CHECK_BBOX +
        calcPartSahValue(diffTotal, diffLeft, mid - i) +
        calcPartSahValue(diffTotal, diffRight, j - mid);
}

void Grid2dBvh::subdivideSah(
    const Aabb3D* bboxes, int* boxIds, BvhNode* bnodes, int i, int j, int& curNodeNum
#ifdef PRINT_STRUCTURE_STAT
    ,int depth, int& maxBoxesInGridCell
#endif
) const {
#ifdef PRINT_STRUCTURE_STAT
    if (m_maxDepth < depth) m_maxDepth = depth;
    if (m_curDepth < depth) m_curDepth = depth;
#endif
    int n = j - i;
    int prevCurNodeNum = curNodeNum;
    BvhNode* curNode = &bnodes[curNodeNum++];

    if (n <= LIMIT_POLYS_STOP) {
        assert(n > 0);
        if (n != 1) {
            curNode->triId = -n;
            curNode->min[0] = curNode->min[1] = curNode->min[2] =
                std::numeric_limits<float>::max();
            curNode->max[0] = curNode->max[1] = curNode->max[2] =
                -std::numeric_limits<float>::max();
            for (int k = i; k < j; ++k) {
                BvhNode* node = &bnodes[curNodeNum++];
                const Aabb3D* box = bboxes + boxIds[k];
                rcVcopy(node->min, box->min);
                rcVcopy(node->max, box->max);
                node->triId = box->polyIndex * 3;
                rcVmin(curNode->min, box->min);
                rcVmax(curNode->max, box->max);
#ifdef PRINT_STRUCTURE_STAT
                ++m_totalNodes;
                ++m_leafNodes;
                ++maxBoxesInGridCell;
#endif
            }
        } else {
            const Aabb3D* box = bboxes + boxIds[i];
            rcVcopy(curNode->min, box->min);
            rcVcopy(curNode->max, box->max);
            curNode->triId = box->polyIndex * 3;
#ifdef PRINT_STRUCTURE_STAT
            ++m_totalNodes;
            ++m_leafNodes;
            ++maxBoxesInGridCell;
#endif
        }
    } else {
        float minSah = std::numeric_limits<float>::max();
        int bestSep = -1;
        int bestAxis = - 1;
        for (int axis = 0; axis < 3; ++axis) {
            std::sort(boxIds + i, boxIds + j, CoordComparer(axis, bboxes));
            BminBmaxSegmentTree tree;
            tree.calcTree(i, j, bboxes, boxIds);
            for (int k = i; k < j - 1; ++k) {
                float sah =
                    calcSah(tree, i, k + 1, j, curNode->min, curNode->max);
                if (sah < minSah) {
                    minSah = sah;
                    bestSep = k + 1;
                    bestAxis = axis;
                }
            }
        }
        float diff[3];
        rcVsub(diff, curNode->max, curNode->min);
        float totalSah = COST_CHECK_BBOX + calcPartSahValue(diff, diff, j - i);
        if (totalSah > minSah) {
            if (bestAxis != 2)
                std::sort(boxIds + i, boxIds + j, CoordComparer(bestAxis, bboxes));
            subdivideSah(bboxes, boxIds, bnodes, i, bestSep, curNodeNum
#ifdef PRINT_STRUCTURE_STAT
                ,depth + 1, maxBoxesInGridCell
#endif
            );
            subdivideSah(bboxes, boxIds, bnodes, bestSep, j, curNodeNum
#ifdef PRINT_STRUCTURE_STAT
                ,depth + 1, maxBoxesInGridCell
#endif
            );
            int delta = curNodeNum - prevCurNodeNum;
            curNode->triId = -delta;
			assert(curNode->triId != 0);
#ifdef PRINT_STRUCTURE_STAT
            ++m_totalNodes;
            ++m_internalNodes;
            ++maxBoxesInGridCell;
#endif
        } else {
            curNode->triId = i - j; // negative value
			assert(curNode->triId != 0);
            for (int k = i; k < j; ++k) {
                BvhNode* node = &bnodes[curNodeNum++];
                const Aabb3D* box = bboxes + boxIds[k];
                rcVcopy(node->min, box->min);
                rcVcopy(node->max, box->max);
                node->triId = box->polyIndex * 3;
#ifdef PRINT_STRUCTURE_STAT
                ++m_totalNodes;
                ++m_leafNodes;
                ++maxBoxesInGridCell;
#endif
            }
        }
    }
}

std::pair<Grid2dBvh::BvhNode*, int> Grid2dBvh::makeBvh(
    const Aabb3D* bboxes, int* boxIds, const int trisNum
) const {
    rcIgnoreUnused(&Grid2dBvh::subdivideMedian);
	BvhNode* bnodes = new(std::nothrow) BvhNode[trisNum * 2];
    if (!bnodes) {
        return {};
    }
    int curNodeNum = 0;
#ifdef PRINT_STRUCTURE_STAT
    int depth = 1;
    int maxBoxesInGridCell = 0;
#endif
    subdivideSah(bboxes, boxIds, bnodes, 0, trisNum, curNodeNum
#ifdef PRINT_STRUCTURE_STAT
        ,depth, maxBoxesInGridCell
#endif
    );
#ifdef PRINT_STRUCTURE_STAT
    m_curDepth = 0;
    if (maxBoxesInGridCell > m_maxBoxesInGridCell)
        m_maxBoxesInGridCell = maxBoxesInGridCell;
#endif
    assert(curNodeNum <= trisNum * 2);
    BvhNode* bnodesNew = allocAlignedArr<BvhNode>(curNodeNum, 16);//new(std::nothrow) BvhNode[curNodeNum];
    if (!bnodesNew) {
        delete [] bnodes;
        return {};
    }
    std::memcpy(bnodesNew, bnodes, curNodeNum * sizeof(BvhNode));
    delete [] bnodes;
#ifdef PRINT_STRUCTURE_STAT
    m_bytesForData += curNodeNum * sizeof(BvhNode);
#endif
    return {bnodesNew, curNodeNum};
}

struct IsectTriArgs
{
#ifdef USAGE_SSE_1_0
    __m128 start;
    __m128 diff;
#else
    const float* start;
    float diff[3];
#endif
};

static void calcIsectTriArgs(IsectTriArgs& args, const float* start, const float* end)
{
#ifdef USAGE_SSE_1_0
    args.start = _mm_setr_ps(start[0], start[1], start[2], 0.f);
    args.diff = _mm_sub_ps(args.start, _mm_setr_ps(end[0], end[1], end[2], 0.f));
#else
    args.start = start;
    rcVsub(args.diff, start, end);
#endif
}

#ifdef USAGE_SSE_1_0
inline __m128 sseVcross(__m128 a, __m128 b) {
    __m128 tmp0 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3,0,2,1));
    __m128 tmp1 = _mm_shuffle_ps(b, b ,_MM_SHUFFLE(3,1,0,2));
    __m128 tmp2 = _mm_mul_ps(tmp0, b);
    __m128 tmp3 = _mm_mul_ps(tmp0, tmp1);
    __m128 tmp4 = _mm_shuffle_ps(tmp2, tmp2,_MM_SHUFFLE(3,0,2,1));
    return _mm_sub_ps(tmp3, tmp4);
}

inline float sseVdot(__m128 a, __m128 b) {
    __m128 tmp = _mm_mul_ps(a, b);
    //return *(float*)&tmp + *((float*)&tmp + 1) + *((float*)&tmp + 2);
    tmp = _mm_hadd_ps(tmp, tmp); // sse3
    tmp = _mm_hadd_ps(tmp, tmp); // sse3
    return _mm_cvtss_f32(tmp);
}

static bool intersectSegmentTriangleRed(
    const IsectTriArgs& args,
    const float* a,
    const float* b,
    const float* c,
    float& t
) {
	static const float EPS = 1e-6;
	float v, w;
    __m128 ma = _mm_load_ps(a);
    __m128 mb = _mm_load_ps(b);
    __m128 mc = _mm_load_ps(c);
    __m128 ab = _mm_sub_ps(mb, ma);
    __m128 ac = _mm_sub_ps(mc, ma);

    // Compute triangle normal. Can be precalculated or cached if
    // intersecting multiple segments against the same triangle
    __m128 norm = sseVcross(ab, ac);

    // Compute denominator d. If d <= 0, segment is parallel to or points
    // away from triangle, so exit early
	float d = sseVdot(args.diff, norm);
	if (std::abs(d) < EPS) return false;
	float invd = 1.f / d;

    // Compute intersection t value of pq with plane of triangle. A ray
    // intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
    // dividing by d until intersection has been found to pierce triangle
    __m128 ap = _mm_sub_ps(args.start, ma);
	t = sseVdot(ap, norm) * invd;

    // Compute barycentric coordinate components and test if within bounds
    __m128 e = sseVcross(args.diff, ap);

	v = sseVdot(ac, e) * invd;
	w = -sseVdot(ab, e) * invd;
    if (
		t < 0.0f || t > 1.f || v < 0.0f || v > 1.f || w < 0.0f || v + w > 1.f
    ) return false;

    return true;
}
#else
static bool intersectSegmentTriangleRed(
    const IsectTriArgs& args,
    const float* a,
    const float* b,
    const float* c,
    float & t
) {
    float v, w;
    float ab[3], ac[3], ap[3], norm[3], e[3];
    rcVsub(ab, b, a);
    rcVsub(ac, c, a);

    // Compute triangle normal. Can be precalculated or cached if
    // intersecting multiple segments against the same triangle
    rcVcross(norm, ab, ac);

    // Compute denominator d. If d <= 0, segment is parallel to or points
    // away from triangle, so exit early
    float d = rcVdot(args.diff, norm);
	if (std::abs(d) < EPS) return false;
	float invd = 1.f / d;

    // Compute intersection t value of pq with plane of triangle. A ray
    // intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
    // dividing by d until intersection has been found to pierce triangle
    rcVsub(ap, args.start, a);
	t = rcVdot(ap, norm) * invd;
    if (t < 0.0f) return false;
	if (t > 1.f) return false; // For segment; exclude this code line for a ray test

    // Compute barycentric coordinate components and test if within bounds
    rcVcross(e, args.diff, ap);
	v = rcVdot(ac, e) * v;
	if (v < 0.0f || v > 1.f) return false;
	w = -rcVdot(ab, e) * invd;
	if (w < 0.0f || v + w > 1.f) return false;

    return true;
}
#endif

struct IsectAabbArgs
{
    const float* sp;
    float d[3];
    float ood[3];
#ifdef USAGE_SSE_1_0
    __m128 start;
    __m128 mul;
#endif
};

static void calcIsectAabbArgs(IsectAabbArgs& args, const float* start, const float* end)
{
    args.sp = start;
    args.d[0] = end[0] - start[0];
    args.d[1] = end[1] - start[1];
    args.d[2] = end[2] - start[2];
    args.ood[0] = 1.f / args.d[0];
    args.ood[1] = 1.f / args.d[1];
    args.ood[2] = 1.f / args.d[2];
#ifdef USAGE_SSE_1_0
    args.start = _mm_setr_ps(args.sp[0], args.sp[1], args.sp[2], 0.f);
    args.mul = _mm_setr_ps(args.ood[0], args.ood[1], args.ood[2], 0.f);
#endif
}

// TODO check fp robustness
static bool isectSegXzAabbRed(
    const IsectAabbArgs& args, const float* amin, const float* amax
) {
#ifdef USAGE_SSE_1_0
    __m128 min = _mm_setr_ps(amin[0], 0.f, amin[2], 0.f);
    __m128 max = _mm_setr_ps(amax[0], 0.f, amax[2], 0.f);
    min = _mm_mul_ps(_mm_sub_ps(min, args.start), args.mul);
    max = _mm_mul_ps(_mm_sub_ps(max, args.start), args.mul);
    __m128 min1 = _mm_min_ps(min, max);
    __m128 max1 = _mm_max_ps(min, max);
    min1 = _mm_max_ps(min1, _mm_set1_ps(0.f));
    max1 = _mm_min_ps(max1, _mm_set1_ps(1.f));

    __m128 min2 = _mm_shuffle_ps(min1, min1, _MM_SHUFFLE(0, 1, 3, 2));
    __m128 max2 = _mm_shuffle_ps(max1, max1, _MM_SHUFFLE(0, 1, 3, 2));
    min = _mm_max_ss(min1, min2);
    max = _mm_min_ss(max1, max2);

    return _mm_comige_ss(max, min);
#else
    float ttmin = 0.f;
    float ttmax = 1.f;

    float tx1 = (amin[0] - args.sp[0]) * args.ood[0];
    float tx2 = (amax[0] - args.sp[0]) * args.ood[0];
    ttmin = std::max(ttmin, std::min(tx1, tx2));
    ttmax = std::min(ttmax, std::max(tx1, tx2));

    float tz1 = (amin[2] - args.sp[2]) * args.ood[2];
    float tz2 = (amax[2] - args.sp[2]) * args.ood[2];
    ttmin = std::max(ttmin, std::min(tz1, tz2));
    ttmax = std::min(ttmax, std::max(tz1, tz2));

    return ttmax >= ttmin;
#endif
}

// TODO check fp robustness
static bool isectSegAabbRed(IsectAabbArgs& args, const float* amin, const float* amax)
{
#ifdef USAGE_SSE_1_0
    __m128 min = _mm_load_ps(amin);
    __m128 max = _mm_load_ps(amax);
    min = _mm_mul_ps(_mm_sub_ps(min, args.start), args.mul);
    max = _mm_mul_ps(_mm_sub_ps(max, args.start), args.mul);
    __m128 min1 = _mm_min_ps(min, max);
    __m128 max1 = _mm_max_ps(min, max);
    min1 = _mm_max_ps(min1, _mm_set1_ps(0.f));
    max1 = _mm_min_ps(max1, _mm_set1_ps(1.f));

    __m128 min2 = _mm_shuffle_ps(min1, min1, _MM_SHUFFLE(0, 3, 2, 1));
    __m128 max2 = _mm_shuffle_ps(max1, max1, _MM_SHUFFLE(0, 3, 2, 1));
    min = _mm_max_ss(min1, min2);
    max = _mm_min_ss(max1, max2);
    min2 = _mm_movehl_ps(min, min);
    max2 = _mm_movehl_ps(max, max);
    min = _mm_max_ss(min, min2);
    max = _mm_min_ss(max, max2);

    return _mm_comige_ss(max, min);
#else
    float tmin = 0.f;
    float tmax = 1.f;

    float tx1 = (amin[0] - args.sp[0]) * args.ood[0];
    float tx2 = (amax[0] - args.sp[0]) * args.ood[0];
    tmin = std::max(tmin, std::min(tx1, tx2));
    tmax = std::min(tmax, std::max(tx1, tx2));

    float ty1 = (amin[1] - args.sp[1]) * args.ood[1];
    float ty2 = (amax[1] - args.sp[1]) * args.ood[1];
    tmin = std::max(tmin, std::min(ty1, ty2));
    tmax = std::min(tmax, std::max(ty1, ty2));

    float tz1 = (amin[2] - args.sp[2]) * args.ood[2];
    float tz2 = (amax[2] - args.sp[2]) * args.ood[2];
    tmin = std::max(tmin, std::min(tz1, tz2));
    tmax = std::min(tmax, std::max(tz1, tz2));

    return tmax >= tmin;
#endif
}

bool Grid2dBvh::segTriCollisionFirstHit(const float* start, const float* end, float& t) const
{
#ifdef USAGE_SSE_1_0
    __m128 composite1 = _mm_setr_ps(start[0], start[2], end[0], end[2]);
    composite1 = _mm_mul_ps(_mm_sub_ps(composite1, m_worldMinVecXzXz), m_cellSizeInvVec);
    __m128 composite2 = _mm_shuffle_ps(composite1, composite1, _MM_SHUFFLE(0, 1, 3, 2));
    __m128 min = _mm_min_ps(composite1, composite2);
    __m128 max = _mm_max_ps(composite1, composite2);
    int xiMin = static_cast<int>(*(const float*)&min);
    int xiMax = static_cast<int>(*(const float*)&max);
    int ziMin = static_cast<int>(*((const float*)&min + 1));
    int ziMax = static_cast<int>(*((const float*)&max + 1));
	if (xiMin < 0) xiMin = 0;
	if (ziMin < 0) ziMin = 0;
	if (xiMax >= m_wszCellsX) xiMax = m_wszCellsX - 1;
	if (ziMax >= m_wszCellsZ) ziMax = m_wszCellsZ - 1;
#else
	XzGridBorders ret = calcXzGridBorders(start, end);
	int xiMin = ret.xiMin;
	int xiMax = ret.xiMax;
	int ziMin = ret.ziMin;
	int ziMax = ret.ziMin;
#endif
    IsectAabbArgs aabbArgs;
    calcIsectAabbArgs(aabbArgs, start, end);
    IsectTriArgs triArgs;
    calcIsectTriArgs(triArgs, start, end);
	int vobIds[VOBS_NUM_COLLIDE_CHEKING] = {0};
	int vobIdsIdx = 0;

    for (int i = xiMin; i <= xiMax; ++i) {
        const GridCell* xShift = m_grid + m_wszCellsZ * i;
        for (int j = ziMin; j <= ziMax; ++j) {
            const GridCell* cell = xShift + j;
#ifdef PRINT_TRI_VS_SEG_LATENCY
            auto tp1 = std::chrono::steady_clock::now();
#endif
            bool ret = isectSegXzAabbRed(aabbArgs, cell->bmin, cell->bmax);
#ifdef PRINT_TRI_VS_SEG_LATENCY
            auto tp2 = std::chrono::steady_clock::now();
            auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
            std::printf("Time diff isectSegAABBXZ ns: %d\n", (int)diff);
#endif
            if (!ret) continue;
            const BvhNode* curNode = cell->childs;
            const BvhNode* endNode = curNode + cell->childsNumber;
            while (curNode < endNode) {
				const bool leaf = curNode->triId >= 0;
#ifdef PRINT_TRI_VS_SEG_LATENCY
                auto tp1 = std::chrono::steady_clock::now();
#endif
                const bool boxIntersect =
                    isectSegAabbRed(aabbArgs, curNode->min, curNode->max);
#ifdef PRINT_TRI_VS_SEG_LATENCY
                auto tp2 = std::chrono::steady_clock::now();
                auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
                std::printf("Time diff isectSegAABB ns: %d\n", (int)diff);
                ++totalNodesTraversed;
                if (leaf) ++totalLeafesTraversed;
#endif
                if (leaf && boxIntersect) {
#ifdef PRINT_TRI_VS_SEG_LATENCY
                    ++totalPolysTraversed;
#endif
                    const int* vIds = m_tris + curNode->triId;
#ifdef PRINT_TRI_VS_SEG_LATENCY
                    auto tp1 = std::chrono::steady_clock::now();
#endif
                    ret = intersectSegmentTriangleRed(
                        triArgs,
                        m_verts + vIds[0],
                        m_verts + vIds[1],
						m_verts + vIds[2],
                        t
                    );
					if (ret) { // TODO last collide id checking
						PolyAreaFlags::FlagType flag = m_triFlags[curNode->triId / 3];
						if (flag.isVobPos) {
							if (flag.isActiveVobPos) {
								int vobId = flag.vobIdOrCollFlags;
								// TODO realize at sse instructions
								bool checked = false;
								for (int k = 0; k < VOBS_NUM_COLLIDE_CHEKING && vobIds[k]; ++k) {
									checked = vobIds[k] == vobId;
									if (checked) break;
								}
								if (!checked) {
									vobIds[vobIdsIdx % VOBS_NUM_COLLIDE_CHEKING] = vobId;
									++vobIdsIdx;
									if (segTriCollisionVobFirstHit(vobId, start, end, t)) {
										return true;
									}
								}
							}
						} else {
							return true;
						}
					}
#ifdef PRINT_TRI_VS_SEG_LATENCY
                    auto tp2 = std::chrono::steady_clock::now();
                    auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
                    std::printf("Time diff intersectSegmentTriangle ns: %d\n", (int)diff);
#endif
                }
                if (leaf || boxIntersect)
                    ++curNode;
                else
                    // triId stores negative number of inside triangles
                    curNode -= curNode->triId;
            }
        }
    }
    return false;
}

bool Grid2dBvh::segTriCollisionVobFirstHit(
	int vobId, const float* start, const float* end, float& t
) const {
	const VobEntry& vob = m_vobs[vobId];
	const MeshEntry& vobMesh = m_vobsMeshes[vob.meshIndex];
	const auto& vobPos = vob.positions[vob.activePosIndex];
	float startReal[3], endReal[3];
	transformVertex(start, vobPos.invTrafo, startReal);
	transformVertex(end, vobPos.invTrafo, endReal);
	IsectAabbArgs aabbArgs;
	calcIsectAabbArgs(aabbArgs, startReal, endReal);
	IsectTriArgs triArgs;
	calcIsectTriArgs(triArgs, startReal, endReal);
	const float* verts = vobMesh.mesh.verts;
	const int* tris = vobMesh.mesh.tris.get();

	const BvhNode* curNode = vobMesh.childs;
	const BvhNode* endNode = curNode + vobMesh.childsNumber;
	while (curNode < endNode) {
		const bool leaf = curNode->triId >= 0;
#ifdef PRINT_TRI_VS_SEG_LATENCY
		auto tp1 = std::chrono::steady_clock::now();
#endif
		const bool boxIntersect =
			isectSegAabbRed(aabbArgs, curNode->min, curNode->max);
#ifdef PRINT_TRI_VS_SEG_LATENCY
		auto tp2 = std::chrono::steady_clock::now();
		auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
        std::printf("Time diff isectSegAABB ns: %d\n", (int)diff);
		++totalNodesTraversed;
		if (leaf) ++totalLeafesTraversed;
#endif
		if (leaf && boxIntersect) {
#ifdef PRINT_TRI_VS_SEG_LATENCY
			++totalPolysTraversed;
#endif
			const int* vIds = tris + curNode->triId;
#ifdef PRINT_TRI_VS_SEG_LATENCY
			auto tp1 = std::chrono::steady_clock::now();
#endif
			bool ret = intersectSegmentTriangleRed(
				triArgs,
				verts + vIds[0],
				verts + vIds[1],
				verts + vIds[2],
				t
			);
			if (ret) {
				return true;
			}
#ifdef PRINT_TRI_VS_SEG_LATENCY
			auto tp2 = std::chrono::steady_clock::now();
			auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
            std::printf("Time diff intersectSegmentTriangle ns: %d\n", (int)diff);
#endif
		}
		if (leaf || boxIntersect)
			++curNode;
		else
			// triId stores negative number of inside triangles
			curNode -= curNode->triId;
	}

	return false;
}

bool Grid2dBvh::segTriCollisionNearestHit(const float* start, const float* end, float& t) const
{
	static constexpr float TINIT_VAL = 2.f;
#ifdef USAGE_SSE_1_0
	__m128 composite1 = _mm_setr_ps(start[0], start[2], end[0], end[2]);
	composite1 = _mm_mul_ps(_mm_sub_ps(composite1, m_worldMinVecXzXz), m_cellSizeInvVec);
	__m128 composite2 = _mm_shuffle_ps(composite1, composite1, _MM_SHUFFLE(0, 1, 3, 2));
	__m128 min = _mm_min_ps(composite1, composite2);
	__m128 max = _mm_max_ps(composite1, composite2);
	int xiMin = static_cast<int>(*(const float*)&min);
	int xiMax = static_cast<int>(*(const float*)&max);
	int ziMin = static_cast<int>(*((const float*)&min + 1));
	int ziMax = static_cast<int>(*((const float*)&max + 1));
	if (xiMin < 0) xiMin = 0;
	if (ziMin < 0) ziMin = 0;
	if (xiMax >= m_wszCellsX) xiMax = m_wszCellsX - 1;
	if (ziMax >= m_wszCellsZ) ziMax = m_wszCellsZ - 1;
#else
	XzGridBorders ret = calcXzGridBorders(start, end);
	int xiMin = ret.xiMin;
	int xiMax = ret.xiMax;
	int ziMin = ret.ziMin;
	int ziMax = ret.ziMin;
#endif
	float tCur;
	IsectAabbArgs aabbArgs;
	calcIsectAabbArgs(aabbArgs, start, end);
	IsectTriArgs triArgs;
	calcIsectTriArgs(triArgs, start, end);
	t = TINIT_VAL;
	int vobIds[VOBS_NUM_COLLIDE_CHEKING] = {0};
	int vobIdsIdx = 0;

	for (int i = xiMin; i <= xiMax; ++i) {
		const GridCell* xShift = m_grid + m_wszCellsZ * i;
		for (int j = ziMin; j <= ziMax; ++j) {
			const GridCell* cell = xShift + j;
#ifdef PRINT_TRI_VS_SEG_LATENCY
			auto tp1 = std::chrono::steady_clock::now();
#endif
			bool ret = isectSegXzAabbRed(aabbArgs, cell->bmin, cell->bmax);
#ifdef PRINT_TRI_VS_SEG_LATENCY
			auto tp2 = std::chrono::steady_clock::now();
			auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
            std::printf("Time diff isectSegAABBXZ ns: %d\n", (int)diff);
#endif
			if (!ret) continue;
			const BvhNode* curNode = cell->childs;
			const BvhNode* endNode = curNode + cell->childsNumber;
			while (curNode < endNode) {
				const bool leaf = curNode->triId >= 0;
#ifdef PRINT_TRI_VS_SEG_LATENCY
				auto tp1 = std::chrono::steady_clock::now();
#endif
				const bool boxIntersect =
					isectSegAabbRed(aabbArgs, curNode->min, curNode->max);
#ifdef PRINT_TRI_VS_SEG_LATENCY
				auto tp2 = std::chrono::steady_clock::now();
				auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
                std::printf("Time diff isectSegAABB ns: %d\n", (int)diff);
				++totalNodesTraversed;
				if (leaf) ++totalLeafesTraversed;
#endif
				if (leaf && boxIntersect) {
#ifdef PRINT_TRI_VS_SEG_LATENCY
					++totalPolysTraversed;
#endif
					const int* vIds = m_tris + curNode->triId;
#ifdef PRINT_TRI_VS_SEG_LATENCY
					auto tp1 = std::chrono::steady_clock::now();
#endif
					ret = intersectSegmentTriangleRed(
						triArgs,
						m_verts + vIds[0],
						m_verts + vIds[1],
						m_verts + vIds[2],
						tCur
					);
					if (ret) { // TODO last collide id checking
						PolyAreaFlags::FlagType flag = m_triFlags[curNode->triId / 3];
						if (flag.isVobPos) {
							if (flag.isActiveVobPos) {
								int vobId = flag.vobIdOrCollFlags;
								// TODO realize at sse instructions
								bool checked = false;
								for (int k = 0; k < VOBS_NUM_COLLIDE_CHEKING && vobIds[k]; ++k) {
									checked = vobIds[k] == vobId;
									if (checked) break;
								}
								if (!checked) {
									vobIds[vobIdsIdx % VOBS_NUM_COLLIDE_CHEKING] = vobId;
									++vobIdsIdx;
									if (segTriCollisionVobNearestHit(vobId, start, end, tCur)) {
										if (tCur < t) t = tCur;
									}
								}
							}
						} else {
							if (tCur < t) t = tCur;
						}
					}
#ifdef PRINT_TRI_VS_SEG_LATENCY
					auto tp2 = std::chrono::steady_clock::now();
					auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
                    std::printf("Time diff intersectSegmentTriangle ns: %d\n", (int)diff);
#endif
				}
				if (leaf || boxIntersect)
					++curNode;
				else
					// triId stores negative number of inside triangles
					curNode -= curNode->triId;
			}
		}
	}

	return t != TINIT_VAL;
}

bool Grid2dBvh::segTriCollisionVobNearestHit(
	int vobId, const float* start, const float* end, float& t
) const {
	static constexpr float TINIT_VAL = 2.f;
	const VobEntry& vob = m_vobs[vobId];
	const MeshEntry& vobMesh = m_vobsMeshes[vob.meshIndex];
	const auto& vobPos = vob.positions[vob.activePosIndex];
	float startReal[3], endReal[3];
	transformVertex(start, vobPos.invTrafo, startReal);
	transformVertex(end, vobPos.invTrafo, endReal);
	IsectAabbArgs aabbArgs;
	calcIsectAabbArgs(aabbArgs, startReal, endReal);
	IsectTriArgs triArgs;
	calcIsectTriArgs(triArgs, startReal, endReal);
	const float* verts = vobMesh.mesh.verts;
	const int* tris = vobMesh.mesh.tris.get();

	t = TINIT_VAL;
	float tCur;
	const BvhNode* curNode = vobMesh.childs;
	const BvhNode* endNode = curNode + vobMesh.childsNumber;
	while (curNode < endNode) {
		const bool leaf = curNode->triId >= 0;
#ifdef PRINT_TRI_VS_SEG_LATENCY
		auto tp1 = std::chrono::steady_clock::now();
#endif
		const bool boxIntersect =
			isectSegAabbRed(aabbArgs, curNode->min, curNode->max);
#ifdef PRINT_TRI_VS_SEG_LATENCY
		auto tp2 = std::chrono::steady_clock::now();
		auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
        std::printf("Time diff isectSegAABB ns: %d\n", (int)diff);
		++totalNodesTraversed;
		if (leaf) ++totalLeafesTraversed;
#endif
		if (leaf && boxIntersect) {
#ifdef PRINT_TRI_VS_SEG_LATENCY
			++totalPolysTraversed;
#endif
			const int* vIds = tris + curNode->triId;
#ifdef PRINT_TRI_VS_SEG_LATENCY
			auto tp1 = std::chrono::steady_clock::now();
#endif
			bool ret = intersectSegmentTriangleRed(
				triArgs,
				verts + vIds[0],
				verts + vIds[1],
				verts + vIds[2],
				tCur
			);
			if (ret) {
				if (tCur < t) t = tCur;
			}
#ifdef PRINT_TRI_VS_SEG_LATENCY
			auto tp2 = std::chrono::steady_clock::now();
			auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
            std::printf("Time diff intersectSegmentTriangle ns: %d\n", (int)diff);
#endif
		}
		if (leaf || boxIntersect)
			++curNode;
		else
			// triId stores negative number of inside triangles
			curNode -= curNode->triId;
	}

	return t != TINIT_VAL;
}

static bool intersectObbTriangle(const OBBExt* be, const float* triPoints)
{
    float minMaxObb[2], minMaxTri[2];
    float cross[3];
    float e[3 * 3];
    rcVsub(e + 0, triPoints + 3, triPoints);
    rcVsub(e + 3, triPoints + 6, triPoints + 3);
    rcVsub(e + 6, triPoints, triPoints + 6);

    // edges
    for (int i = 0; i < 3; ++i) {
        rcVcross(cross, e + i * 3, be->b.dir);
        calcProjection(be->verts, 8, cross, minMaxObb);
        calcProjection(triPoints, 3, cross, minMaxTri);
        if (minMaxObb[1] < minMaxTri[0] || minMaxObb[0] > minMaxTri[1]) return false;
        rcVcross(cross, e + i * 3, be->b.dir + 3);
        calcProjection(be->verts, 8, cross, minMaxObb);
        calcProjection(triPoints, 3, cross, minMaxTri);
        if (minMaxObb[1] < minMaxTri[0] || minMaxObb[0] > minMaxTri[1]) return false;
        rcVcross(cross, e + i * 3, be->b.dir + 6);
        calcProjection(be->verts, 8, cross, minMaxObb);
        calcProjection(triPoints, 3, cross, minMaxTri);
        if (minMaxObb[1] < minMaxTri[0] || minMaxObb[0] > minMaxTri[1]) return false;
    }
    // obb faces
    calcProjection(be->verts, 8, be->b.dir, minMaxObb);
    calcProjection(triPoints, 3, be->b.dir, minMaxTri);
    if (minMaxObb[1] < minMaxTri[0] || minMaxObb[0] > minMaxTri[1]) return false;
    calcProjection(be->verts, 8, be->b.dir + 3, minMaxObb);
    calcProjection(triPoints, 3, be->b.dir + 3, minMaxTri);
    if (minMaxObb[1] < minMaxTri[0] || minMaxObb[0] > minMaxTri[1]) return false;
    calcProjection(be->verts, 8, be->b.dir + 6, minMaxObb);
    calcProjection(triPoints, 3, be->b.dir + 6, minMaxTri);
    if (minMaxObb[1] < minMaxTri[0] || minMaxObb[0] > minMaxTri[1]) return false;
    // tri
    rcVcross(cross, e + 0, e + 3);
    calcProjection(be->verts, 8, cross, minMaxObb);
    calcProjection(triPoints, 3, cross, minMaxTri);
    if (minMaxObb[1] < minMaxTri[0] || minMaxObb[0] > minMaxTri[1]) return false;

    return true;
}

bool Grid2dBvh::obbTriCollisionFirstHit(const OBBExt* be) const // TODO add robustness
{
    float triPoints[3 * 3];
    float min[3], max[3];
    rcVcopy(min, be->verts);
    rcVcopy(max, be->verts);
    for (int i = 1; i < 8; ++i) {
        rcVmin(min, be->verts + i * 3);
        rcVmax(max, be->verts + i * 3);
    }
	XzGridBorders ret = calcXzGridBorders(min, max);

	for (int i = ret.xiMin; i <= ret.xiMax; ++i) {
        const GridCell* xShift = m_grid + m_wszCellsZ * i;
		for (int j = ret.ziMin; j <= ret.ziMax; ++j) {
            const GridCell* cell = xShift + j;
//#ifdef PRINT_TRI_VS_OBB_LATENCY
//            auto tp1 = std::chrono::steady_clock::now();
//#endif
            bool ret = checkAabbsCollisionXZ(min, max, cell->bmin, cell->bmax);
//#ifdef PRINT_TRI_VS_OBB_LATENCY
//            auto tp2 = std::chrono::steady_clock::now();
//            auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
//            std::printf("Time diff checkAabbsCollisionXZ ns: %d\n", (int)diff);
//#endif
            if (!ret) continue;
            const BvhNode* curNode = cell->childs;
            const BvhNode* endNode = curNode + cell->childsNumber;
            while (curNode < endNode) {
				const bool leaf = curNode->triId >= 0;
//#ifdef PRINT_TRI_VS_OBB_LATENCY
//                auto tp1 = std::chrono::steady_clock::now();
//#endif
                const bool boxIntersect =
                    checkAabbsCollision(min, max, curNode->min, curNode->max);
//#ifdef PRINT_TRI_VS_OBB_LATENCY
//                auto tp2 = std::chrono::steady_clock::now();
//                auto diff =
//                    std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
//                std::printf("Time diff checkAabbsCollision ns: %d\n", (int)diff);
//#endif
//#ifdef PRINT_TRI_VS_OBB_LATENCY
//                ++totalNodesTraversed;
//                if (leaf) ++totalLeafesTraversed;
//#endif
                if (leaf && boxIntersect) {
//#ifdef PRINT_TRI_VS_OBB_LATENCY
//                    ++totalPolysTraversed;
//#endif
                    const int* vIds = m_tris + curNode->triId;
                    rcVcopy(triPoints, m_verts + vIds[0]);
                    rcVcopy(triPoints + 3, m_verts + vIds[1]);
                    rcVcopy(triPoints + 6, m_verts + vIds[2]);
//#ifdef PRINT_TRI_VS_OBB_LATENCY
//                auto tp3 = std::chrono::steady_clock::now();
//#endif
                    ret = intersectObbTriangle(be, triPoints);
//#ifdef PRINT_TRI_VS_OBB_LATENCY
//                auto tp4 = std::chrono::steady_clock::now();
//                auto diff =
//                    std::chrono::duration_cast<std::chrono::nanoseconds>(tp4 - tp3).count();
//                std::printf("Time diff intersectObbTriangle ns: %d\n", (int)diff);
//#endif
					if (ret) { // TODO last collide id checking
						PolyAreaFlags::FlagType flag = m_triFlags[curNode->triId / 3];
						if (flag.isVobPos) {
							if (flag.isActiveVobPos) {
								int vobId = flag.vobIdOrCollFlags;
								if (obbTriCollisionVobFirstHit(vobId, be)) {
									return true;
								}
							}
						} else {
							return true;
						}
					}
                }
                if (leaf || boxIntersect)
                    ++curNode;
                else
                    // triId stores negative number of inside triangles
                    curNode -= curNode->triId;
            }
        }
    }
    return false;
}

bool Grid2dBvh::obbTriCollisionVobFirstHit(int vobId, const OBBExt* be) const
{
	const VobEntry& vob = m_vobs[vobId];
	const MeshEntry& vobMesh = m_vobsMeshes[vob.meshIndex];
	const auto& vobPos = vob.positions[vob.activePosIndex];
	OBBExt* beMut = const_cast<OBBExt*>(be);
	float vertex[3];
	for (int i = 0; i < 3 * 8; i += 3) {
		transformVertex(&be->verts[i], vobPos.invTrafo, vertex);
		rcVcopy(&beMut->verts[i], vertex);
	}
	transformVertex(be->b.center, vobPos.invTrafo, vertex);
	rcVcopy(beMut->b.center, vertex);
	for (int i = 0; i < 3 * 3; i += 3) {
		transformDirection(&be->b.dir[i], vobPos.invTrafo, vertex);
		rcVcopy(&beMut->b.dir[i], vertex);
	}
	float triPoints[3 * 3];
	float min[3], max[3];
	rcVcopy(min, be->verts);
	rcVcopy(max, be->verts);
	for (int i = 1; i < 8; ++i) {
		rcVmin(min, be->verts + i * 3);
		rcVmax(max, be->verts + i * 3);
	}
	const float* verts = vobMesh.mesh.verts;
	const int* tris = vobMesh.mesh.tris.get();

	const BvhNode* curNode = vobMesh.childs;
	const BvhNode* endNode = curNode + vobMesh.childsNumber;
	while (curNode < endNode) {
		const bool leaf = curNode->triId >= 0;
//#ifdef PRINT_TRI_VS_OBB_LATENCY
//                auto tp1 = std::chrono::steady_clock::now();
//#endif
		const bool boxIntersect =
			checkAabbsCollision(min, max, curNode->min, curNode->max);
//#ifdef PRINT_TRI_VS_OBB_LATENCY
//                auto tp2 = std::chrono::steady_clock::now();
//                auto diff =
//                    std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
//                std::printf("Time diff checkAabbsCollision ns: %d\n", (int)diff);
//#endif
//#ifdef PRINT_TRI_VS_OBB_LATENCY
//                ++totalNodesTraversed;
//                if (leaf) ++totalLeafesTraversed;
//#endif
		if (leaf && boxIntersect) {
//#ifdef PRINT_TRI_VS_OBB_LATENCY
//                    ++totalPolysTraversed;
//#endif
			const int* vIds = tris + curNode->triId;
			rcVcopy(triPoints, verts + vIds[0]);
			rcVcopy(triPoints + 3, verts + vIds[1]);
			rcVcopy(triPoints + 6, verts + vIds[2]);
//#ifdef PRINT_TRI_VS_OBB_LATENCY
//                auto tp3 = std::chrono::steady_clock::now();
//#endif
			bool ret = intersectObbTriangle(be, triPoints);
//#ifdef PRINT_TRI_VS_OBB_LATENCY
//                auto tp4 = std::chrono::steady_clock::now();
//                auto diff =
//                    std::chrono::duration_cast<std::chrono::nanoseconds>(tp4 - tp3).count();
//                std::printf("Time diff intersectObbTriangle ns: %d\n", (int)diff);
//#endif
			if (ret) {
				return true;
			}
		}
		if (leaf || boxIntersect)
			++curNode;
		else
			// triId stores negative number of inside triangles
			curNode -= curNode->triId;
	}
	return false;
}

#ifdef PRINT_STRUCTURE_STAT
void Grid2dBvh::printStat(rcContext* ctx) const
{
    ctx->log(
        RC_LOG_WARNING,
        "Tris num: %d, verts num: %d, totalNodes: %d, m_leafNodes: %d, "
		"internalNodes: %d, maxDepth: %d, maxTrisInGridCell: %d, "
		"maxBoxesInGridCell: %d\n",
		m_trisNum, m_vertsNum, m_totalNodes, m_leafNodes,
		m_internalNodes, m_maxDepth, m_maxTrisInGridCell, m_maxBoxesInGridCell
	);
    ctx->log(
        RC_LOG_WARNING,
        "Construction mem size MB: %f, data mem size MB: %f\n",
		static_cast<float>(m_bytesPerConstruction) / (1024 * 1024),
		static_cast<float>(m_bytesForData) / (1024 * 1024)
	);
}
#endif

#if (PRINT_TRI_VS_SEG_LATENCY || PRINT_TRI_VS_OBB_LATENCY)
	int Grid2dBvh::getNodesPerCall() const {return totalNodesTraversed;}
	int Grid2dBvh::getLeafesPerCall() const {return totalLeafesTraversed;}
	int Grid2dBvh::getPolysPerCall() const {return totalPolysTraversed;}
	void Grid2dBvh::clearStatPerCall() const {
		totalNodesTraversed = 0;
		totalLeafesTraversed = 0;
		totalPolysTraversed = 0;
	}
#endif
