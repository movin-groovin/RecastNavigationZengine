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

void LoggerAdapter::doLogMessage(common::LogCategory category, const char* msg, int len)
{
	rcLogCategory rcCat = RC_LOG_ERROR;
	switch (category)
	{
		case common::LogCategory::LOG_PROGRESS:
			rcCat = RC_LOG_PROGRESS;
			break;
		case common::LogCategory::LOG_WARNING:
			rcCat = RC_LOG_WARNING;
			break;
		case common::LogCategory::LOG_ERROR:
			rcCat = RC_LOG_ERROR;
			break;
	}
	m_ctx->doLog(rcCat, msg, len);
}

InputGeom::InputGeom():
	m_showOffsetPlanes(),
	m_xMinOffsetCut(),
	m_xMaxOffsetCut(),
	m_zMinOffsetCut(),
	m_zMaxOffsetCut()
{
	release();
	zeroBboxes();
}

InputGeom::~InputGeom()
{
	release();
}

void InputGeom::release()
{
	m_meshExt.reset();
}

void InputGeom::zeroBboxes()
{
	std::memset(&m_meshBMin, 0, sizeof(m_meshBMin));
	std::memset(&m_meshBMax, 0, sizeof(m_meshBMax));
	std::memset(&m_meshBMinCur, 0, sizeof(m_meshBMinCur));
	std::memset(&m_meshBMaxCur, 0, sizeof(m_meshBMaxCur));
}

bool InputGeom::loadMesh(
	const char* navMeshName,
	const char* staticMesh,
	const char* vobsMesh,
    const char* markedMesh,
    float bvhGridSize,
	float xMinOffsetCut,
	float xMaxOffsetCut,
	float zMinOffsetCut,
	float zMaxOffsetCut
) {
	release();
    m_meshExt.reset(new(std::nothrow) MeshLoaderObjExt);
	if (!m_meshExt)
	{
		m_log->log(common::LogCategory::LOG_ERROR, "loadMesh(dir): Out of memory 'm_mesh'");
		return false;
	}
#ifdef PRINT_STRUCTURE_STAT
    auto tp1 = std::chrono::steady_clock::now();
#endif
	m_log->log(
        common::LogCategory::LOG_WARNING,
        "Loading static: %s, vobs: %s, marked: %s",
        staticMesh, vobsMesh, markedMesh
    );
	fixOffsets(xMinOffsetCut, xMaxOffsetCut, zMinOffsetCut, zMaxOffsetCut);
    auto ret = m_meshExt->load(
		staticMesh, vobsMesh, markedMesh, xMinOffsetCut,
		xMaxOffsetCut, zMinOffsetCut,  zMaxOffsetCut
	);
	if (!ret.isOk())
	{
		m_log->log(common::LogCategory::LOG_ERROR, "loadMesh(dir): could not load mesh, "
				 "static: %s, vobs: %s, marked: %s, codes: 0 = %d, 1 = %d, 2 = %d, 3 = %d",
				 staticMesh, vobsMesh, markedMesh, (int)ret.code0, (int)ret.code1,
				 (int)ret.code2, (int)ret.code3);
		return false;
	}

#ifdef PRINT_STRUCTURE_STAT
    auto tp2 = std::chrono::steady_clock::now();
#endif
	m_space.release();
	m_space.init(m_log.get());
	int loadRet = m_space.load(m_meshExt.get(), static_cast<int>(bvhGridSize));
	if (loadRet != mesh::Grid2dBvh::SUCCESSFUL)
	{
		m_log->log(common::LogCategory::LOG_ERROR, "loadMesh(dir): mesh bvh construction error");
		return false;
	}
	m_space.getBounds(m_meshBMinCur, m_meshBMaxCur);

#ifdef PRINT_STRUCTURE_STAT
    auto tp3 = std::chrono::steady_clock::now();
    auto diffParsing = std::chrono::duration_cast<std::chrono::milliseconds>(tp2 - tp1).count();
    auto diffConstr = std::chrono::duration_cast<std::chrono::milliseconds>(tp3 - tp2).count();
	m_log->log(
		common::LogCategory::LOG_WARNING,
        "loadMesh(dir), mesh parsing: %f sec, bvh construction: %f sec",
        (double)diffParsing / 1000, (double)diffConstr / 1000
    );
    m_space.printStat();
#endif

    return true;
}

void InputGeom::fixOffsets(
	float& xMinOffsetCut,
	float& xMaxOffsetCut,
	float& zMinOffsetCut,
	float& zMaxOffsetCut
) {
	xMinOffsetCut += m_meshBMinCur[0] - m_meshBMin[0];
	xMaxOffsetCut += m_meshBMax[0] - m_meshBMaxCur[0];
	zMinOffsetCut += m_meshBMinCur[2] - m_meshBMin[2];
	zMaxOffsetCut += m_meshBMax[2] - m_meshBMaxCur[2];
}

void InputGeom::updateOffsets(
	float xMinOffsetCut,
	float xMaxOffsetCut,
	float zMinOffsetCut,
	float zMaxOffsetCut,
	bool showOffsetPlanes
) {
	m_xMinOffsetCut = xMinOffsetCut;
	m_xMaxOffsetCut = xMaxOffsetCut;
	m_zMinOffsetCut = zMinOffsetCut;
	m_zMaxOffsetCut = zMaxOffsetCut;
	m_showOffsetPlanes = showOffsetPlanes;
}

bool InputGeom::loadFromDir(BuildContext* ctx, const char* filepath, float bvhGridSize)
{
	m_staticMeshName = filepath;
	m_staticMeshName.append(1, common::Constants::SEP).append("static_mesh.obj");
	m_vobsMeshName = filepath;
	m_vobsMeshName.append(1, common::Constants::SEP).append("vobs_mesh.obj");
	m_markedMeshName = filepath;
	m_markedMeshName.append(1, common::Constants::SEP).append("marked_mesh.obj");
	m_navMeshName = filepath;
	m_navMeshName.append(1, common::Constants::SEP).append("navmesh.bin");
	m_baseMeshName = filepath;

	zeroBboxes();
	m_log = std::make_unique<LoggerAdapter>(ctx);
    bool ret = loadMesh(
		m_navMeshName.c_str(), m_staticMeshName.c_str(), m_vobsMeshName.c_str(),
		m_markedMeshName.c_str(), bvhGridSize, 0.f, 0.f, 0.f, 0.f
	);
	m_space.getBounds(m_meshBMin, m_meshBMax);
	return ret;
}

bool InputGeom::saveBinaryMesh() const
{
	return m_space.saveBinaryMesh();
}

void InputGeom::cutMesh(float offsetXmin, float offsetXmax, float offsetZmin, float offsetZmax)
{
	loadMesh(
		m_navMeshName.c_str(), m_staticMeshName.c_str(), m_vobsMeshName.c_str(),
		m_markedMeshName.c_str(), (float)m_space.getCellSize(), offsetXmin, offsetXmax,
		offsetZmin, offsetZmax
	);
}

int InputGeom::getOverlappingRectCellIds(
	const float* min, const float* max, int* cellIds, int idsSize
) const {
    return m_space.getOverlappingRectCellIds(min, max, cellIds, idsSize);
}

const mesh::Grid2dBvh::TrianglesData& InputGeom::extractOverlappingRectData(
	int cellId
) const {
	return m_space.extractOverlappingRectData(cellId);
}

bool InputGeom::obbCollDetect(const geometry::OBBExt* be) const
{
#ifdef PRINT_TOTAL_COLLISION_STAT
	static uint64_t callCnt = 0;
    static uint64_t nsCnt = 0;
    auto tp1 = std::chrono::steady_clock::now();
#endif // PRINT_TOTAL_COLLISION_STAT
	bool ret = m_space.obbTriCollisionFirstHit(be);
#ifdef PRINT_TOTAL_COLLISION_STAT
	auto tp2 = std::chrono::steady_clock::now();
	auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
	++callCnt;
	nsCnt += diff;
	if (nsCnt % PRINT_PER_N_CALLS == 0) {
		m_ctx->log(RC_LOG_PROGRESS, "Obb vs tri(first) +%llu calls, av. time: %llu ns",
			PRINT_PER_N_CALLS, nsCnt / callCnt);
	}
#endif // PRINT_TOTAL_COLLISION_STAT
    return ret;
}

bool InputGeom::raycastMesh(
    const float* src, const float* dst, float& tmin, bool nearestHit
) const {
#ifdef PRINT_TOTAL_COLLISION_STAT
    static uint64_t callCntFh = 0;
	static uint64_t callCntNh = 0;
    static uint64_t nsCntFh = 0;
	static uint64_t nsCntNh = 0;
	static double lenFh = 0.;
	static double lenNh = 0.;
    auto tp1 = std::chrono::steady_clock::now();
#endif // PRINT_TOTAL_COLLISION_STAT
	bool ret;

	if (nearestHit) {
		ret = m_space.segTriCollisionNearestHit(src, dst, tmin);
#ifdef PRINT_TOTAL_COLLISION_STAT
		auto tp2 = std::chrono::steady_clock::now();
		auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
		++callCntNh;
		nsCntNh += diff;
		lenNh += rcVdist(src, dst);
		if (callCntNh % PRINT_PER_N_CALLS == 0) {
			m_ctx->log(RC_LOG_PROGRESS, "Seg vs tri(nearest) +%llu calls, av. time: %llu ns, av. len.: %f",
				PRINT_PER_N_CALLS, nsCntNh / callCntNh, lenNh / callCntNh);
		}
#endif // PRINT_TOTAL_COLLISION_STAT
	}
	else {
		ret = m_space.segTriCollisionFirstHit(src, dst, tmin);
#ifdef PRINT_TOTAL_COLLISION_STAT
		auto tp2 = std::chrono::steady_clock::now();
		auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
		++callCntFh;
		nsCntFh += diff;
		lenFh += rcVdist(src, dst);
		if (callCntFh % PRINT_PER_N_CALLS == 0) {
			m_ctx->log(RC_LOG_PROGRESS, "Seg vs tri(first) +%llu calls, av. time: %llu ns, av. len.: %f",
				PRINT_PER_N_CALLS, nsCntFh / callCntFh, lenFh / callCntFh);
		}
#endif // PRINT_TOTAL_COLLISION_STAT
	}

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
	if (ret != mesh::Grid2dBvh::SUCCESSFUL) {
		m_log->log(common::LogCategory::LOG_ERROR, "Error of addOffMeshConn, code: %d", ret);
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
		const mesh::Grid2dBvh::OffMeshData& data = m_space.getOffMeshData();
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

void InputGeom::drawCutPlanes(struct duDebugDraw* dd)
{	
	if (!m_showOffsetPlanes) return;
	
	float xMinOffsetCut = m_xMinOffsetCut;
	float xMaxOffsetCut = m_xMaxOffsetCut;
	float zMinOffsetCut = m_zMinOffsetCut;
	float zMaxOffsetCut = m_zMaxOffsetCut;
	fixOffsets(xMinOffsetCut, xMaxOffsetCut, zMinOffsetCut, zMaxOffsetCut);

	glDisable(GL_CULL_FACE);
	unsigned int colXmin = duRGBA(255, 0, 0, 255);
	unsigned int colXmax = duRGBA(0, 255, 0, 255);
	unsigned int colZmin = duRGBA(0, 0, 255, 255);
	unsigned int colZmax = duRGBA(255, 255, 0, 255);
	dd->begin(DU_DRAW_QUADS);

	dd->vertex(m_meshBMinCur[0] + m_xMinOffsetCut, m_meshBMinCur[1], m_meshBMinCur[2], colXmin);
	dd->vertex(m_meshBMinCur[0] + m_xMinOffsetCut, m_meshBMinCur[1], m_meshBMaxCur[2], colXmin);
	dd->vertex(m_meshBMinCur[0] + m_xMinOffsetCut, m_meshBMaxCur[1], m_meshBMaxCur[2], colXmin);
	dd->vertex(m_meshBMinCur[0] + m_xMinOffsetCut, m_meshBMaxCur[1], m_meshBMinCur[2], colXmin);

	dd->vertex(m_meshBMaxCur[0] - m_xMaxOffsetCut, m_meshBMinCur[1], m_meshBMinCur[2], colXmax);
	dd->vertex(m_meshBMaxCur[0] - m_xMaxOffsetCut, m_meshBMinCur[1], m_meshBMaxCur[2], colXmax);
	dd->vertex(m_meshBMaxCur[0] - m_xMaxOffsetCut, m_meshBMaxCur[1], m_meshBMaxCur[2], colXmax);
	dd->vertex(m_meshBMaxCur[0] - m_xMaxOffsetCut, m_meshBMaxCur[1], m_meshBMinCur[2], colXmax);

	dd->vertex(m_meshBMinCur[0], m_meshBMinCur[1], m_meshBMinCur[2] + m_zMinOffsetCut, colZmin);
	dd->vertex(m_meshBMaxCur[0], m_meshBMinCur[1], m_meshBMinCur[2] + m_zMinOffsetCut, colZmin);
	dd->vertex(m_meshBMaxCur[0], m_meshBMaxCur[1], m_meshBMinCur[2] + m_zMinOffsetCut, colZmin);
	dd->vertex(m_meshBMinCur[0], m_meshBMaxCur[1], m_meshBMinCur[2] + m_zMinOffsetCut, colZmin);

	dd->vertex(m_meshBMinCur[0], m_meshBMinCur[1], m_meshBMaxCur[2] - m_zMaxOffsetCut, colZmax);
	dd->vertex(m_meshBMaxCur[0], m_meshBMinCur[1], m_meshBMaxCur[2] - m_zMaxOffsetCut, colZmax);
	dd->vertex(m_meshBMaxCur[0], m_meshBMaxCur[1], m_meshBMaxCur[2] - m_zMaxOffsetCut, colZmax);
	dd->vertex(m_meshBMinCur[0], m_meshBMaxCur[1], m_meshBMaxCur[2] - m_zMaxOffsetCut, colZmax);

	dd->end();
	glEnable(GL_CULL_FACE);
}

int InputGeom::getConvexVolumeCount() const {
	return m_space.getMarkedAreaSize();
}

const mesh::MarkedArea* InputGeom::getConvexVolume(int i) const {
	return m_space.getMarkedArea(i);
}

void InputGeom::addConvexVolume(
	const float* verts,
	const int nverts,
	const float minh,
	const float maxh,
    unsigned char area
) {
	if (mesh::Grid2dBvh::SUCCESSFUL != m_space.addMarkedArea(verts, nverts, minh, maxh, area)) {
		m_log->log(common::LogCategory::LOG_ERROR, "Error of addMarkedArea");
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
		const mesh::MarkedArea& ma = *m_space.getMarkedArea(idx);
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
	return m_navMeshName.c_str();
}

const char* InputGeom::getMarkedMeshName() const
{
	return m_markedMeshName.c_str();
}

const char* InputGeom::getBaseMeshName() const
{
	return m_baseMeshName.c_str();
}

const MeshLoaderObjExt* InputGeom::getMeshExt() const
{
	return m_meshExt.get();
}

const float* InputGeom::getMeshBoundsMin() const
{
	return m_meshBMinCur;
}

const float* InputGeom::getMeshBoundsMax() const
{
	return m_meshBMaxCur;
}
