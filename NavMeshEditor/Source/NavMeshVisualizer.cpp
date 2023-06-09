
#include "NavMeshVisualizer.h"
#include "Recast.h"
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "DetourNavMesh.h"
#include "DetourCommon.h"
#include "DetourAssert.h"
#include "DetourDebugDraw.h"
#include <cstring>
#include <cstdio>
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif


NavMeshVisualizerTool::NavMeshVisualizerTool():
	m_sample(),
	m_hitPoly(),
	m_extractPrelimData(),
	m_calcPrelimData(),
	m_showPolyBbox(),
	m_showJmpFwdBbox(),
	m_showJmpDownBbox(),
	m_showClimbBbox(),
	m_showClimbOverlappedBbox()
{
	std::memset(m_polyConnectionEdges, 0, sizeof(m_polyConnectionEdges));
	std::memset(m_polySelectionEdges, 0, sizeof(m_polySelectionEdges));
}

void NavMeshVisualizerTool::init(Sample* sample)
{
	m_sample = sample;
}

void NavMeshVisualizerTool::reset()
{
	m_hitPoly = 0;
}

void NavMeshVisualizerTool::handleMenu()
{
	dtNavMesh* nav = m_sample->getNavMesh();
	if (!m_hitPoly)
		return;

	char msg[64];
	std::sprintf(msg, "Polygon id: %llu", m_hitPoly);
	imguiLabel(msg);
	if (imguiCheck("Show jump forward bbox", m_showJmpFwdBbox))
	{
		m_showJmpFwdBbox = !m_showJmpFwdBbox;
	}
	if (imguiCheck("Show jump down bbox", m_showJmpDownBbox))
	{
		m_showJmpDownBbox = !m_showJmpDownBbox;
	}
	if (imguiCheck("Show climb bbox", m_showClimbBbox))
	{
		m_showClimbBbox = !m_showClimbBbox;
	}

	for (int i = 0; i < DT_VERTS_PER_POLYGON; ++i)
	{
		std::sprintf(msg, "Edge %d", i);
		if (m_polyConnectionEdges[i] && imguiCheck(msg, m_polySelectionEdges[i], true, 10))
		{
			m_polySelectionEdges[i] = !m_polySelectionEdges[i];
		}
	}

	if (imguiCheck("Show climb overlapped bbox", m_showClimbOverlappedBbox))
	{
		m_showClimbOverlappedBbox = !m_showClimbOverlappedBbox;
	}

	if (imguiButton("Show bbox"))
	{
		m_showPolyBbox = true;
	}
	if (imguiButton("Clear bbox"))
	{
		m_showPolyBbox = false;
	}
	imguiSeparator();
	if (imguiButton("Extract prelim. data"))
	{
		m_extractPrelimData = true;
	}
	if (imguiButton("Calc prelim. data"))
	{
		m_calcPrelimData = true;
	}
}

void NavMeshVisualizerTool::handleClick(const float* s, const float* p, bool shift)
{
	rcIgnoreUnused(s);
	rcIgnoreUnused(shift);

	if (!m_sample)
		return;
	dtNavMesh* nav = m_sample->getNavMesh();
	if (!nav)
		return;
	dtNavMeshQuery* query = m_sample->getNavMeshQuery();
	if (!query)
		return;

	const float halfExtents[3] = { 20, 50, 20 };
	dtQueryFilter filter;
	const dtMeshTile* tile{};
	const dtPoly* poly{};

	dtVcopy(m_hitPos, p);
	query->findNearestPoly(p, halfExtents, &filter, &m_hitPoly, nullptr);
	if (!m_hitPoly)
		return;
	std::memset(m_polyConnectionEdges, 0, sizeof(m_polyConnectionEdges));
	std::memset(m_polySelectionEdges, 0, sizeof(m_polySelectionEdges));
	m_showPolyBbox = false;
	nav->getTileAndPolyByRef(m_hitPoly, &tile, &poly);
	if (!poly) {
		m_sample->getContext()->log(
			RC_LOG_ERROR,
			"Error of findNearestPoly for ref: '%llu' at position, x: %f, y: %f, z: %f",
			m_hitPoly, p[0], p[1], p[2]
		);
		m_hitPoly = 0;
		return;
	}
	for (int i = 0, n = poly->vertCount; i < n; ++i)
	{
		m_polyConnectionEdges[i] = !poly->neis[i];
	}
}

void NavMeshVisualizerTool::handleToggle()
{
	;
}

void NavMeshVisualizerTool::handleStep()
{
	;
}

void NavMeshVisualizerTool::handleUpdate(const float dt)
{
	dtIgnoreUnused(dt);

	if (!m_hitPoly)
		return;
	if (!m_sample->getNavMesh())
		return;
	if (!m_sample->getJmpNavMeshQuery())
		return;
	if (!m_sample->getInputGeom())
		return;

	if (m_extractPrelimData)
	{
		extractPreliminaryJumpData();
		m_extractPrelimData = false;
	}
	if (m_calcPrelimData)
	{
		if(m_sample->getInputGeom())
			calcPreliminaryJumpData();
		m_calcPrelimData = false;
	}
}

void NavMeshVisualizerTool::extractPreliminaryJumpData()
{
	const dtNavMesh* nav = m_sample->getNavMesh();
	const dtMeshTile* tile{};
	const dtPoly* poly{};
	nav->getTileAndPolyByRefUnsafe(m_hitPoly, &tile, &poly);
	rcContext* ctx = m_sample->getContext();

	ctx->log(RC_LOG_PROGRESS, "Extraction preliminary data for poly ref: %llu", m_hitPoly);
	for (int i = 0, n = poly->vertCount; i < n; ++i)
	{
		if (/*poly->neis[i] || */!poly->getEdgeJumpClimbFlags(i))
			continue;

		ctx->log(RC_LOG_PROGRESS, "   Edge: %d, jmp_down: %d, jmp_fwd: %d, climb: %d",
			i, poly->canJumpDownFromPoly(i), poly->canJumpForwardFromPoly(i), poly->canClimbFromPoly(i));
	}
	ctx->log(RC_LOG_PROGRESS, "   Climb overlapped: %d", poly->canClimbOverlappedFromPoly());
}

void NavMeshVisualizerTool::calcPreliminaryJumpData()
{
	const dtNavMesh* nav = m_sample->getNavMesh();
	const dtMeshTile* tile{};
	const dtPoly* poly{};
	nav->getTileAndPolyByRefUnsafe(m_hitPoly, &tile, &poly);
	rcContext* ctx = m_sample->getContext();

	ctx->log(RC_LOG_PROGRESS, "Calculation preliminary data for poly ref: %llu", m_hitPoly);

	for (int i = 0, n = poly->vertCount; i < n; ++i)
	{
		if (poly->neis[i])
			continue;
		if (!m_polySelectionEdges[i])
			continue;

		calcPreliminaryJumpFwdDownData(i, tile, poly);
		calcPreliminaryClimbData(i, tile, poly);
	}
	calcPreliminaryClimbOverlappedData(tile, poly);
}

void NavMeshVisualizerTool::calcPreliminaryJumpFwdDownData(const int edgeIdx, const dtMeshTile* tile, const dtPoly* poly)
{
	// TODO move such args to GUI
	static constexpr float CHECK_BBOX_HEIGHT = 250.f;
	static constexpr float SHRINK_COEFF = 0.95f;
	//
	const float checkBboxFwdDst = (tile->header->walkableRadius + 1.0f / tile->header->bvQuantFactor) * 2.f;
	static const int DIRS_NUM = geometry::Obb::DIRS_SIZE;
	static const int VERTS_NUM = geometry::Obb::VERTS_SIZE;
	float dirs[3 * DIRS_NUM];
	float verts[3 * VERTS_NUM];
	float v1[3], v2[3];
	float polyCenter[3];
	rcContext* ctx = m_sample->getContext();
	dtJmpNavMeshQuery* query = m_sample->getJmpNavMeshQuery();
	InputGeom* geom = m_sample->getInputGeom();

	const int polyVertsNum = poly->vertCount;
	geometry::vcopy(v1, &tile->verts[poly->verts[edgeIdx] * 3]);
	geometry::vcopy(v2, &tile->verts[poly->verts[(edgeIdx + 1) % polyVertsNum] * 3]);
	dtNavMesh::calcPolyCenter(tile, poly, polyCenter);
	geometry::Obb obb;
	bool res = dtJmpNavMeshQuery::calcObbDataForJumpingForwardDown(
		v1, v2, polyCenter, checkBboxFwdDst, CHECK_BBOX_HEIGHT, SHRINK_COEFF, verts, dirs
	);
	if (!res) {
		// too little poly
		return;
	}

	obb.init(dirs, verts);
	if (geom->obbCollDetect(&obb)) {
		ctx->log(RC_LOG_PROGRESS, "Found collision for jumping forward/down at edge: %d", edgeIdx);
	}
	else {
		ctx->log(RC_LOG_PROGRESS, "Not found collision for jumping forward/down at edge: %d", edgeIdx);
	}
}

void NavMeshVisualizerTool::calcPreliminaryClimbData(const int edgeIdx, const dtMeshTile* tile, const dtPoly* poly)
{
	// TODO move such args to GUI
	const float CLIMB_OVERLAPPED_MAX_HEIGHT = 450.f;
	//
	const float checkBboxFwdDst = (tile->header->walkableRadius + 1.0f / tile->header->bvQuantFactor) * 2.f;
	const float minClimbHeight = tile->header->walkableClimb - 1.0f / tile->header->bvQuantFactor;
	static const int DIRS_NUM = geometry::Obb::DIRS_SIZE;
	static const int VERTS_NUM = geometry::Obb::VERTS_SIZE;
	float dirs[3 * DIRS_NUM];
	float verts[3 * VERTS_NUM];
	float v1[3], v2[3];
	float polyCenter[3];
	rcContext* ctx = m_sample->getContext();
	dtJmpNavMeshQuery* query = m_sample->getJmpNavMeshQuery();

	const int polyVertsNum = poly->vertCount;
	geometry::vcopy(v1, &tile->verts[poly->verts[edgeIdx] * 3]);
	geometry::vcopy(v2, &tile->verts[poly->verts[(edgeIdx + 1) % polyVertsNum] * 3]);
	dtNavMesh::calcPolyCenter(tile, poly, polyCenter);
	bool res = dtJmpNavMeshQuery::calcObbDataForClimbing(
		v1, v2, polyCenter, checkBboxFwdDst, minClimbHeight,
		CLIMB_OVERLAPPED_MAX_HEIGHT, verts, dirs
	);
	if (!res) {
		// too little poly
		return;
	}

	float min[3], max[3];
	dtQueryFilter filter;
	geometry::calcAabb(verts, VERTS_NUM, min, max);
	dtFindCollidedPolysQuery<7, 256> collider(
		query, dirs, DIRS_NUM, verts, VERTS_NUM, 256
	);
	dtStatus status = query->queryPolygonsAabb(min, max, &filter, &collider);
	if (dtStatusSucceed(status)) {
		const int n = collider.getPolysNum();
		ctx->log(RC_LOG_PROGRESS, "Found for edge: %d climbing polys: %d", edgeIdx, n);
		for (int i = 0; i < n; ++i)
		{
			dtPolyRef id = collider.getPoly(i);
			ctx->log(RC_LOG_PROGRESS, "  Poly ref: %llu", id);
		}
	}
	else {
		ctx->log(RC_LOG_PROGRESS, __FUNCTION__": Error of polys finding");
	}
}

void NavMeshVisualizerTool::calcPreliminaryClimbOverlappedData(const dtMeshTile* tile, const dtPoly* poly)
{	
	// TODO move such args to GUI
	const float CLIMB_OVERLAPPED_MAX_HEIGHT = 450.f;
	//
	const float minClimbOverlappedHeight = tile->header->walkableHeight - 1.0f / tile->header->bvQuantFactor;
	static const int NUM_DIRS = MAX_PLANES_PER_BOUNDING_POLYHEDRON;
	static const int NUM_VERTS = (NUM_DIRS - 1) * 2;
	float verts[3 * NUM_VERTS];
	float dirs[3 * NUM_DIRS];
	const int polyVertsNum = poly->vertCount;
	float min[3], max[3];
	dtQueryFilter filter;
	rcContext* ctx = m_sample->getContext();
	dtJmpNavMeshQuery* query = m_sample->getJmpNavMeshQuery();

	dtJmpNavMeshQuery::calcObpDataForOverlappedClimbing(
		tile, poly, minClimbOverlappedHeight, CLIMB_OVERLAPPED_MAX_HEIGHT, verts, dirs
	);
	geometry::calcAabb(verts, polyVertsNum * 2, min, max);
	dtFindCollidedPolysQuery<7, 256> collider(
		query, dirs, polyVertsNum + 1, verts, polyVertsNum * 2, 256
	);

	dtStatus status = query->queryPolygonsAabb(min, max, &filter, &collider);
	if (dtStatusSucceed(status)) {
		const int n = collider.getPolysNum();
		ctx->log(RC_LOG_PROGRESS, "Found overlapped climbing polys: %d", n);
		for (int i = 0; i < n; ++i)
		{
			dtPolyRef id = collider.getPoly(i);
			ctx->log(RC_LOG_PROGRESS, "  Poly ref: %llu", id);
		}
	}
	else {
		ctx->log(RC_LOG_PROGRESS, __FUNCTION__": Error of polys finding");
	}
}

void NavMeshVisualizerTool::renderClimbOverlappedBbox(const dtMeshTile* tile, const dtPoly* poly)
{
	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
		return;
	dtJmpNavMeshQuery* query = m_sample->getJmpNavMeshQuery();
	if (!query)
		return;

	// TODO move such args to GUI
	const float CLIMB_OVERLAPPED_MAX_HEIGHT = 450.f;
	//
	const float minClimbOverlappedHeight = tile->header->walkableHeight - 1.0f / tile->header->bvQuantFactor;
	static const int NUM_DIRS = MAX_PLANES_PER_BOUNDING_POLYHEDRON;
	static const int NUM_VERTS = (NUM_DIRS - 1) * 2;
	float verts[3 * NUM_VERTS];
	float dirs[3 * NUM_DIRS];
	const int polyVertsNum = poly->vertCount;

	dtJmpNavMeshQuery::calcObpDataForOverlappedClimbing(
		tile, poly, minClimbOverlappedHeight, CLIMB_OVERLAPPED_MAX_HEIGHT, verts, dirs
	);

	renderObp(verts, polyVertsNum * 2);
}

void NavMeshVisualizerTool::renderClimbBbox(const int edgeIdx, const dtMeshTile* tile, const dtPoly* poly)
{
	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
		return;
	dtJmpNavMeshQuery* query = m_sample->getJmpNavMeshQuery();
	if (!query)
		return;
	
	// TODO move such args to GUI
	const float CLIMB_OVERLAPPED_MAX_HEIGHT = 450.f;
	//
	const float checkBboxFwdDst = (tile->header->walkableRadius + 1.0f / tile->header->bvQuantFactor) * 2.f;
	const float minClimbHeight = tile->header->walkableClimb - 1.0f / tile->header->bvQuantFactor;
	static const int DIRS_NUM = geometry::Obb::DIRS_SIZE;
	static const int VERTS_NUM = geometry::Obb::VERTS_SIZE;
	float dirs[3 * DIRS_NUM];
	float verts[3 * VERTS_NUM];
	float v1[3], v2[3];
	float polyCenter[3];

	const int polyVertsNum = poly->vertCount;
	assert(edgeIdx < polyVertsNum);
	geometry::vcopy(v1, &tile->verts[poly->verts[edgeIdx] * 3]);
	geometry::vcopy(v2, &tile->verts[poly->verts[(edgeIdx + 1) % polyVertsNum] * 3]);
	dtNavMesh::calcPolyCenter(tile, poly, polyCenter);
	bool res = dtJmpNavMeshQuery::calcObbDataForClimbing(
		v1, v2, polyCenter, checkBboxFwdDst, minClimbHeight,
		CLIMB_OVERLAPPED_MAX_HEIGHT, verts, dirs
	);
	if (!res) {
		// too little poly
		return;
	}

	renderObp(verts, geometry::Obb::VERTS_SIZE);
}

void NavMeshVisualizerTool::renderJumpFwdDownBbox(
	const int edgeIdx, const dtMeshTile* tile, const dtPoly* poly
) {
	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
		return;

	// TODO move such args to GUI
	static constexpr float CHECK_BBOX_HEIGHT = 250.f;
	static constexpr float SHRINK_COEFF = 0.95f;
	//
	const float checkBboxFwdDst = (tile->header->walkableRadius + 1.0f / tile->header->bvQuantFactor) * 2.f;
	static const int DIRS_NUM = geometry::Obb::DIRS_SIZE;
	static const int VERTS_NUM = geometry::Obb::VERTS_SIZE;
	float dirs[3 * DIRS_NUM];
	float verts[3 * VERTS_NUM];
	float v1[3], v2[3];
	float polyCenter[3];

	const int polyVertsNum = poly->vertCount;
	assert(edgeIdx < polyVertsNum);
	geometry::vcopy(v1, &tile->verts[poly->verts[edgeIdx] * 3]);
	geometry::vcopy(v2, &tile->verts[poly->verts[(edgeIdx + 1) % polyVertsNum] * 3]);
	dtNavMesh::calcPolyCenter(tile, poly, polyCenter);
	//geometry::Obb obb;
	bool res = dtJmpNavMeshQuery::calcObbDataForJumpingForwardDown(
		v1, v2, polyCenter, checkBboxFwdDst, CHECK_BBOX_HEIGHT, SHRINK_COEFF, verts, dirs
	);
	if (!res) {
		// too little poly
		return;
	}

	renderObp(/*obb.getVerts()*/verts, geometry::Obb::VERTS_SIZE);
}

void NavMeshVisualizerTool::renderObp(const float* vertices, const uint32_t verticesNum)
{
	assert((verticesNum & 1) == 0); // even verticesNum value
	const uint32_t verticesNumHalf = verticesNum >> 1;
	const float* lowerPlane = vertices;
	const float* upperPlane = vertices + 3 * verticesNumHalf;
	const uint32_t color = duRGBA(0, 0, 255, 255);
	duDebugDraw& dd = m_sample->getDebugDraw();
	dd.begin(DU_DRAW_LINES, 2.0f);
	for (uint32_t i = 0, j = verticesNumHalf - 1; i < verticesNumHalf; ++i, ++j)
	{
		const float* v1 = lowerPlane + i * 3;
		const float* v2 = lowerPlane + (j % verticesNumHalf) * 3;
		dd.vertex(v1[0], v1[1], v1[2], color);
		dd.vertex(v2[0], v2[1], v2[2], color);
		v1 = upperPlane + i * 3;
		v2 = upperPlane + (j % verticesNumHalf) * 3;
		dd.vertex(v1[0], v1[1], v1[2], color);
		dd.vertex(v2[0], v2[1], v2[2], color);
		v1 = lowerPlane + i * 3;
		v2 = upperPlane + i * 3;
		dd.vertex(v1[0], v1[1], v1[2], color);
		dd.vertex(v2[0], v2[1], v2[2], color);
	}
	dd.end();
}

void NavMeshVisualizerTool::handleRender()
{
	if (!m_hitPoly)
		return;
	const dtNavMesh* nav = m_sample->getNavMesh();
	if (!nav)
		return;

	duDebugDraw& dd = m_sample->getDebugDraw();
	duDebugDrawNavMeshPoly(&dd, *nav, m_hitPoly, duRGBA(0, 0, 0, 128));

	const dtMeshTile* tile{};
	const dtPoly* poly{};
	nav->getTileAndPolyByRefUnsafe(m_hitPoly, &tile, &poly);

	if (m_showPolyBbox) {
		for (int i = 0; i < DT_VERTS_PER_POLYGON; ++i)
		{
			if (!m_polySelectionEdges[i])
				continue;

			if (m_showJmpFwdBbox) {
				renderJumpFwdDownBbox(i, tile, poly);
			}
			if (m_showJmpDownBbox) {
				renderJumpFwdDownBbox(i, tile, poly);
			}
			if (m_showClimbBbox) {
				renderClimbBbox(i, tile, poly);
			}
		}
		if (m_showClimbOverlappedBbox) {
			renderClimbOverlappedBbox(tile, poly);
		}
	}
}

void NavMeshVisualizerTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	// Tool help
	const int h = view[3];
	imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB: Click select poly", imguiRGBA(255, 255, 255, 192));

	// draw edge indices
	const dtNavMesh* nav = m_sample->getNavMesh();
	if (!nav || !m_hitPoly)
		return;
	const dtMeshTile* tile{};
	const dtPoly* poly{};
	nav->getTileAndPolyByRefUnsafe(m_hitPoly, &tile, &poly);
	float v1[3], v2[3];
	float edgeCenter[3];
	char text[32];
	for (int i = 0, n = poly->vertCount; i < n; ++i)
	{
		geometry::vcopy(v1, &tile->verts[poly->verts[i] * 3]);
		geometry::vcopy(v2, &tile->verts[poly->verts[(i + 1) % n] * 3]);
		geometry::vadd(edgeCenter, v1, v2);
		geometry::vmul(edgeCenter, 0.5f);

		double x, y, z;
		auto res = gluProject((double)edgeCenter[0], (double)edgeCenter[1], (double)edgeCenter[2],
			model, proj, view, &x, &y, &z);
		if (!res)
			continue;
		std::sprintf(text, "%d", i);
		imguiDrawText(
			(int)x, (int)y, IMGUI_ALIGN_CENTER, text, imguiRGBA(0, 0, 0, 220)
		);
	}
}