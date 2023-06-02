
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


NavMeshVisualizerTool::NavMeshVisualizerTool():
	m_sample(),
	m_hitPoly(),
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
		std::sprintf(msg, "Edge %d", i + 1);
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
}

void NavMeshVisualizerTool::handleClick(const float* s, const float* p, bool shift)
{
	rcIgnoreUnused(s);
	rcIgnoreUnused(shift);

	if (!m_sample) return;
	InputGeom* geom = m_sample->getInputGeom();
	if (!geom) return;
	dtNavMesh* nav = m_sample->getNavMesh();
	if (!nav) return;
	dtNavMeshQuery* query = m_sample->getNavMeshQuery();
	if (!query) return;

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
	;
}

void NavMeshVisualizerTool::renderClimbOverlappedBbox(const dtMeshTile* tile, const dtPoly* poly)
{
	const float CLIMB_OVERLAPPED_MIN_HEIGHT = 180.f;
	const float CLIMB_OVERLAPPED_MAX_HEIGHT = 450.f;
	static dtPolyRef oldRef = 0;

	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
		return;
	dtJmpNavMeshQuery* query = m_sample->getJmpNavMeshQuery();
	if (!query)
		return;

	rcContext* ctx = m_sample->getContext();
	duDebugDraw& dd = m_sample->getDebugDraw();
	float verts[3 * 6];
	const int vertsNum = poly->vertCount;
	for (int i = 0; i < vertsNum; ++i) {
		geometry::vcopy(verts + i * 3, &tile->verts[poly->verts[i] * 3]);
	}

	//renderObpForOverlClimbing(m_ctx, m_startRef, m_jmpPathFinder, dd, poly->norm, poly->dist, verts, n, 180.f, 450.f);
	float points[3 * 6 * 2];
	float* halfPart = points + 3 * vertsNum;
	for (int i = 0, n = 3 * vertsNum; i < n; i += 3)
	{
		geometry::calcVerticalVertexProjectionOnPlane(verts + i, poly->norm, poly->dist, points + i);
		geometry::vcopy(halfPart + i, points + i);
		(points + i)[1] += tile->header->walkableHeight;// CLIMB_OVERLAPPED_MIN_HEIGHT;
		(halfPart + i)[1] += CLIMB_OVERLAPPED_MAX_HEIGHT;
	}

	if (oldRef != m_hitPoly)
	{
		float dirs[3 * 7];
		bool lastDirCalced = false;
		float* lastDir = dirs + vertsNum * 3;
		for (int i = 0, j = vertsNum - 1; i < vertsNum; ++i, ++j)
		{
			const float* v0 = points + (i + 1) * 3;
			const float* v1 = points + i * 3;
			const float* v2 = points + (j % vertsNum) * 3;
			geometry::calcPerpToEdgeXz(v1, v2, dirs + i);
			if (!lastDirCalced && i < vertsNum - 1)
			{
				float e1[3], e2[3];
				geometry::vsub(e1, v0, v1);
				geometry::vsub(e2, v2, v1);
				geometry::vcross(lastDir, e1, e2);
				if (geometry::vlen(lastDir) > 1e-3)
					lastDirCalced = true;
			}
		}
		if (!lastDirCalced)
		{
			assert(false);
			lastDir[0] = lastDir[2] = 0.f;
			lastDir[1] = 1.f;
		}

		float min[3], max[3];
		dtQueryFilter filter;
		dtFindCollidedPolysQuery<7, 256> collider(
			query, dirs, vertsNum + 1, points, vertsNum * 2, 256
		);
		filter.setIncludeFlags(common::SamplePolyFlags::SAMPLE_POLYFLAGS_ALL ^
			common::SamplePolyFlags::SAMPLE_POLYFLAGS_DISABLED);
		filter.setExcludeFlags(0);
		geometry::calcAabb(points, vertsNum * 2, min, max);
		dtStatus status = query->queryPolygonsAabb(min, max, &filter, &collider);
		if (dtStatusSucceed(status)) {
			int n = collider.getPolysNum();
			ctx->log(RC_LOG_PROGRESS, "Found polys: %d", n);
			for (int i = 0; i < n; ++i)
			{
				dtPolyRef id = collider.getPoly(i);
				ctx->log(RC_LOG_PROGRESS, "  Poly ref: %llu", id);
			}
		}
		else {
			ctx->log(RC_LOG_PROGRESS, "Error of polys finding");
		}
		oldRef = m_hitPoly;
	}

	const uint32_t color = duRGBA(0, 0, 255, 255);
	const float* first = points;
	const float* second = points + 3 * vertsNum;
	dd.begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0, j = vertsNum - 1; i < vertsNum; ++i, ++j)
	{
		const float* v1 = first + i * 3;
		const float* v2 = first + (j % vertsNum) * 3;
		dd.vertex(v1[0], v1[1], v1[2], color);
		dd.vertex(v2[0], v2[1], v2[2], color);
		v1 = second + i * 3;
		v2 = second + (j % vertsNum) * 3;
		dd.vertex(v1[0], v1[1], v1[2], color);
		dd.vertex(v2[0], v2[1], v2[2], color);
		v1 = first + i * 3;
		v2 = second + i * 3;
		dd.vertex(v1[0], v1[1], v1[2], color);
		dd.vertex(v2[0], v2[1], v2[2], color);
	}
	dd.end();
}

void NavMeshVisualizerTool::handleRender()
{
	if (!m_hitPoly) return;
	
	duDebugDraw& dd = m_sample->getDebugDraw();
	const dtNavMesh* nav = m_sample->getNavMesh();
	if (nav && m_hitPoly)
	{
		duDebugDrawNavMeshPoly(&dd, *nav, m_hitPoly, duRGBA(0, 0, 0, 128));
	}

	const dtMeshTile* tile{};
	const dtPoly* poly{};
	nav->getTileAndPolyByRefUnsafe(m_hitPoly, &tile, &poly);

	if (m_showPolyBbox) {
		for (int i = 0; i < DT_VERTS_PER_POLYGON; ++i)
		{
			if (!m_polySelectionEdges[i])
				continue;

			if (m_showJmpFwdBbox) {
				;
			}
			if (m_showJmpDownBbox) {
				;
			}
			if (m_showClimbBbox) {
				;
			}
		}
		if (m_showClimbOverlappedBbox) {
			renderClimbOverlappedBbox(tile, poly);
		}
	}
}

void NavMeshVisualizerTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	rcIgnoreUnused(model);
	rcIgnoreUnused(proj);

	// Tool help
	const int h = view[3];

	imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB: Click select poly", imguiRGBA(255, 255, 255, 192));
}