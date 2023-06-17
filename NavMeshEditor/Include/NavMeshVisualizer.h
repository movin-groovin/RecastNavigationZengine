#pragma once

#include "Sample.h"
#include "DetourNavMesh.h"

// Tool to visualize features of mesh and navmesh

class NavMeshVisualizerTool : public SampleTool
{
private:
	Sample* m_sample;
	float m_hitPos[3];
	dtPolyRef m_hitPoly;
	bool m_extractPrelimData;
	bool m_calcPrelimData;
	bool m_polyConnectionEdges[DT_VERTS_PER_POLYGON];
	bool m_polySelectionEdges[DT_VERTS_PER_POLYGON];
	bool m_showPolyBbox;
	bool m_showJmpFwdBbox;
	bool m_showJmpDownBbox;
	bool m_showClimbBbox;
	bool m_showClimbOverlappedBbox;
	bool m_showVobInfo;
	int m_vobIdxVisualize;
	bool m_showAveragePoly;

public:
	NavMeshVisualizerTool();

	virtual int type() { return TOOL_NAVMESH_VISUALIZER; }
	virtual void init(Sample* sample);
	virtual void reset();
	virtual void handleMenu();
	virtual void handleClick(const float* s, const float* p, bool shift);
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleUpdate(const float dt);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);

private:
	void renderAveragePoly(const dtMeshTile* tile, const dtPoly* poly);
	void renderOverlayVob(double* proj, double* model, int* view);
	void renderVobBbox();
	void renderClimbOverlappedBbox(const dtMeshTile* tile, const dtPoly* poly);
	void renderClimbBbox(const int edgeIdx, const dtMeshTile* tile, const dtPoly* poly);
	void renderJumpForwardOrDownBbox(const int edgeIdx, const dtMeshTile* tile, const dtPoly* poly);
	void extractPreliminaryJumpData();
	void calcPreliminaryJumpData();
	void calcPreliminaryJumpForwardOrDownData(const int edgeIdx, const dtMeshTile* tile, const dtPoly* poly);
	void calcPreliminaryClimbData(const int edgeIdx, const dtMeshTile* tile, const dtPoly* poly);
	void calcPreliminaryClimbOverlappedData(const dtMeshTile* tile, const dtPoly* poly);
	void renderObp(const float* vertices, const uint32_t verticesNum);
};