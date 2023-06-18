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

#ifndef RECASTSAMPLE_H
#define RECASTSAMPLE_H

#include "Common.h"
#include "Recast.h"
#include "SampleInterfaces.h"

/// Tool types.
enum SampleToolType
{
	TOOL_NONE = 0,
	TOOL_TILE_EDIT,
	//TOOL_TILE_HIGHLIGHT,
	//TOOL_TEMP_OBSTACLE,
	TOOL_NAVMESH_TESTER,
	//TOOL_NAVMESH_PRUNE,
	//TOOL_OFFMESH_CONNECTION,
	TOOL_CONVEX_VOLUME,
	//TOOL_CROWD,
	TOOL_NAVMESH_VISUALIZER,
	MAX_TOOLS
};

class SampleDebugDraw : public DebugDrawGL
{
public:
	virtual unsigned int areaToCol(unsigned int area);
};

enum SamplePartitionType
{
	SAMPLE_PARTITION_WATERSHED,
	SAMPLE_PARTITION_MONOTONE,
	SAMPLE_PARTITION_LAYERS,
};

struct SampleTool
{
	virtual ~SampleTool() {}
	virtual int type() = 0;
	virtual void init(class Sample* sample) = 0;
	virtual void reset() = 0;
	virtual void handleMenu() = 0;
	virtual void handleClick(const float* s, const float* p, bool shift) = 0;
	virtual void handleRender() = 0;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) = 0;
	virtual void handleToggle() = 0;
	virtual void handleStep() = 0;
	virtual void handleUpdate(const float dt) = 0;
};

struct SampleToolState {
	virtual ~SampleToolState() {}
	virtual void init(class Sample* sample) = 0;
	virtual void reset() = 0;
	virtual void handleRender() = 0;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) = 0;
	virtual void handleUpdate(const float dt) = 0;
};

class Sample
{
protected:
	class InputGeom* m_geom;
	class dtNavMesh* m_navMesh;
	class dtNavMeshQuery* m_navQuery;
	class dtJmpNavMeshQuery* m_JmpNavQuery;
	class dtCrowd* m_crowd;

	unsigned char m_navMeshDrawFlags;

	float m_camSpeed;
	float m_threadsNum;
	float m_threadsMax;

	float m_cellSize;
	float m_cellHeight;
	float m_agentHeight;
	float m_agentLiquidWalk;
	float m_agentLiquidFord;
	float m_agentLiquidSwim;
	float m_agentRadius;
	float m_agentMaxClimb;
	float m_agentMaxSlope;
	float m_regionMinSize;
	float m_regionMergeSize;
	float m_edgeMaxLen;
	float m_edgeMaxError;
	float m_vertsPerPoly;
	float m_detailSampleDist;
	float m_detailSampleMaxError;
	int m_partitionType;
	float m_prelimBoxFwdDst;
	float m_prelimBoxFwdClimbDst;
	float m_prelimBoxHeight;
	float m_prelimMinClimbHeight;
	float m_prelimMinClimbOverlappedHeight;
	float m_prelimMaxClimbHeight;
	float m_prelimBboxShrinkCoeff;

	bool m_filterLowHangingObstacles;
	bool m_filterLedgeSpans;
	bool m_filterWalkableLowHeightSpans;
	bool m_erodeBorderSpans;
	
	SampleTool* m_tool;
	SampleToolState* m_toolStates[MAX_TOOLS];
	
	BuildContext* m_ctx;

	SampleDebugDraw m_dd;
	VboDebugDraw m_ddVboMesh;
	VboDebugDraw m_ddVboNvm;
	VboDebugDraw m_ddVboNvmMisc;
	VboDebugDraw m_ddVboNvmTile;

protected:
	dtNavMesh* loadAll(const char* path);
	dtNavMesh* loadAll(FILE* fp);
	void saveAll(const char* path, const dtNavMesh* mesh);
	void saveAll(FILE* fp, const dtNavMesh* mesh);

public:
	Sample();
	virtual ~Sample();
	Sample(const Sample&) = delete;
	Sample& operator=(const Sample&) = delete;
	
	void setContext(BuildContext* ctx) { m_ctx = ctx; }
	rcContext* getContext() const { return m_ctx; }
	
	void setTool(SampleTool* tool);
	SampleToolState* getToolState(int type) { return m_toolStates[type]; }
	void setToolState(int type, SampleToolState* s) { m_toolStates[type] = s; }

	SampleDebugDraw& getDebugDraw() { return m_dd; }
	void resetDrawers();
	void resetNavMeshDrawers();

	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	virtual void handleClick(const float* s, const float* p, bool shift);
	virtual void handleToggle();
	virtual void handleStep();
    virtual void handleRender(const float* cameraPos = nullptr);
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged(class InputGeom* geom);
	virtual bool handleBuild();
	virtual void handleUpdate(const float dt);
	virtual void interruptAsyncBuilding();
	virtual float getAsyncBuildingProgress() const;
	virtual bool isAsyncBuilding() const;

	virtual class InputGeom* getInputGeom() { return m_geom; }
	virtual class dtNavMesh* getNavMesh() { return m_navMesh; }
	virtual class dtNavMeshQuery* getNavMeshQuery() { return m_navQuery; }
	virtual class dtJmpNavMeshQuery* getJmpNavMeshQuery() { return m_JmpNavQuery; }
	virtual class dtCrowd* getCrowd() { return m_crowd; }
	virtual float getAgentRadius() const { return m_agentRadius; }
	virtual float getAgentHeight() const { return m_agentHeight; }
	virtual float getAgentClimb() const { return m_agentMaxClimb; }
	virtual float getPrelimBoxFwdDst() const { return m_prelimBoxFwdDst; }
	virtual float getPrelimBoxFwdClimbDst() const { return m_prelimBoxFwdClimbDst; }
	virtual float getPrelimBoxHeight() const { return m_prelimBoxHeight; }
	virtual float getPrelimMinClimbHeight() const { return m_prelimMinClimbHeight; }
	virtual float getPrelimMinClimbOverlappedHeight() const { return m_prelimMinClimbOverlappedHeight; }
	virtual float getPrelimMaxClimbHeight() const { return m_prelimMaxClimbHeight; }
	virtual float getPrelimBboxShrinkCoeff() const { return m_prelimBboxShrinkCoeff; }

	float getCamSpeed () const { return m_camSpeed; }
	
	unsigned char getNavMeshDrawFlags() const { return m_navMeshDrawFlags; }
	void setNavMeshDrawFlags(unsigned char flags) { m_navMeshDrawFlags = flags; }

	void updateToolStates(const float dt);
	void initToolStates(Sample* sample);
	void resetToolStates();
	void renderToolStates();
	void renderOverlayToolStates(double* proj, double* model, int* view);

	void resetCommonSettings();
	void handleCommonSettings();
};


#endif // RECASTSAMPLE_H
