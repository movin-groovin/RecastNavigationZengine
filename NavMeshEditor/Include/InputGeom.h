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

#ifndef INPUTGEOM_H
#define INPUTGEOM_H

#include "SampleInterfaces.h"
#include "MeshLoaderObj.h"
#include "Recast.h"
#include "Common.h"
#include "Geometry.h"
#include "Mesh.h"

#include <string>
#include <utility>
#include <limits>
#include <memory>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <cassert>
#include <unordered_map>
#include <type_traits>

#include <immintrin.h>


class LoggerAdapter: public common::BaseLogger
{
public:
	LoggerAdapter(BuildContext* ctx): m_ctx(ctx) {}
	~LoggerAdapter() = default;

private:
	void doLogMessage(common::LogCategory category, const char* msg, int len) override;

private:
	BuildContext* m_ctx;
};

class alignas(__m128) InputGeom
{
public:
	InputGeom();
	~InputGeom();
	InputGeom(const InputGeom&) = delete;
	InputGeom& operator=(const InputGeom&) = delete;

	void release();
	void zeroBboxes();

    bool loadFromDir(class BuildContext* ctx, const char* filepath, float bvhGridSize);
	bool saveBinaryMesh() const;
	void updateOffsets(
		float xMinOffsetCut,
		float xMaxOffsetCut,
		float zMinOffsetCut,
		float zMaxOffsetCut,
		bool showOffsetPlanes
	);
	void fixOffsets(
		float& xMinOffsetCut,
		float& xMaxOffsetCut,
		float& zMinOffsetCut,
		float& zMaxOffsetCut
	);
	void cutMesh(float offsetXmin, float offsetXmax, float offsetZmin, float offsetZmax);

	// service calls
	const mesh::Grid2dBvh& getSpace() { return m_space; }
	int getVertCount() const;
	int getTriCount() const;
	const char* getNavMeshName() const;
	const char* getMarkedMeshName() const;
	const char* getBaseMeshName() const;

	// Method to return static mesh data
	const MeshLoaderObjExt* getMeshExt() const;
	const float* getMeshBoundsMin() const;
	const float* getMeshBoundsMax() const;

	// collisions
    bool raycastMesh(const float* src, const float* dst, float& tmin, bool nearestHit) const;
    bool obbCollDetect(const geometry::Obb* obb) const;

	// mesh extracting
    int getOverlappingRectCellIds(
		const float* min, const float* max, int* cellIds, int idsSize
	) const;
	const mesh::Grid2dBvh::TrianglesData& extractOverlappingRectData(int cellId) const;

	// Off-Mesh connections.
	int getOffMeshConnectionCount() const;
	const float* getOffMeshConnectionVerts() const;
	const float* getOffMeshConnectionRads() const;
	const unsigned char* getOffMeshConnectionDirs() const;
	const unsigned char* getOffMeshConnectionAreas() const;
	const unsigned short* getOffMeshConnectionFlags() const;
	const unsigned int* getOffMeshConnectionId() const;
	void addOffMeshConnection(
		const float* spos, const float* epos, const float rad,
		unsigned char bidir, unsigned char area, unsigned short flags
	);
	void deleteOffMeshConnection(int i);
	void drawOffMeshConnections(struct duDebugDraw* dd, bool hilight = false);
	void drawCutPlanes(struct duDebugDraw* dd);

	// Box Volumes.
	int getConvexVolumeCount() const;
	const mesh::MarkedArea* getConvexVolume(int i) const;
	void addConvexVolume(
		const float* verts,
		const int nverts,
		const float minh,
		const float maxh,
		unsigned char area
	);
	void deleteConvexVolume(int i);
	void totalDeleteMarkedAreas();
	void drawConvexVolumes(struct duDebugDraw* dd, bool hilight = false);

private:
	bool loadMesh(
		const char* navMeshName,
		const char* staticMesh,
		const char* vobsMesh,
		const char* markedMesh,
		float bvhGridSize,
		float xMinOffsetCut,
		float xMaxOffsetCut,
		float zMinOffsetCut,
		float zMaxOffsetCut
	);

private:
	std::unique_ptr<LoggerAdapter> m_log;
	//mesh::Octree m_oct;
	mesh::Grid2dBvh m_space;
	bool m_showOffsetPlanes;
	float m_xMinOffsetCut;
	float m_xMaxOffsetCut;
	float m_zMinOffsetCut;
	float m_zMaxOffsetCut;
	std::string m_staticMeshName;
	std::string m_vobsMeshName;
	std::string m_markedMeshName;
	std::string m_navMeshName;
	std::string m_baseMeshName;
	std::unique_ptr<MeshLoaderObjExt> m_meshExt;
	float m_meshBMin[3];
	float m_meshBMax[3];
	float m_meshBMinCur[3];
	float m_meshBMaxCur[3];
};

#endif // INPUTGEOM_H
