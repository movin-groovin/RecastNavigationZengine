
#include "Mesh.h"

#include <chrono>
#include <tuple>

namespace mesh
{

Octree::Octree(
#ifdef LOGGING_ENABLED
	common::BaseLogger* log,
#endif
	int maxDepth,
	int minTrisInLeaf
):
#ifdef LOGGING_ENABLED
    m_log(log),
#endif
	m_maxDepth(maxDepth),
    m_minTrisInLeaf(minTrisInLeaf)
{}

bool Octree::Load(MeshLoaderInterface* mesh)
{
    m_polyNum = mesh->getTriCount();
	m_vertsNum = mesh->getVertCount();
	m_tris.reset(new(std::nothrow) int[3 * m_polyNum]);
	m_verts.reset(new(std::nothrow) float[3 * m_vertsNum]);
	if (!m_tris || !m_verts) {
		return false;
	}
	std::memcpy(m_tris.get(), mesh->getTris(), sizeof(int) * 3 * m_polyNum);
	std::memcpy(m_verts.get(), mesh->getVerts(), sizeof(float) * 3 * m_vertsNum);
	const int* tris = m_tris.get();
	const float* verts = m_verts.get();

    geometry::Aabb3D* bboxes = new(std::nothrow) geometry::Aabb3D[m_polyNum];
    if (!bboxes) {
        m_memInsufficient = true;
        return false;
    }
	std::unique_ptr<std::unique_ptr<int[]>[]> constrTimeIds(
		new(std::nothrow) std::unique_ptr<int[]>[m_maxDepth]
	);
	if (!constrTimeIds) {
		return false;
	}
    for (int i = 0; i < m_maxDepth + 1; ++i) {
		constrTimeIds[i].reset(new(std::nothrow) int [m_polyNum]);
		if (!constrTimeIds[i]) {
			return false;
		}
		std::memset(constrTimeIds[i].get(), 0, sizeof(int) * m_polyNum);
    }
    memset(m_bmax, 0, sizeof(m_bmax));
    m_bmin[0] = m_bmin[1] = m_bmin[2] = std::numeric_limits<float>::max();
	std::unique_ptr<int[]>& zeroLevel = constrTimeIds[0];
    for (int i = 0; i < m_polyNum; ++i) {
        calcAabb(verts, &tris[i * 3], &bboxes[i]);
        bboxes[i].polyIndex = i;
        geometry::vmin(m_bmin, bboxes[i].min);
		geometry::vmax(m_bmax, bboxes[i].max);
        zeroLevel[i] = i;
    }
	geometry::vcopy(m_root.bmin, m_bmin);
	geometry::vcopy(m_root.bmax, m_bmax);
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
			constrTimeIds, 1, newCenter, halfSpan, verts, tris, bboxes, m_polyNum
        );
    }
    m_averPolysInLeaf /= m_leafNum;
    delete [] bboxes;
    return true;
}

Octree::Node* Octree::LoadDo(
	std::unique_ptr<std::unique_ptr<int[]>[]>& constrTimeIds,
    int depth, const float* center, const float* span, const float* verts,
    const int* tris, const geometry::Aabb3D* bboxes, int polysNum
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
    const auto& curLevel = constrTimeIds[depth - 1];
    auto& nextLevel = constrTimeIds[depth];
    int curPolysNum = 0;
    for (int i = 0, j = 0; i < polysNum; ++i) {
        const geometry::Aabb3D* box = &bboxes[curLevel[i]];
        if (geometry::checkAabbsCollision(cur->bmin, cur->bmax, box->min, box->max)) {
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
			constrTimeIds, depth + 1, newCenter, childHalfSpan,
			verts, tris, bboxes, curPolysNum
        );
    }

    return cur;
}

bool Octree::checkOverlapAabb(
	const float* start, const float* end, const float* bmin, const float* bmax
) {
	float tmin = 0.f;
	float tmax = 1.f;
	return geometry::isectSegAabb(start, end, bmin, bmax, tmin, tmax);
}

#ifdef PRINT_STRUCTURE_STAT
void Octree::printStat() const
{
	m_log->log(common::LogCategory::LOG_PROGRESS, "Delta x: %f, y: %f, z: %f\n", m_bmax[0] - m_bmin[0],
		m_bmax[1] - m_bmin[1], m_bmax[2] - m_bmin[2]);
	m_log->log(common::LogCategory::LOG_PROGRESS, "Polys num: %d, leafs num: %d, average polys in leaf: %f, "
		"max polys in leaf: %d, min polys in leaf: %d, current max depth: "
		"%d", m_polyNum, m_leafNum, m_averPolysInLeaf,
		m_maxPolysInLeaf, m_minPolysInLeaf, m_curMaxDepthReached
	);
}
#endif

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
#ifdef PRINT_TOTAL_COLLISION_STAT
	m_log->log(
		common::LogCategory::LOG_PROGRESS,
		"%d = time diff ns: %d, total nodes: %d, total leafes: %d, total polys: %d",
        n, (int)diff, m_totalNodes, m_leafNodes, m_polys
	);
#endif
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
            const int* vIds = &m_tris[cur->m_trianglePolys[i] * 3];
            ++m_polys;
            if (geometry::intersectSegmentTriangle(
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

void Octree::calcAabb(const float* verts, const int* triangle, geometry::Aabb3D* bbox)
{
	geometry::vcopy(bbox->min, &verts[triangle[0] * 3]);
	geometry::vcopy(bbox->max, &verts[triangle[0] * 3]);
    for (int i = 1; i < 3; ++i) {
		geometry::vmin(bbox->min, &verts[triangle[i] * 3]);
		geometry::vmax(bbox->max, &verts[triangle[i] * 3]);
    }
}

Grid2dBvh::Grid2dBvh() {
	clearState();
}

Grid2dBvh::~Grid2dBvh() {
	release();
}

bool Grid2dBvh::isLoaded() const {
	return m_grid;
}

void Grid2dBvh::release() {
	delete[] m_grid;
	m_grid = nullptr;
	delete[] m_tris;
	m_tris = nullptr;
	delete[] m_triFlags;
	m_triFlags = nullptr;
	common::freeAlignedArr<float>(m_verts, 0);
	m_verts = nullptr;
	delete[] m_vobs;
	m_vobs = nullptr;
	delete[] m_vobsMeshes;
	m_vobsMeshes = nullptr;
	m_moverNameToVob.release();
	m_overlappingRectData.release();
	m_markedAreas.release();
	m_offMeshConns.release();
#ifdef RENDERING_ENABLED
	m_renderingData.release();
#endif
	clearState();
}

void Grid2dBvh::init(
#ifdef LOGGING_ENABLED
	common::BaseLogger* log
#endif
) {
#ifdef LOGGING_ENABLED
	m_log = log;
#endif
}

void Grid2dBvh::clearState()
{
	m_cellSize = 0;
	m_cellSizeInv = 0.f;
#ifdef USAGE_SSE_1_0
	m_cellSizeInvVec = _mm_setr_ps(0.f, 0.f, 0.f, 0.f);
#endif
	m_cellsNum = 0;
	m_trisNum = 0;
	m_vertsNum = 0;
	m_vobsNum = 0;
	m_vobsMeshesNum = 0;
#ifdef USAGE_SSE_1_0
	m_worldMinVecXzXz = _mm_setr_ps(0.f, 0.f, 0.f, 0.f);
#endif
	m_worldMin[0] = m_worldMin[1] = m_worldMin[2] = std::numeric_limits<float>::max();
	m_worldMax[0] = m_worldMax[1] = m_worldMax[2] = -std::numeric_limits<float>::max();
	m_worldSize[0] = m_worldSize[1] = m_worldSize[2] = 0.f;
	m_wszCellsX = 0;
	m_wszCellsY = 0;
	m_wszCellsZ = 0;
#ifdef LOGGING_ENABLED
	m_log = nullptr;
#endif
#ifdef PRINT_STRUCTURE_STAT
	m_totalNodes = 0;
	m_leafNodes = 0;
	m_internalNodes = 0;
	m_maxDepth = 0;
	m_curDepth = 0;
	m_maxTrisInGridCell = 0;
	m_maxBoxesInGridCell = 0;
	m_bytesPerConstruction = 0;
	m_bytesForData = 0;
#endif
#if (PRINT_TRI_VS_SEG_LATENCY || PRINT_TRI_VS_OBB_LATENCY)
	totalNodesTraversed = 0;
	totalLeafesTraversed = 0;
	totalPolysTraversed = 0;
#endif
}

void Grid2dBvh::calc3DAabb(const float* verts, const int* triangle, geometry::Aabb3D* bbox, int vertsBlock)
{
	geometry::vcopy(bbox->min, &verts[triangle[0] * vertsBlock]);
	geometry::vcopy(bbox->max, &verts[triangle[0] * vertsBlock]);
	for (int i = 1; i < 3; ++i) {
		geometry::vmin(bbox->min, &verts[triangle[i] * vertsBlock]);
		geometry::vmax(bbox->max, &verts[triangle[i] * vertsBlock]);
	}
}

bool Grid2dBvh::checkTriangleBelongAabb(
	const float* bmin, const float* bmax, const int* triangle
) const {
	const float* v0 = &m_verts[triangle[0] * Constants::CUR_VERTS_BLOCK];
	const float* v1 = &m_verts[triangle[1] * Constants::CUR_VERTS_BLOCK];
	const float* v2 = &m_verts[triangle[2] * Constants::CUR_VERTS_BLOCK];
	return geometry::intersectionAabbVsTriangle(bmin, bmax, v0, v1, v2);
}

void Grid2dBvh::fillPolyFlags(
	FlagType* flagsTo,
	const FlagType* flagsFrom,
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

int Grid2dBvh::constructVobs(MeshLoaderInterface* mesh)
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
	m_vobsMeshes = new(std::nothrow) BvhVobMeshEntry[m_vobsMeshesNum];
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
			Position& pos = vobTo.positions[j];
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
						delete[] cell.vobResidence;
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
		delete[] cell.vobResidence;
		cell.vobResidence = tmp;
#ifdef PRINT_STRUCTURE_STAT
		m_bytesForData += sizeof(GridCell::VobPosResidence) * cell.vobResidencesNum;
#endif
	}

	int maxTrisNum = mesh->getMaxTrisPerVob();
	std::unique_ptr<geometry::Aabb3D[]> bboxes(new(std::nothrow) geometry::Aabb3D[maxTrisNum]);
	std::unique_ptr<int[]> boxIds(new(std::nothrow) int[maxTrisNum]);
	if (!bboxes || !boxIds) {
		return ERROR_NO_MEMORY;
	}
	const VobMeshEntry* vobsMesh = mesh->getVobMeshes();
	for (int i = 0; i < m_vobsMeshesNum; ++i) {
		const VobMeshEntry& vm = vobsMesh[i];
		BvhVobMeshEntry& vmCp = m_vobsMeshes[i];
		if (vm.isEmpty()) {
			// TODO log
			continue;
		}
		if (!vmCp.copy(vm)) {
			return ERROR_NO_MEMORY;
		}
		int* tris = vm.tris;
		const int trisNum = vmCp.triCount;
		for (int j = 0; j < trisNum; ++j)
		{
			calc3DAabb(vm.verts, tris + j * 3, bboxes.get() + j, Constants::REGULAR_VERTS_BLOCK);
			bboxes[j].polyIndex = j;
			boxIds[j] = j;
		}
		auto dat = makeBvh(bboxes.get(), boxIds.get(), trisNum);
		if (!dat.first) {
			return ERROR_NO_MEMORY;
		}
		m_vobsMeshes[i].childs = dat.first;
		m_vobsMeshes[i].childsNumber = dat.second;
		fillPolyFlags(vmCp.flags, vm.flags, trisNum);
	}

#ifdef PRINT_STRUCTURE_STAT
	m_bytesPerConstruction +=
		sizeof(int) * m_cellsNum + (sizeof(geometry::Aabb3D) + sizeof(int)) * maxTrisNum;
	m_bytesForData += sizeof(VobEntry) * m_vobsNum + sizeof(BvhVobMeshEntry) * m_vobsMeshesNum;
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
		trafo[0] * vertex[0] + trafo[1] * vertex[1] + trafo[2] * vertex[2] + trafo[3]/**1.f*/;
	vertexNew[1] =
		trafo[4] * vertex[0] + trafo[5] * vertex[1] + trafo[6] * vertex[2] + trafo[7]/**1.f*/;
	vertexNew[2] =
		trafo[8] * vertex[0] + trafo[9] * vertex[1] + trafo[10] * vertex[2] + trafo[11]/**1.f*/;
}

void Grid2dBvh::transformVertex(
	const float* vertex, const Position* pos, float* vertexNew
) {
	if (pos)
		transformVertex(vertex, pos->trafo, vertexNew);
	else
		geometry::vcopy(vertexNew, vertex);
}

void Grid2dBvh::transformDirection(const float* normal, const float* trafo, float* normalNew)
{
	// 0: 0-3
	// 0: 4-7
	// 0: 8-11
	// 0: 12-15
	normalNew[0] =
		trafo[0] * normal[0] + trafo[1] * normal[1] + trafo[2] * normal[2]/* + trafo[3]*0.f*/;
	normalNew[1] =
		trafo[4] * normal[0] + trafo[5] * normal[1] + trafo[6] * normal[2]/* + trafo[7]*0.f*/;
	normalNew[2] =
		trafo[8] * normal[0] + trafo[9] * normal[1] + trafo[10] * normal[2]/* + trafo[11]*0.f*/;
	if (normalNew[1] < 0.f) {
		normalNew[0] *= -1.f;
		normalNew[1] *= -1.f;
		normalNew[2] *= -1.f;
	}
}

#ifdef RENDERING_ENABLED
int Grid2dBvh::constructRenderingData(MeshLoaderInterface* mesh)
{
	m_renderingData.vertsNum = mesh->getVertCountStatic();
	m_renderingData.trisNum = mesh->getTriCountStatic();
	int vobsVertsNum = 0, vobsTrisNum = 0;
	for (int i = 0; i < m_vobsNum; ++i)
	{
		const VobEntry& vob = m_vobs[i];
		const BvhVobMeshEntry& vmesh = m_vobsMeshes[vob.meshIndex];
		vobsVertsNum += vmesh.vertCount;
		vobsTrisNum += vmesh.triCount;
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

	for (int i = 0, vPos = 0; i < vertsPos; i += 3, vPos += Constants::CUR_VERTS_BLOCK) {
		geometry::vcopy(m_renderingData.verts + i, m_verts + vPos);
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
		const auto& vobMesh = m_vobsMeshes[vob.meshIndex];
		const auto& vobPos = vob.positions[vob.activePosIndex];
		const float* verts = vobMesh.verts;
		const int* tris = vobMesh.tris;
		const FlagType* flags = vobMesh.flags;
		const float* normals = vobMesh.normals;
		vob.vertsPosRendering = vertsPos / 3;

		for (int j = 0; j < vobMesh.vertCount; ++j, vertsPos += 3) {
			transformVertex(
				verts + j * Constants::CUR_VERTS_BLOCK, vobPos.trafo, m_renderingData.verts + vertsPos
			);
			(m_renderingData.verts + vertsPos)[2] *= -1.f;
		}
		for (
			int j = 0, m = vobMesh.triCount * 3;
			j < m;
			j += 3, trisPos += 3, ++flagsPos, ++flags
			) {
			m_renderingData.tris[trisPos] = tris[j] / Constants::CUR_VERTS_BLOCK + deltaVerts;
			m_renderingData.tris[trisPos + 1] = tris[j + 1] / Constants::CUR_VERTS_BLOCK + deltaVerts;
			m_renderingData.tris[trisPos + 2] = tris[j + 2] / Constants::CUR_VERTS_BLOCK + deltaVerts;
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
#endif // RENDERING_ENABLED

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
			const auto& mesh = m_vobsMeshes[vob.meshIndex];
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

void Grid2dBvh::copyDataFromBvh(
	Grid2dBvh::TrianglesData& resData,
	int& trisPos,
	int& vertsPos,
	const BvhNode* curNode,
	const BvhNode* endNode,
	const Position* pos,
	const float* verts,
	const int* tris,
	const FlagType* triFlags
) {
	while (curNode < endNode) {
		if (curNode->triId >= 0) {
			FlagType flag = triFlags[curNode->triId / 3];
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
		if (vob.isDoor()/* || vob.isLadder()*/) {
			continue;
		}
		const BvhVobMeshEntry& vMesh = m_vobsMeshes[vob.meshIndex];
		assert(vob.activePosIndex < vob.posCnt);
		const Position* pos = &vob.positions[vob.activePosIndex];
		curNode = vMesh.childs;
		endNode = curNode + vMesh.childsNumber;
		// TODO replace to explicit extracting from mesh of resident vobs
		copyDataFromBvh(
			customData, trisPos, vertsPos, curNode, endNode, pos,
			vMesh.verts, vMesh.tris, vMesh.flags
		);
	}
	customData.vertsNumCurrent = vertsPos / 3;
	customData.trisNumCurrent = trisPos / 3;
}

#ifdef RENDERING_ENABLED
const Grid2dBvh::TrianglesData& Grid2dBvh::getRenderingData() const
{
	return m_renderingData;
}
#endif

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
		for (int j = 0; j < Position::POS_TRIS_NUM; ++j) {
			m_triFlags[posOld.aabbTris[i]].isActiveVobPos = false;
		}
		vob.activePosIndex = stateId;
		const auto& posNew = vob.positions[vob.activePosIndex];
		for (int j = 0; j < Position::POS_TRIS_NUM; ++j) {
			m_triFlags[posNew.aabbTris[i]].isActiveVobPos = true;
		}
#ifdef RENDERING_ENABLED
		const auto& vobMesh = m_vobsMeshes[vob.meshIndex];
		const float* verts = vobMesh.verts;
		float* resVerts = &m_renderingData.verts[vob.vertsPosRendering * 3];
		for (int j = 0, n = vobMesh.vertCount * 3; j < n; j += 3, resVerts += 3) {
			transformVertex(verts + j, posNew.trafo, resVerts);
		}
#endif
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
	return { xiMin, xiMax, ziMin, ziMax };
}

int Grid2dBvh::linkMarkedAreaWithGrid(
	int vertsNum, const float* verts, int eInd, MarkedEntry& e
) {
	static const int IDS_DELTA = 4;
	float bbox[3 * 4];
	float min[3], max[3];
	geometry::vcopy(min, verts);
	geometry::vcopy(max, verts);
	for (int j = 3; j < vertsNum * 3; j += 3) {
		geometry::vmin(min, verts + j);
		geometry::vmax(max, verts + j);
	}
	for (int i = 0; i < 4; ++i) {
		geometry::vcopy(bbox + i * 3, min);
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
			if (geometry::checkPolyVsPolyXz(bbox, 4, verts, vertsNum)) {
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
					delete[] cell.markedIndices;
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
					delete[] e.gridIds;
					e.gridIds = newIds;
				}
				e.gridIds[e.idsNum] = index;
				e.idsNum += 1;
			}
		}
	}

	return SUCCESSFUL;
}

int Grid2dBvh::constructMarkedAreas(MeshLoaderInterface* mesh)
{
	const int mNum = mesh->getMarkedCount();
	const MarkedArea* marked = mesh->getMarked();
	return constructMarkedAreas(mNum, marked);
}

int Grid2dBvh::constructMarkedAreas(int mNum, const MarkedArea* marked)
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
		const MarkedArea& from = marked[i];
		MarkedArea& to = data[i].data;
		to.minh = from.minh;
		to.maxh = from.maxh;
		to.area = from.area;
		to.vertsNum = from.vertsNum;
		to.verts = new(std::nothrow) float[3 * to.vertsNum];
		if (!to.verts) {
			delete[] data;
			return ERROR_NO_MEMORY;
		}
#ifdef PRINT_STRUCTURE_STAT
		m_bytesForData += sizeof(float) * 3 * to.vertsNum;
#endif
		std::memcpy(to.verts, from.verts, 3 * sizeof(float) * to.vertsNum);
		int ret = linkMarkedAreaWithGrid(to.vertsNum, to.verts, i, data[i]);
		if (ret != SUCCESSFUL) {
			delete[] data;
			return ret;
		}
	}
	m_markedAreas.num = mNum;
	m_markedAreas.size = mNum;
	m_markedAreas.data = data;

	return SUCCESSFUL;
}

const MarkedArea* Grid2dBvh::getMarkedArea(int i) const
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
				delete[] marked;
				return ret;
			}
		}
		delete[] m_markedAreas.data;
		m_markedAreas.data = marked;
	}

	MarkedEntry& newArea = m_markedAreas.data[m_markedAreas.num];
	MarkedArea& data = newArea.data;
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
	delete[] m_markedAreas.data;
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
	static const int NUM_VERTS = 20;
	static const float Y_OFFSET = 100.f;
	int num = 0;

	for (int i = 0; i < m_vobsNum; ++i)
	{
		VobEntry& vob = m_vobs[i];
		const BvhVobMeshEntry& mesh = m_vobsMeshes[vob.meshIndex];
		if (!mesh.isEmpty() && vob.isLadder()) {
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
		const BvhVobMeshEntry& mesh = m_vobsMeshes[vob.meshIndex];
		if (mesh.isEmpty()) {
			//assert(1 != 1);
			// TODO log
			continue;
		}
		if (!vob.isLadder()) {
			continue;
		}

		const float* verts = mesh.verts;
		const Position& vobPos = vob.positions[vob.activePosIndex];
		int nVerts = mesh.vertCount;
		std::unique_ptr<std::pair<float, int>[]> dat(
			new(std::nothrow) std::pair<float, int>[nVerts]
		);
		float vert[3];
		for (int j = 0; j < nVerts; ++j) {
			transformVertex(verts + j * Constants::CUR_VERTS_BLOCK, vobPos.trafo, vert);
			//vert[2] *= -1.f;
			dat[j].first = vert[1];
			dat[j].second = j;
		}
		std::sort(
			dat.get(),
			dat.get() + nVerts,
			[](const std::pair<float, int>& v1, const std::pair<float, int>& v2) {
				return v1.first < v2.first;
			}
		);
		float low[3] = { 0.f, 0.f, 0.f };
		const float yMin = dat[0].first;
		int j = 0;
		for (int n = std::min(nVerts, NUM_VERTS); j < n; ++j) {
			if (dat[j].first > yMin + Y_OFFSET) {
				break;
			}
			transformVertex(verts + dat[j].second * Constants::CUR_VERTS_BLOCK, vobPos.trafo, vert);
			vert[2] *= -1.f;
			low[0] += vert[0];
			low[1] += vert[1];
			low[2] += vert[2];
		}
		low[0] /= j;
		low[1] /= j;
		low[2] /= j;
		float high[3] = { 0.f, 0.f, 0.f };
		const float yMax = dat[nVerts - 1].first;
		j = 0;
		for (int n = std::max(nVerts - NUM_VERTS, 0), k = nVerts - 1; k >= n; ++j, --k) {
			if (dat[k].first < yMax - Y_OFFSET) {
				break;
			}
			transformVertex(verts + dat[k].second * Constants::CUR_VERTS_BLOCK, vobPos.trafo, vert);
			vert[2] *= -1.f;
			high[0] += vert[0];
			high[1] += vert[1];
			high[2] += vert[2];
		}
		high[0] /= j;
		high[1] /= j;
		high[2] /= j;
		geometry::vcopy(m_offMeshConns.offMeshVerts + m_offMeshConns.offMeshNum * 3 * 2, low);
		geometry::vcopy(m_offMeshConns.offMeshVerts + m_offMeshConns.offMeshNum * 3 * 2 + 3, high);
		m_offMeshConns.offMeshRads[m_offMeshConns.offMeshNum] = 60.f;
		m_offMeshConns.offMeshDirs[m_offMeshConns.offMeshNum] = 1;
		m_offMeshConns.offMeshAreas[m_offMeshConns.offMeshNum] = common::SamplePolyAreas::SAMPLE_POLYAREA_GROUND;
		m_offMeshConns.offMeshFlags[m_offMeshConns.offMeshNum] = common::SamplePolyFlags::SAMPLE_POLYFLAGS_WALK;
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
			delete[] offMeshVerts;
			delete[] offMeshRads;
			delete[] offMeshDirs;
			delete[] offMeshAreas;
			delete[] offMeshFlags;
			delete[] offMeshId;
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
	geometry::vcopy(m_offMeshConns.offMeshVerts + curNum * 2 * 3, spos);
	geometry::vcopy(m_offMeshConns.offMeshVerts + curNum * 2 * 3 + 3, epos);
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

bool Grid2dBvh::saveBinaryMesh() const
{
	return false;
}

int Grid2dBvh::load(MeshLoaderInterface* mesh, int cellSize)
{
	int ret = loadInternal(mesh, cellSize);
	if (ret != SUCCESSFUL) {
		release();
	}
	return ret;
}

void Grid2dBvh::getBounds(float* bMin, float* bMax) const
{
	geometry::vcopy(bMin, m_worldMin);
	geometry::vcopy(bMax, m_worldMax);
}

void Grid2dBvh::getWorldSize(float* res) const
{
	geometry::vcopy(res, m_worldSize);
}

int Grid2dBvh::getCellSize() const
{
	return m_cellSize;
}

int Grid2dBvh::loadInternal(MeshLoaderInterface* mesh, int cellSize)
{
	m_cellSize = cellSize;
	m_cellSizeInv = 1.0f / m_cellSize;
#ifdef USAGE_SSE_1_0
	m_cellSizeInvVec =
		_mm_setr_ps(m_cellSizeInv, m_cellSizeInv, m_cellSizeInv, m_cellSizeInv);
#endif
	const float* vertsCp = mesh->getVerts();
	m_vertsNum = mesh->getVertCount();
	m_verts = common::allocAlignedArr<float>(m_vertsNum * Constants::CUR_VERTS_BLOCK, 16);
	if (!m_verts) {
		return ERROR_NO_MEMORY;
	}
	for (int i = 0; i < m_vertsNum; ++i) {
		std::memcpy(m_verts + Constants::CUR_VERTS_BLOCK * i, vertsCp + 3 * i, sizeof(float) * 3);
#ifdef USAGE_SSE_1_0
		m_verts[Constants::CUR_VERTS_BLOCK * i + 3] = 0.f;
#endif
	}
	m_trisNum = mesh->getTriCount();
	m_tris = new(std::nothrow) int[m_trisNum * 3];
	m_triFlags = new(std::nothrow) FlagType[m_trisNum];
	if (!m_tris || !m_triFlags) {
		return ERROR_NO_MEMORY;
	}
	std::memcpy(m_tris, mesh->getTris(), 3 * m_trisNum * sizeof(int));
	fillPolyFlags(m_triFlags, mesh->getFlags(), m_trisNum);
#ifdef PRINT_STRUCTURE_STAT
	m_bytesForData += sizeof(float) * Constants::CUR_VERTS_BLOCK * m_vertsNum +
		sizeof(int) * 3 * m_trisNum +
		sizeof(uint32_t) * m_trisNum;
#endif
	std::unique_ptr<geometry::Aabb3D[]> bboxes(new(std::nothrow) geometry::Aabb3D[m_trisNum]);
	std::unique_ptr<int[]> boxIds(new(std::nothrow) int[m_trisNum]);
	if (!bboxes || !boxIds) {
		return ERROR_NO_MEMORY;
	}
#ifdef PRINT_STRUCTURE_STAT
	m_bytesPerConstruction += m_trisNum * (sizeof(geometry::Aabb3D) + sizeof(int));
#endif
	for (int i = 0; i < m_trisNum; ++i)
	{
		calc3DAabb(m_verts, m_tris + i * 3, bboxes.get() + i, Constants::CUR_VERTS_BLOCK);
		geometry::vmin(m_worldMin, bboxes[i].min);
		geometry::vmax(m_worldMax, bboxes[i].max);
		bboxes[i].polyIndex = i;
		boxIds[i] = i;
	}
#ifdef USAGE_SSE_1_0
	m_worldMinVecXzXz = _mm_setr_ps(m_worldMin[0], m_worldMin[2], m_worldMin[0], m_worldMin[2]);
#endif
	geometry::vsub(m_worldSize, m_worldMax, m_worldMin);
	m_wszCellsX = static_cast<int>(std::ceil(m_worldSize[0] / m_cellSize));
	m_wszCellsY = 0;
	m_wszCellsZ = static_cast<int>(std::ceil(m_worldSize[2] / m_cellSize));
	m_cellsNum = m_wszCellsX * m_wszCellsZ;
	m_grid = new(std::nothrow) GridCell[m_cellsNum];
	if (!m_grid) {
		return ERROR_NO_MEMORY;
	}
#ifdef PRINT_STRUCTURE_STAT
	m_log->log(
		common::LogCategory::LOG_WARNING,
		"World size bbox, dx: %f, dy: %f, dz: %f",
		m_worldSize[0], m_worldSize[1], m_worldSize[2]
	);
	m_log->log(
		common::LogCategory::LOG_WARNING,
		"Bmin x: %f, y: %f, z: %f; "
		"Bmax x: %f, y: %f, z: %f",
		m_worldMin[0], m_worldMin[1], m_worldMin[2],
		m_worldMax[0], m_worldMax[1], m_worldMax[2]
	);
	m_log->log(
		common::LogCategory::LOG_WARNING,
		"Size of vertices, polys and flags: %f\n",
		static_cast<float>(m_bytesForData) / (1024 * 1024)
	);
	m_bytesForData += m_cellsNum * sizeof(GridCell);
#endif
	if (SUCCESSFUL != constructVobs(mesh)) {
		return ERROR_NO_MEMORY;
	}
#ifdef RENDERING_ENABLED
	if (SUCCESSFUL != constructRenderingData(mesh)) {
		return ERROR_NO_MEMORY;
	}
#endif
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
		xSplit[i].reset(new(std::nothrow) int[PRIMARY_SIZE_X]);
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
		geometry::vcopy(cellMinMax[i].min, cellMin);
		geometry::vcopy(cellMinMax[i].max, cellMax);
	}
	std::sort(boxIds.get(), boxIds.get() + m_trisNum, geometry::CoordComparer(0, bboxes.get()));
	for (int i = 0, j = 0; j < m_trisNum; /*++j*/)
	{
		int cellStart = i;
		while (true)
		{
			if (cellStart >= m_wszCellsX) {
				++j;
				break;
			}
			bool aabbColl = geometry::checkAabbsCollision(
				cellMinMax[cellStart].min,
				cellMinMax[cellStart].max,
				bboxes[boxIds[j]].min,
				bboxes[boxIds[j]].max
			);
			if (!aabbColl) {
				if (cellStart == i) {
					++i;
				}
				else {
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
					int* tmp = new(std::nothrow) int[xSplit[cellStart][1]];
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
			zSplit[i][j].reset(new(std::nothrow) int[PRIMARY_SIZE_Z]);
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
			geometry::vcopy(cellMinMax[j].min, cellMin);
			geometry::vcopy(cellMinMax[j].max, cellMax);
		}
		std::sort(boxIds + 2, boxIds + boxIds[0], geometry::CoordComparer(2, bboxes.get()));
		for (int j = 0, k = 2; k < boxIds[0]; /*++k*/)
		{
			int cellStart = j;
			while (true)
			{
				if (cellStart >= m_wszCellsZ) {
					++k;
					break;
				}
				bool aabbColl = geometry::checkAabbsCollision(
					cellMinMax[cellStart].min,
					cellMinMax[cellStart].max,
					bboxes[boxIds[k]].min,
					bboxes[boxIds[k]].max
				);
				if (!aabbColl) {
					if (j == cellStart) {
						++j;
					}
					else {
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
						int* tmp = new(std::nothrow) int[zSplit[i][cellStart][1]];
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
				geometry::vcopy(cell->bmin, cellMin);
				geometry::vcopy(cell->bmax, cellMax);
				cell->childsNumber = ret.second;
				cell->childs = ret.first;
#ifdef PRINT_STRUCTURE_STAT
				if (trisNum > m_maxTrisInGridCell) m_maxTrisInGridCell = trisNum;
#endif
			}
			else {
				return ERROR_NO_MEMORY;
			}
			trisVertsPerCell[pos].first = trisNum * 3; // verts
			trisVertsPerCell[pos].second = trisNum; // tris
		}
	}
	for (int i = 0; i < 3 * m_trisNum; ++i) {
		m_tris[i] *= Constants::CUR_VERTS_BLOCK;
	}
	if (SUCCESSFUL != constructOverlappingRectData(std::move(trisVertsPerCell))) {
		return ERROR_NO_MEMORY;
	}
#ifdef PRINT_STRUCTURE_STAT
	m_bytesPerConstruction +=
		sizeof(geometry::BminBmaxSegmentTree::BminBmax) *
		geometry::BminBmaxSegmentTree::getResourceManager().getSize();
#endif
	geometry::BminBmaxSegmentTree::freeResources();

	return SUCCESSFUL;
}

void Grid2dBvh::subdivideMedian(
	const geometry::Aabb3D* bboxes, int* boxIds, BvhNode* bnodes, int i, int j, int& curNodeNum
#ifdef PRINT_STRUCTURE_STAT
	, int depth, int& maxBoxesInGridCell
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
		geometry::vcopy(curNode->min, bboxes[boxIds[i]].min);
		geometry::vcopy(curNode->max, bboxes[boxIds[i]].max);
		curNode->triId = bboxes[boxIds[i]].polyIndex * 3;
#ifdef PRINT_STRUCTURE_STAT
		++m_totalNodes;
		++m_leafNodes;
#endif
	}
	else {
		geometry::vcopy(curNode->min, bboxes[boxIds[i]].min);
		geometry::vcopy(curNode->max, bboxes[boxIds[i]].max);
		for (int k = i + 1; k < j; ++k) {
			geometry::vmin(curNode->min, bboxes[boxIds[k]].min);
			geometry::vmax(curNode->max, bboxes[boxIds[k]].max);
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
			std::sort(boxIds + i, boxIds + j, geometry::CoordComparer(0, bboxes));
		else if (maxAxis == 1)
			std::sort(boxIds + i, boxIds + j, geometry::CoordComparer(1, bboxes));
		else if (maxAxis == 2)
			std::sort(boxIds + i, boxIds + j, geometry::CoordComparer(2, bboxes));
		int k = i + n / 2;
		subdivideMedian(bboxes, boxIds, bnodes, i, k, curNodeNum
#ifdef PRINT_STRUCTURE_STAT
			, depth + 1, maxBoxesInGridCell
#endif
		);
		subdivideMedian(bboxes, boxIds, bnodes, k, j, curNodeNum
#ifdef PRINT_STRUCTURE_STAT
			, depth + 1, maxBoxesInGridCell
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
	const geometry::BminBmaxSegmentTree& tree,
	int i,
	int mid,
	int j,
	float* totalMin,
	float* totalMax
) {
	float bmin[3], bmax[3], diffLeft[3], diffRight[3], diffTotal[3];

	tree.calcBbox(i, mid - 1, bmin, bmax);
	geometry::vcopy(totalMin, bmin);
	geometry::vcopy(totalMax, bmax);
	geometry::vsub(diffLeft, bmax, bmin);

	tree.calcBbox(mid, j - 1, bmin, bmax);
	geometry::vmin(totalMin, bmin);
	geometry::vmax(totalMax, bmax);
	geometry::vsub(diffRight, bmax, bmin);

	geometry::vsub(diffTotal, totalMax, totalMin);
	return COST_CHECK_BBOX +
		calcPartSahValue(diffTotal, diffLeft, mid - i) +
		calcPartSahValue(diffTotal, diffRight, j - mid);
}

void Grid2dBvh::subdivideSah(
	const geometry::Aabb3D* bboxes, int* boxIds, BvhNode* bnodes, int i, int j, int& curNodeNum
#ifdef PRINT_STRUCTURE_STAT
	, int depth, int& maxBoxesInGridCell
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
				const geometry::Aabb3D* box = bboxes + boxIds[k];
				geometry::vcopy(node->min, box->min);
				geometry::vcopy(node->max, box->max);
				node->triId = box->polyIndex * 3;
				geometry::vmin(curNode->min, box->min);
				geometry::vmax(curNode->max, box->max);
#ifdef PRINT_STRUCTURE_STAT
				++m_totalNodes;
				++m_leafNodes;
				++maxBoxesInGridCell;
#endif
			}
		}
		else {
			const geometry::Aabb3D* box = bboxes + boxIds[i];
			geometry::vcopy(curNode->min, box->min);
			geometry::vcopy(curNode->max, box->max);
			curNode->triId = box->polyIndex * 3;
#ifdef PRINT_STRUCTURE_STAT
			++m_totalNodes;
			++m_leafNodes;
			++maxBoxesInGridCell;
#endif
		}
	}
	else {
		float minSah = std::numeric_limits<float>::max();
		int bestSep = -1;
		int bestAxis = -1;
		for (int axis = 0; axis < 3; ++axis) {
			std::sort(boxIds + i, boxIds + j, geometry::CoordComparer(axis, bboxes));
			geometry::BminBmaxSegmentTree tree;
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
		geometry::vsub(diff, curNode->max, curNode->min);
		float totalSah = COST_CHECK_BBOX + calcPartSahValue(diff, diff, j - i);
		if (totalSah > minSah) {
			if (bestAxis != 2)
				std::sort(boxIds + i, boxIds + j, geometry::CoordComparer(bestAxis, bboxes));
			subdivideSah(bboxes, boxIds, bnodes, i, bestSep, curNodeNum
#ifdef PRINT_STRUCTURE_STAT
				, depth + 1, maxBoxesInGridCell
#endif
			);
			subdivideSah(bboxes, boxIds, bnodes, bestSep, j, curNodeNum
#ifdef PRINT_STRUCTURE_STAT
				, depth + 1, maxBoxesInGridCell
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
		}
		else {
			curNode->triId = i - j; // negative value
			assert(curNode->triId != 0);
			for (int k = i; k < j; ++k) {
				BvhNode* node = &bnodes[curNodeNum++];
				const geometry::Aabb3D* box = bboxes + boxIds[k];
				geometry::vcopy(node->min, box->min);
				geometry::vcopy(node->max, box->max);
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
	const geometry::Aabb3D* bboxes, int* boxIds, const int trisNum
) const {
	common::ignoreUnused(&Grid2dBvh::subdivideMedian);
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
		, depth, maxBoxesInGridCell
#endif
	);
#ifdef PRINT_STRUCTURE_STAT
	m_curDepth = 0;
	if (maxBoxesInGridCell > m_maxBoxesInGridCell)
		m_maxBoxesInGridCell = maxBoxesInGridCell;
#endif
	assert(curNodeNum <= trisNum * 2);
	BvhNode* bnodesNew = common::allocAlignedArr<BvhNode>(curNodeNum, 16);
	if (!bnodesNew) {
		delete[] bnodes;
		return {};
	}
	std::memcpy(bnodesNew, bnodes, curNodeNum * sizeof(BvhNode));
	delete[] bnodes;
#ifdef PRINT_STRUCTURE_STAT
	m_bytesForData += curNodeNum * sizeof(BvhNode);
#endif
	return { bnodesNew, curNodeNum };
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
	geometry::IsectAabbArgs aabbArgs;
	geometry::calcIsectAabbArgs(aabbArgs, start, end);
	geometry::IsectTriArgs triArgs;
	calcIsectTriArgs(triArgs, start, end);
	int vobIds[VOBS_NUM_COLLIDE_CHEKING] = { 0 };
	int vobIdsIdx = 0;

	for (int i = xiMin; i <= xiMax; ++i) {
		const GridCell* xShift = m_grid + m_wszCellsZ * i;
		for (int j = ziMin; j <= ziMax; ++j) {
			const GridCell* cell = xShift + j;
			bool ret = geometry::isectSegXzAabbRed(aabbArgs, cell->bmin, cell->bmax);
			if (!ret) continue;
			const BvhNode* curNode = cell->childs;
			const BvhNode* endNode = curNode + cell->childsNumber;
			while (curNode < endNode) {
				const bool leaf = curNode->triId >= 0;
				const bool boxIntersect =
					geometry::isectSegAabbRed(aabbArgs, curNode->min, curNode->max);
				if (leaf && boxIntersect) {
					const int* vIds = m_tris + curNode->triId;
					ret = geometry::intersectSegmentTriangleRed(
						triArgs,
						m_verts + vIds[0],
						m_verts + vIds[1],
						m_verts + vIds[2],
						t
					);
					if (ret) { // TODO last collide id checking
						FlagType flag = m_triFlags[curNode->triId / 3];
						if (flag.isVobPos) {
							if (flag.isActiveVobPos) {
								int vobId = flag.vobIdOrCollFlags;
								// TODO realize at sse instructions
								bool checked = false;
								// TODO % by bitwise
								for (int k = 0; k < VOBS_NUM_COLLIDE_CHEKING && vobIds[k]; ++k) {
									checked = vobIds[k] == vobId;
									if (checked) break;
								}
								if (!checked) {
									vobIds[vobIdsIdx & VNC_CHK] = vobId;
									++vobIdsIdx;
									if (segTriCollisionVobFirstHit(vobId, start, end, t)) {
										return true;
									}
								}
							}
						}
						else {
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

bool Grid2dBvh::segTriCollisionVobFirstHit(
	int vobId, const float* start, const float* end, float& t
) const {
	const VobEntry& vob = m_vobs[vobId];
	const BvhVobMeshEntry& vobMesh = m_vobsMeshes[vob.meshIndex];
	const auto& vobPos = vob.positions[vob.activePosIndex];
	float startReal[3], endReal[3];
	transformVertex(start, vobPos.invTrafo, startReal);
	transformVertex(end, vobPos.invTrafo, endReal);
	geometry::IsectAabbArgs aabbArgs;
	geometry::calcIsectAabbArgs(aabbArgs, startReal, endReal);
	geometry::IsectTriArgs triArgs;
	calcIsectTriArgs(triArgs, startReal, endReal);
	const float* verts = vobMesh.verts;
	const int* tris = vobMesh.tris;

	const BvhNode* curNode = vobMesh.childs;
	const BvhNode* endNode = curNode + vobMesh.childsNumber;
	while (curNode < endNode) {
		const bool leaf = curNode->triId >= 0;
		const bool boxIntersect =
			geometry::isectSegAabbRed(aabbArgs, curNode->min, curNode->max);
		if (leaf && boxIntersect) {
			const int* vIds = tris + curNode->triId;
			bool ret = geometry::intersectSegmentTriangleRed(
				triArgs,
				verts + vIds[0],
				verts + vIds[1],
				verts + vIds[2],
				t
			);
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
	geometry::IsectAabbArgs aabbArgs;
	geometry::calcIsectAabbArgs(aabbArgs, start, end);
	geometry::IsectTriArgs triArgs;
	calcIsectTriArgs(triArgs, start, end);
	t = TINIT_VAL;
	int vobIds[VOBS_NUM_COLLIDE_CHEKING] = { 0 };
	int vobIdsIdx = 0;

	for (int i = xiMin; i <= xiMax; ++i) {
		const GridCell* xShift = m_grid + m_wszCellsZ * i;
		for (int j = ziMin; j <= ziMax; ++j) {
			const GridCell* cell = xShift + j;
			bool ret = geometry::isectSegXzAabbRed(aabbArgs, cell->bmin, cell->bmax);
			if (!ret) continue;
			const BvhNode* curNode = cell->childs;
			const BvhNode* endNode = curNode + cell->childsNumber;
			while (curNode < endNode) {
				const bool leaf = curNode->triId >= 0;
				const bool boxIntersect =
					geometry::isectSegAabbRed(aabbArgs, curNode->min, curNode->max);
				if (leaf && boxIntersect) {
					const int* vIds = m_tris + curNode->triId;
					ret = geometry::intersectSegmentTriangleRed(
						triArgs,
						m_verts + vIds[0],
						m_verts + vIds[1],
						m_verts + vIds[2],
						tCur
					);
					if (ret) { // TODO last collide id checking
						FlagType flag = m_triFlags[curNode->triId / 3];
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
									vobIds[vobIdsIdx & VNC_CHK] = vobId;
									++vobIdsIdx;
									if (segTriCollisionVobNearestHit(vobId, start, end, tCur)) {
										if (tCur < t) t = tCur;
									}
								}
							}
						}
						else {
							if (tCur < t) t = tCur;
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

	return t != TINIT_VAL;
}

bool Grid2dBvh::segTriCollisionVobNearestHit(
	int vobId, const float* start, const float* end, float& t
) const {
	static constexpr float TINIT_VAL = 2.f;
	const VobEntry& vob = m_vobs[vobId];
	const BvhVobMeshEntry& vobMesh = m_vobsMeshes[vob.meshIndex];
	const auto& vobPos = vob.positions[vob.activePosIndex];
	float startReal[3], endReal[3];
	transformVertex(start, vobPos.invTrafo, startReal);
	transformVertex(end, vobPos.invTrafo, endReal);
	geometry::IsectAabbArgs aabbArgs;
	geometry::calcIsectAabbArgs(aabbArgs, startReal, endReal);
	geometry::IsectTriArgs triArgs;
	calcIsectTriArgs(triArgs, startReal, endReal);
	const float* verts = vobMesh.verts;
	const int* tris = vobMesh.tris;

	t = TINIT_VAL;
	float tCur;
	const BvhNode* curNode = vobMesh.childs;
	const BvhNode* endNode = curNode + vobMesh.childsNumber;
	while (curNode < endNode) {
		const bool leaf = curNode->triId >= 0;
		const bool boxIntersect =
			geometry::isectSegAabbRed(aabbArgs, curNode->min, curNode->max);
		if (leaf && boxIntersect) {
			const int* vIds = tris + curNode->triId;
			bool ret = geometry::intersectSegmentTriangleRed(
				triArgs,
				verts + vIds[0],
				verts + vIds[1],
				verts + vIds[2],
				tCur
			);
			if (ret) {
				if (tCur < t) t = tCur;
			}
		}
		if (leaf || boxIntersect)
			++curNode;
		else
			// triId stores negative number of inside triangles
			curNode -= curNode->triId;
	}

	return t != TINIT_VAL;
}

bool Grid2dBvh::obbTriCollisionFirstHit(const geometry::OBBExt* be) const // TODO add robustness
{
	float triPoints[3 * 3];
	float min[3], max[3];
	geometry::vcopy(min, be->verts);
	geometry::vcopy(max, be->verts);
	for (int i = 1; i < 8; ++i) {
		geometry::vmin(min, be->verts + i * 3);
		geometry::vmax(max, be->verts + i * 3);
	}
	XzGridBorders ret = calcXzGridBorders(min, max);

	for (int i = ret.xiMin; i <= ret.xiMax; ++i) {
		const GridCell* xShift = m_grid + m_wszCellsZ * i;
		for (int j = ret.ziMin; j <= ret.ziMax; ++j) {
			const GridCell* cell = xShift + j;
			bool ret = geometry::checkAabbsCollisionXZ(min, max, cell->bmin, cell->bmax);
			if (!ret) continue;
			const BvhNode* curNode = cell->childs;
			const BvhNode* endNode = curNode + cell->childsNumber;
			while (curNode < endNode) {
				const bool leaf = curNode->triId >= 0;
				const bool boxIntersect =
					geometry::checkAabbsCollision(min, max, curNode->min, curNode->max);
				if (leaf && boxIntersect) {
					const int* vIds = m_tris + curNode->triId;
					geometry::vcopy(triPoints, m_verts + vIds[0]);
					geometry::vcopy(triPoints + 3, m_verts + vIds[1]);
					geometry::vcopy(triPoints + 6, m_verts + vIds[2]);
					ret = geometry::intersectObbTriangle(be, triPoints);
					if (ret) { // TODO last collide id checking
						FlagType flag = m_triFlags[curNode->triId / 3];
						if (flag.isVobPos) {
							if (flag.isActiveVobPos) {
								int vobId = flag.vobIdOrCollFlags;
								if (obbTriCollisionVobFirstHit(vobId, be)) {
									return true;
								}
							}
						}
						else {
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

bool Grid2dBvh::obbTriCollisionVobFirstHit(int vobId, const geometry::OBBExt* be) const
{
	const VobEntry& vob = m_vobs[vobId];
	const BvhVobMeshEntry& vobMesh = m_vobsMeshes[vob.meshIndex];
	const auto& vobPos = vob.positions[vob.activePosIndex];
	geometry::OBBExt* beMut = const_cast<geometry::OBBExt*>(be);
	float vertex[3];
	for (int i = 0; i < 3 * 8; i += 3) {
		transformVertex(&be->verts[i], vobPos.invTrafo, vertex);
		geometry::vcopy(&beMut->verts[i], vertex);
	}
	transformVertex(be->b.center, vobPos.invTrafo, vertex);
	geometry::vcopy(beMut->b.center, vertex);
	for (int i = 0; i < 3 * 3; i += 3) {
		transformDirection(&be->b.dir[i], vobPos.invTrafo, vertex);
		geometry::vcopy(&beMut->b.dir[i], vertex);
	}
	float triPoints[3 * 3];
	float min[3], max[3];
	geometry::vcopy(min, be->verts);
	geometry::vcopy(max, be->verts);
	for (int i = 1; i < 8; ++i) {
		geometry::vmin(min, be->verts + i * 3);
		geometry::vmax(max, be->verts + i * 3);
	}
	const float* verts = vobMesh.verts;
	const int* tris = vobMesh.tris;

	const BvhNode* curNode = vobMesh.childs;
	const BvhNode* endNode = curNode + vobMesh.childsNumber;
	while (curNode < endNode) {
		const bool leaf = curNode->triId >= 0;
		const bool boxIntersect =
			geometry::checkAabbsCollision(min, max, curNode->min, curNode->max);
		if (leaf && boxIntersect) {
			const int* vIds = tris + curNode->triId;
			geometry::vcopy(triPoints, verts + vIds[0]);
			geometry::vcopy(triPoints + 3, verts + vIds[1]);
			geometry::vcopy(triPoints + 6, verts + vIds[2]);
			bool ret = geometry::intersectObbTriangle(be, triPoints);
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
void Grid2dBvh::printStat() const
{
	m_log->log(
		common::LogCategory::LOG_WARNING,
		"Tris num: %d, verts num: %d, totalNodes: %d, m_leafNodes: %d, "
		"internalNodes: %d, maxDepth: %d, maxTrisInGridCell: %d, "
		"maxBoxesInGridCell: %d\n",
		m_trisNum, m_vertsNum, m_totalNodes, m_leafNodes,
		m_internalNodes, m_maxDepth, m_maxTrisInGridCell, m_maxBoxesInGridCell
	);
	m_log->log(
		common::LogCategory::LOG_WARNING,
		"Construction mem size MB: %f, data mem size MB: %f\n",
		static_cast<float>(m_bytesPerConstruction) / (1024 * 1024),
		static_cast<float>(m_bytesForData) / (1024 * 1024)
	);
}
#endif

#if defined(PRINT_TRI_VS_SEG_LATENCY) || defined(PRINT_TRI_VS_OBB_LATENCY)
int Grid2dBvh::getNodesPerCall() const { return totalNodesTraversed; }
int Grid2dBvh::getLeafesPerCall() const { return totalLeafesTraversed; }
int Grid2dBvh::getPolysPerCall() const { return totalPolysTraversed; }
void Grid2dBvh::clearStatPerCall() const {
	totalNodesTraversed = 0;
	totalLeafesTraversed = 0;
	totalPolysTraversed = 0;
}
#endif

} // namespace mesh