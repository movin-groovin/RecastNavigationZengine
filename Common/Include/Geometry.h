#ifndef GEOMETRY_LIB_H
#define GEOMETRY_LIB_H

#include "Common.h"

#include <cstring>
#include <cmath>
#include <cassert>
#include <cfloat>
#include <algorithm>
#include <type_traits>

#include <immintrin.h>

namespace geometry
{

// constants and PODs
struct Constants
{
	static constexpr float EPS = 1e-6f;
	static constexpr float EPS_ZERO_NORMAL = 1e-4f;
};

// functions
template <typename T>
inline T scalarMin(T a, T b)
{
	return std::min<typename std::remove_cv<typename std::remove_reference<T>::type>::type>(a, b);
}

template <typename T>
inline T scalarMax(T a, T b)
{
	return std::max<typename std::remove_cv<typename std::remove_reference<T>::type>::type>(a, b);
}

// TODO sse versions
inline void vmin(float* vres, const float* v)
{
	vres[0] = std::min(vres[0], v[0]);
	vres[1] = std::min(vres[1], v[1]);
	vres[2] = std::min(vres[2], v[2]);
}

inline void vmax(float* vres, const float* v)
{
	vres[0] = std::max(vres[0], v[0]);
	vres[1] = std::max(vres[1], v[1]);
	vres[2] = std::max(vres[2], v[2]);
}

inline void vcopy(float* dest, const float* v)
{
	dest[0] = v[0];
	dest[1] = v[1];
	dest[2] = v[2];
}

inline void vsub(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0] - v2[0];
	dest[1] = v1[1] - v2[1];
	dest[2] = v1[2] - v2[2];
}

inline void vsub(float* res, const float* v)
{
	res[0] -= v[0];
	res[1] -= v[1];
	res[2] -= v[2];
}

inline void vadd(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0] + v2[0];
	dest[1] = v1[1] + v2[1];
	dest[2] = v1[2] + v2[2];
}

inline void vadd(float* res, const float* v)
{
	res[0] += v[0];
	res[1] += v[1];
	res[2] += v[2];
}

inline void vmul(float* res, const float m)
{
	res[0] *= m;
	res[1] *= m;
	res[2] *= m;
}

inline float vlen(const float* v)
{
	return std::sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

inline float vdist(const float* v1, const float* v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return std::sqrtf(dx * dx + dy * dy + dz * dz);
}

inline void vcross(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
	dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
	dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

inline float vdot(const float* v1, const float* v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

inline void vmad(float* dest, const float* v1, const float* v2, const float s)
{
	dest[0] = v1[0] + v2[0] * s;
	dest[1] = v1[1] + v2[1] * s;
	dest[2] = v1[2] + v2[2] * s;
}

inline void calcAabbCenter(float* center, const float* bmin, const float* bmax)
{
	vadd(center, bmin, bmax);
	vmul(center, 0.5f);
}

inline bool checkAabbsCollision(
	const float* aMin, const float* aMax, const float* bMin, const float* bMax
) {
	if (aMax[0] < bMin[0] || bMax[0] < aMin[0]) return false;
	if (aMax[1] < bMin[1] || bMax[1] < aMin[1]) return false;
	if (aMax[2] < bMin[2] || bMax[2] < aMin[2]) return false;
	return true;
}

inline bool checkAabbsCollisionXZ(
	const float* aMin, const float* aMax, const float* bMin, const float* bMax
) {
	if (aMax[0] < bMin[0] || bMax[0] < aMin[0]) return false;
	if (aMax[2] < bMin[2] || bMax[2] < aMin[2]) return false;
	return true;
}

inline float vdotXz(const float* v1, const float* v2)
{
	return v1[0] * v2[0] + v1[2] * v2[2];
}

inline void vnormalize(float* v)
{
	float d = 1.0f / vlen(v);
	v[0] *= d;
	v[1] *= d;
	v[2] *= d;
}

#ifdef USAGE_SSE_1_0
inline __m128 sseVcross(__m128 a, __m128 b) {
	__m128 tmp0 = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
	__m128 tmp1 = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2));
	__m128 tmp2 = _mm_mul_ps(tmp0, b);
	__m128 tmp3 = _mm_mul_ps(tmp0, tmp1);
	__m128 tmp4 = _mm_shuffle_ps(tmp2, tmp2, _MM_SHUFFLE(3, 0, 2, 1));
	return _mm_sub_ps(tmp3, tmp4);
}

inline float sseVdot(__m128 a, __m128 b) {
	__m128 tmp = _mm_mul_ps(a, b);
	//return *(float*)&tmp + *((float*)&tmp + 1) + *((float*)&tmp + 2);
	tmp = _mm_hadd_ps(tmp, tmp); // sse3
	tmp = _mm_hadd_ps(tmp, tmp); // sse3
	return _mm_cvtss_f32(tmp);
}
#endif // USAGE_SSE_1_0

bool intersectSegmentTriangle(
	const float* sp, const float* sq, const float* a, const float* b, const float* c, float& t
);

bool isectSegAabb(
	const float* sp, const float* sq, const float* amin,
	const float* amax, float& tmin, float& tmax
);

void calcProjection(
	const float* points, int n, const float* axis, float(&minMax)[2]
);

void calcVerticalVertexProjectionOnPlane(const float* v, const float* n, const float d, float* vProj);

void makeAabbPoints(float* aabbPoints, const float* bmin, const float* bmax);

bool intersectionAabbVsTriangle(
	const float* bmin, const float* bmax, const float* v0, const float* v1, const float* v2
);

bool intersectionObbVsTriangle(const struct Obb* obb, const float* triPoints);

bool intersectionSegmentVsPlane(
	const float* n, const float d, const float* A, const float* B, float* P
);

// not tested
bool intersectionSegmentVsPlane(
	const float* v0, const float* v1, const float* v2, const float* A, const float* B, float* P
);

void calcProjectionXz(
	const float* points, int n, const float* axis, float(&minMax)[2]
);

void makeAabbPointsXz(float* aabbPoints, const float* bmin, const float* bmax);

// edge connection minds no collision ( <=, >=)
bool intersectionAabbVsTriangleXz(
	const float* bmin, const float* bmax, const float* v0, const float* v1, const float* v2
);

bool isPointInAabbXz(const float* p, const float* bmin, const float* bmax);

void calcIsectTriArgs(struct IsectTriArgs& args, const float* start, const float* end);

void transformVertex(const float* vertex, const float* trafo, float* vertexNew);

void transformDirection(const float* normal, const float* trafo, float* normalNew);

bool intersectSegmentTriangleRed(
	const struct IsectTriArgs& args,
	const float* a,
	const float* b,
	const float* c,
	float& t
);

void calcCenterAndHalfExtents(const float* verts, const int vertsSize, float* center, float* halfExtents);

void calcAabb(const float* verts, const int vertsNum, float* min, float* max);
void calcAabb16BytesAligned(const float* verts, const int vertsNum, float* min, float* max);

void calcIsectAabbArgs(struct IsectAabbArgs& args, const float* start, const float* end);

bool isectSegXzAabbRed(
	const struct IsectAabbArgs& args, const float* amin, const float* amax
);

bool isectSegAabbRed(struct IsectAabbArgs& args, const float* amin, const float* amax);

bool checkPolyVsPolyXz(
	const float* first, int firstNum, const float* second, int secondNum
);

bool calcDirOutOfPolyXz(const float* v1, const float* v2, const float* inPoly, float* dir);

// dirs consists 9 floats, points - 24 floats
void calcObbDirsAndPoints(
	const float* v1,
	const float* v2,
	const float* fwdDirNorm,
	const float fwdDst,
	const float height,
	float* dirs,
	float* points
);

inline void calcPerpToEdgeXz(const float* v1, const float* v2, float* perp)
{
	vsub(perp, v2, v1);
	perp[1] = 0.f;
	const float tmp = perp[0];
	perp[0] = perp[2];
	perp[2] = -tmp;
	vnormalize(perp);
}

// structs and classes
using transformCallable = void (const float* /*old*/, const float* /*trafo matrix*/, float* /*new*/);

struct Plane
{
	Plane() : norm{ 0.f, 0.f, 0.f }, dist(0.f) {}
	Plane(const float* n, float d) : dist(d) { std::memcpy(norm, n, 3 * sizeof(float)); }
	Plane(float nx, float ny, float nz, float d) : norm{ nx, ny, nz }, dist(d) {}

	float norm[3];
	float dist;
};

struct AabbTri
{
	float min[3];
	float max[3];
	int triIndex;
};

struct AabbVob
{
	float min[3];
	float max[3];
	int vobIndex;
};

struct Aabb
{
	float min[3];
	float max[3];
};

struct Obb
{
public:
	static const int DIRS_SIZE = 3;
	static const int VERTS_SIZE = 8;

private:
	float m_center[3];
	float m_halfSizes[3];
	float m_dirs[3 * DIRS_SIZE];
	float m_verts[3 * VERTS_SIZE];

private:
	void calcObbPoints()
	{
		// directions: forward, side, up
		const float* dirFwd = m_dirs;
		const float* dirSide = m_dirs + 3;
		const float* dirUp = m_dirs + 6;
		vadd(m_verts, m_center, dirSide);
		vsub(m_verts, m_verts, dirFwd);
		vmad(m_verts + 3, m_verts, dirFwd, 2.f);
		vmad(m_verts + 6, m_verts + 3, dirSide, -2.f);
		vmad(m_verts + 9, m_verts, dirSide, -2.f);
		vadd(m_verts + 12, m_verts, dirUp);
		vadd(m_verts + 15, m_verts + 3, dirUp);
		vadd(m_verts + 18, m_verts + 6, dirUp);
		vadd(m_verts + 21, m_verts + 9, dirUp);
		vmad(m_verts, m_verts, dirUp, -1.f);
		vmad(m_verts + 3, m_verts + 3, dirUp, -1.f);
		vmad(m_verts + 6, m_verts + 6, dirUp, -1.f);
		vmad(m_verts + 9, m_verts + 9, dirUp, -1.f);
	}

public:
	void init(
		const float* center,
		const float* halfSizes,
		const float* dirs
	) {
		std::memcpy(m_center, center, sizeof(m_center));
		std::memcpy(m_halfSizes, halfSizes, sizeof(m_halfSizes));
		std::memcpy(m_dirs, dirs, sizeof(m_dirs));
		calcObbPoints();
	}

	void init(const float* dirs, const float* verts)
	{
		float stub[] = { FLT_MAX, FLT_MAX, FLT_MAX };
		setCenter(stub);
		setHalfWidths(stub);
		setDirs(dirs);
		std::memcpy(m_verts, verts, sizeof(float) * 3 * VERTS_SIZE);
	}

	void clear()
	{
		std::memset(m_center, 0, sizeof(m_center));
		std::memset(m_halfSizes, 0, sizeof(m_halfSizes));
		std::memset(m_dirs, 0, sizeof(m_dirs));
		std::memset(m_verts, 0, sizeof(m_verts));
	}

	void copy(Obb& to) const
	{
		std::memcpy(to.m_center, m_center, sizeof(m_center));
		std::memcpy(to.m_halfSizes, m_halfSizes, sizeof(m_halfSizes));
		std::memcpy(to.m_dirs, m_dirs, sizeof(m_dirs));
		std::memcpy(to.m_verts, m_verts, sizeof(m_verts));
	}

	void transformCenter(const float* oldCenter, const float* trafoMatrix)
	{
		transformVertex(oldCenter, trafoMatrix, m_center);
	}
	void transformDirections(const float* oldDirections, const float* trafoMatrix)
	{
		transformDirection(oldDirections, trafoMatrix, m_dirs);
		transformDirection(oldDirections + 3, trafoMatrix, m_dirs + 3);
		transformDirection(oldDirections + 6, trafoMatrix, m_dirs + 6);
	}
	void transformVertices(const float* oldVertices, const float* trafoMatrix)
	{
		const float* start = oldVertices;
		const float* end = oldVertices + VERTS_SIZE * 3;
		float* pos = m_verts;
		for (; start < end; start += 3, pos += 3) {
			transformVertex(start, trafoMatrix, pos);
		}
	}
	void transform(const float* trafoMatrix)
	{
		transformCenter(m_center, trafoMatrix);
		transformDirections(m_dirs, trafoMatrix);
		transformVertices(m_verts, trafoMatrix);
	}

	int getVertsNum() const { return VERTS_SIZE; }
	const float* getVerts() const { return m_verts; }
	const float* getVert(const int idx) const { assert(idx >= 0 && idx < 8); return &m_verts[idx * 3]; }
	void setVert(const int idx, const float* v)
	{
		assert(idx >= 0 && idx < 8);
		vcopy(m_verts + std::ptrdiff_t(idx) * 3, v);
	}
	void setVerts(const float* dat) { std::memcpy(m_verts, dat, sizeof(m_verts)); }

	int getDirsNum() const { return DIRS_SIZE; }
	const float* getDirs() const { return m_dirs; }
	const float* getDir(const int idx) const { assert(idx >= 0 && idx < 3); return &m_dirs[idx * 3]; }
	void setDir(const int idx, const float* dat)
	{
		assert(idx >= 0 && idx < 3);
		vcopy(m_dirs + std::ptrdiff_t(idx) * 3, dat);
	}
	void setDirs(const float* dirs) { std::memcpy(m_dirs, dirs, sizeof(m_dirs)); }

	const float* getCenter() const { return m_center; }
	void setCenter(const float* v) { vcopy(m_center, v); }

	const float* getHalfWidths() const { return m_halfSizes; }
	void setHalfWidths(const float* dat) { std::memcpy(m_halfSizes, dat, sizeof(m_halfSizes)); }
};

template <int PLANES_SIZE_>
struct Obp // oriented bounding polyhedron
{
public:
	static const int PLANES_SIZE = PLANES_SIZE_;
	static const int VERTS_SIZE = PLANES_SIZE_ * 2;

private:
	float m_center[3];
	int m_dirsNum;
	float m_dirs[3 * PLANES_SIZE];
	int m_vertsNum;
	float m_verts[3 * VERTS_SIZE];

public:
	void init(const int dirsNum, const float* dirs, const int vertsNum, const float* verts)
	{
		assert(dirsNum <= PLANES_SIZE);
		assert(vertsNum <= VERTS_SIZE);
		std::memset(m_center, 0, sizeof(m_center));
		m_dirsNum = dirsNum;
		std::memcpy(m_dirs, dirs, sizeof(float) * 3 * dirsNum);
		m_vertsNum = vertsNum;
		std::memcpy(m_verts, verts, sizeof(float) * 3 * vertsNum);
	}

	void init(const int dirsNum, const float* dirs, const int vertsNum, const float* verts, const float* center)
	{
		init(dirs, dirsNum, verts, vertsNum);
		if (center) {
			std::memcpy(m_center, center, sizeof(m_center));
		}
		else {
			for (int i = 0, n = vertsNum * 3; i < n; i += 3) {
				vadd(m_center, verts + i);
			}
			const float d = 1.f / vertsNum;
			vertsNum[0] *= d;
			vertsNum[1] *= d;
			vertsNum[2] *= d;
		}
	}

	void clear()
	{
		std::memset(m_center, 0, sizeof(m_center));
		//std::memset(m_dirs, 0, sizeof(float) * 3 * MAX_PLANES_SIZE);
		m_dirsNum = 0;
		//std::memset(m_verts, 0, sizeof(float) * 3 * MAX_VERTS_SIZE);
		m_vertsNum = 0;
	}

	const float* getCenter() const { return m_center; }
	const float* getDirs() const { return m_dirs; }
	const float* getDir(const int idx) const { assert(idx >= 0 && idx < PLANES_SIZE); return &m_dirs[idx * 3]; }
	const float* getVerts() const { return m_verts; }
	const float* getVert(const int idx) const { assert(idx >= 0 && idx < VERTS_SIZE); return &m_verts[idx * 3]; }
	int getDirsNum() const { return m_dirsNum; }
	int getVertsNum() const { return m_vertsNum; }
};

template <int MAX_POLY_VERTS_SIZE>
bool intersectionObbVsPoly(
	const Obb* obb, const float* polyNorm, const float* polyPoints, const int polyPointsNum
) {
	float minMaxBp[2], minMaxPoly[2];
	float polyEdges[3 * MAX_POLY_VERTS_SIZE];
	for (int i = 0; i < polyPointsNum * 3 - 3; i += 3)
		vsub(polyEdges + i, polyPoints + i, polyPoints + i + 3);
	vsub(polyEdges + polyPointsNum * 3 - 3, polyPoints + polyPointsNum * 3 - 3, polyPoints);

	// perps to faces
	for (int i = 0; i < Obb::DIRS_SIZE; ++i) {
		calcProjection(obb->getVerts(), Obb::VERTS_SIZE, obb->getDir(i), minMaxBp);
		calcProjection(polyPoints, polyPointsNum, obb->getDir(i), minMaxPoly);
		if (minMaxBp[1] < minMaxPoly[0] || minMaxBp[0] > minMaxPoly[1]) return false;
	}

	// poly norm
	calcProjection(obb->getVerts(), Obb::VERTS_SIZE, polyNorm, minMaxBp);
	calcProjection(polyPoints, polyPointsNum, polyNorm, minMaxPoly);
	if (minMaxBp[1] < minMaxPoly[0] || minMaxBp[0] > minMaxPoly[1]) return false;

	// edges
	float cross[3];
	for (int i = 0; i < Obb::DIRS_SIZE; ++i) {
		for (int j = 0, n = polyPointsNum * 3; j < n; j += 3) {
			vcross(cross, polyEdges + j, obb->getDir(i));
			calcProjection(obb->getVerts(), Obb::VERTS_SIZE, cross, minMaxBp);
			calcProjection(polyPoints, polyPointsNum, cross, minMaxPoly);
			if (minMaxBp[1] < minMaxPoly[0] || minMaxBp[0] > minMaxPoly[1]) return false;
		}
	}

	return true;
}

template <typename OBP, int MAX_POLY_VERTS_SIZE>
bool intersectionObpVsPolygon( // y aligned oriented bounding polyhedron
	const OBP* bp, const float* polyNorm, const float* polyPoints, const int polyPointsNum
) {
	float minMaxBp[2], minMaxPoly[2];
	float polyEdges[3 * MAX_POLY_VERTS_SIZE];
	for (int i = 0; i < polyPointsNum * 3 - 3; i += 3)
		vsub(polyEdges + i, polyPoints + i, polyPoints + i + 3);
	vsub(polyEdges + polyPointsNum * 3 - 3, polyPoints + polyPointsNum * 3 - 3, polyPoints);

	// perps to faces
	for (int i = 0, n = bp->getDirsNum(); i < n; ++i) {
		calcProjection(bp->getVerts(), bp->getVertsNum(), bp->getDir(i), minMaxBp);
		calcProjection(polyPoints, polyPointsNum, bp->getDir(i), minMaxPoly);
		if (minMaxBp[1] < minMaxPoly[0] || minMaxBp[0] > minMaxPoly[1]) return false;
	}

	// poly norm
	calcProjection(bp->getVerts(), bp->getVertsNum(), polyNorm, minMaxBp);
	calcProjection(polyPoints, polyPointsNum, polyNorm, minMaxPoly);
	if (minMaxBp[1] < minMaxPoly[0] || minMaxBp[0] > minMaxPoly[1]) return false;

	// edges
	float cross[3];
	for (int i = 0, n = bp->getDirsNum(); i < n; ++i) {
		for (int j = 0, n = polyPointsNum * 3; j < n; j += 3) {
			vcross(cross, polyEdges + j, bp->getDir(i));
			calcProjection(bp->getVerts(), bp->getVertsNum(), cross, minMaxBp);
			calcProjection(polyPoints, polyPointsNum, cross, minMaxPoly);
			if (minMaxBp[1] < minMaxPoly[0] || minMaxBp[0] > minMaxPoly[1]) return false;
		}
	}

	return true;
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
			delete[] m_data;
			m_data = nullptr;
			m_n = 0;
		}

		BminBmax* getData() { return m_data; }
		int getSize() const { return m_n; }

	private:
		BminBmax* m_data = nullptr;
		int m_n = 0;
	};

public:
	BminBmaxSegmentTree() = default;
	~BminBmaxSegmentTree() = default;
	BminBmaxSegmentTree(const BminBmaxSegmentTree&) = delete;
	const BminBmaxSegmentTree& operator=(const BminBmaxSegmentTree&) = delete;

	static const ResourceManager& getResourceManager()
	{
		return ResourceManager::Instance();
	}
	static void freeResources()
	{
		ResourceManager::Instance().free();
	}

	bool calcTree(uint32_t i, uint32_t j, const AabbTri* bboxes, const int* boxIds)
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
			const AabbTri* box = bboxes + boxIds[k];
			vcopy(leafes->min, box->min);
			vcopy(leafes->max, box->max);
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
				vcopy(dat[l].min, dat[l * 2].min);
				vmin(dat[l].min, dat[l * 2 + 1].min);
				vcopy(dat[l].max, dat[l * 2].max);
				vmax(dat[l].max, dat[l * 2 + 1].max);
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
			vmin(min, m_curDat[i].min);
			vmax(max, m_curDat[i].max);
			return;
		}
		while (i < j) {
			if (i % 2 != 0) {
				vmin(min, m_curDat[i].min);
				vmax(max, m_curDat[i].max);
				++i;
			}
			if (j % 2 == 0) {
				vmin(min, m_curDat[j].min);
				vmax(max, m_curDat[j].max);
				--j;
			}
			i >>= 1;
			j >>= 1;
			if (i == j) {
				vmin(min, m_curDat[i].min);
				vmax(max, m_curDat[i].max);
			}
		}
	}

private:
	int m_i = -1;
	int m_j = -1;
	uint32_t m_curSize = 0;
	const BminBmax* m_curDat = nullptr;
};

struct CoordComparator
{
	CoordComparator(const int axisNum_, const AabbTri* boxes_) : axisNum(axisNum_), boxes(boxes_)
	{
		assert(axisNum_ >= 0 && axisNum_ <= 2);
	}

	bool operator() (const int l, const int r) const
	{
		return boxes[l].min[axisNum] < boxes[r].min[axisNum];
	}

	int axisNum;
	const AabbTri* boxes;
};

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

} // namespace geometry

#endif // GEOMETRY_LIB_H