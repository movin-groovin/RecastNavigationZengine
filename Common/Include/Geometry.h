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

bool intersectionObbVsTriangle(const struct OBBExt* be, const float* triPoints);

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

bool intersectSegmentTriangleRed(
	const struct IsectTriArgs& args,
	const float* a,
	const float* b,
	const float* c,
	float& t
);

void calcIsectAabbArgs(struct IsectAabbArgs& args, const float* start, const float* end);

bool isectSegXzAabbRed(
	const struct IsectAabbArgs& args, const float* amin, const float* amax
);

bool isectSegAabbRed(struct IsectAabbArgs& args, const float* amin, const float* amax);

bool checkPolyVsPolyXz(
	const float* first, int firstNum, const float* second, int secondNum
);

// structs and classes
struct Plane
{
	Plane() : norm{ 0.f, 0.f, 0.f }, dist(0.f) {}
	Plane(const float* n, float d) : dist(d) { std::memcpy(norm, n, 3 * sizeof(float)); }
	Plane(float nx, float ny, float nz, float d) : norm{ nx, ny, nz }, dist(d) {}

	float norm[3];
	float dist;
};

struct Aabb3D
{
	float min[3];
	float max[3];
	int polyIndex;
};

struct OBB
{
public:
	static const int DIRS_NUM = 3;

protected:
	float dir[3 * DIRS_NUM]; // 3 vectors
	float center[3];
	float _halfWidth[3];

public:
	void init(
		const float* src,
		const float* dst,
		const float* sideDir,
		const float halfWidth,
		const float deltaH,
		const float scale
	) {
		vsub(dir, dst, src);
		vmul(dir, 0.5f);
		float* dir_3 = dir + 3;
		if (sideDir) {
			vcopy(dir_3, sideDir);
		}
		else {
			dir_3[0] = dir[2];
			dir_3[1] = 0.f;
			dir_3[2] = -dir[0];
		}
		vnormalize(dir_3);
		vmul(dir_3, halfWidth);
		float* dir_6 = dir + 6;
		dir_6[0] = 0.f;
		dir_6[1] = deltaH;
		dir_6[2] = 0.f;
		vadd(center, src, dir);
		vadd(center, dir_6);
		// shrink relatively center
		vmul(dir, scale); // 0.95 ?
		vmul(dir_3, scale); // 0.95 ?
		vmul(dir_6, scale); // 0.95 ?
		_halfWidth[0] = vlen(dir);
		_halfWidth[1] = vlen(dir_3);
		_halfWidth[2] = vlen(dir_6); //deltaH;
	}

	void clear()
	{
		std::memset(dir, 0, sizeof(dir));
		std::memset(center, 0, sizeof(center));
		std::memset(_halfWidth, 0, sizeof(_halfWidth));
	}

	void copy(OBB& to) const
	{
		std::memcpy(to.dir, dir, sizeof(dir));
		std::memcpy(to.center, center, sizeof(center));
		std::memcpy(to._halfWidth, _halfWidth, sizeof(_halfWidth));
	}

	const float* getDirs() const { return dir; }
	const float* getDir(const int idx) const { assert(idx >= 0 && idx < 3); return &dir[idx * 3]; }
	void setDir(const int idx, const float* dat)
	{
		assert(idx >= 0 && idx < 3);
		vcopy(dir + std::ptrdiff_t(idx) * 3, dat);
	}
	const float* getCenter() const { return center; }
	void setCenter(const float* v) { vcopy(center, v); }
	const float* getHalfWidth() const { return _halfWidth; }
	void setHalfWidth(const float* dat) { std::memcpy(_halfWidth, dat, sizeof(_halfWidth)); }
};

struct OBBExt: private OBB
{
public:
	static const int VERTS_NUM = 8;

private:
	float m_verts[3 * VERTS_NUM];

public:
	void init(
		const float* src,
		const float* dst,
		const float halfWidth,
		const float deltaH,
		const float scale
	) {
		OBB::init(src, dst, nullptr, halfWidth, deltaH, scale);
		calcObbPoints();
	}

	void init(
		const float* src,
		const float* dst,
		const float* sideDir,
		const float halfWidth,
		const float deltaH,
		const float scale
	) {
		OBB::init(src, dst, sideDir, halfWidth, deltaH, scale);
		calcObbPoints();
	}

	void init(const float* dirs, const float* verts)
	{
		float stub[] = {FLT_MAX, FLT_MAX, FLT_MAX};
		setCenter(stub);
		setHalfWidth(stub);
		setDir(0, dirs);
		setDir(1, dirs + 3);
		setDir(2, dirs + 6);
		std::memcpy(m_verts, verts, sizeof(float) * 3 * VERTS_NUM);
	}

	void clear()
	{
		OBB::clear();
		std::memset(m_verts, 0, sizeof(m_verts));
	}

	void copy(OBBExt& to) const
	{
		OBB::copy(to);
		std::memcpy(to.m_verts, m_verts, sizeof(m_verts));
	}

	const float* getVerts() const { return m_verts; }
	const float* getVert(const int idx) const { assert(idx >= 0 && idx < 8); return &m_verts[idx * 3]; }
	void setVert(const int idx, const float* v)
	{
		assert(idx >= 0 && idx < 8);
		vcopy(m_verts + std::ptrdiff_t(idx) * 3, v);
	}
	void setVerts(const float* dat) { std::memcpy(m_verts, dat, sizeof(m_verts)); }

	using OBB::DIRS_NUM;
	using OBB::getDir;
	using OBB::getDirs;
	using OBB::getCenter;
	using OBB::getHalfWidth;
	using OBB::setCenter;
	using OBB::setDir;
	using OBB::setHalfWidth;

private:
	void calcObbPoints()
	{
		// directions: forward, side, up
		const float* dirFwd = dir;
		const float* dirSide = dir + 3;
		const float* dirUp = dir + 6;
		vadd(m_verts, center, dirSide);
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

		//// forward, left, up
		//vadd(m_verts, center, dir);
		//vadd(m_verts, dir + 3);
		//vadd(m_verts, dir + 6);

		//vadd(m_verts + 3, center, dir);
		//vadd(m_verts + 3, dir + 3);
		//vsub(m_verts + 3, dir + 6);

		//vadd(m_verts + 6, center, dir);
		//vsub(m_verts + 6, dir + 3);
		//vadd(m_verts + 6, dir + 6);

		//vadd(m_verts + 9, center, dir);
		//vsub(m_verts + 9, dir + 3);
		//vsub(m_verts + 9, dir + 6);

		//vsub(m_verts + 12, center, dir);
		//vadd(m_verts + 12, dir + 3);
		//vadd(m_verts + 12, dir + 6);

		//vsub(m_verts + 15, center, dir);
		//vadd(m_verts + 15, dir + 3);
		//vsub(m_verts + 15, dir + 6);

		//vsub(m_verts + 18, center, dir);
		//vsub(m_verts + 18, dir + 3);
		//vadd(m_verts + 18, dir + 6);

		//vsub(m_verts + 21, center, dir);
		//vsub(m_verts + 21, dir + 3);
		//vsub(m_verts + 21, dir + 6);
	}
};

template <int MAX_PLANES_NUM>
struct YAlignedObp // oriented bounding polyhedron
{
private:
	static const int MAX_VERTS_NUM = MAX_PLANES_NUM * 2;

public:
	static const int MAX_PLANES_NUM_VAL = MAX_PLANES_NUM;
	static const int MAX_VERTS_NUM_VAL = MAX_VERTS_NUM;

public:
	void init(const float* dirs, const int dirsNum, const float* verts, const int vertsNum)
	{
		m_dirsNum = dirsNum;
		std::memcpy(m_dirs, dirs, sizeof(float) * 3 * MAX_PLANES_NUM);
		m_vertsNum = vertsNum;
		std::memcpy(m_verts, verts, sizeof(float) * 3 * MAX_VERTS_NUM);
	}

	void clear()
	{
		std::memset(m_dirs, 0, sizeof(float) * 3 * MAX_PLANES_NUM);
		std::memset(m_verts, 0, sizeof(float) * 3 * MAX_VERTS_NUM);
	}

	const float* getDir() const { return m_dirs; }
	const float* getDir(const int idx) const { assert(idx >= 0 && idx < MAX_PLANES_NUM); return &m_dirs[idx * 3]; }
	const float* getVerts() const { return m_verts; }
	const float* getVert(const int idx) const { assert(idx >= 0 && idx < MAX_VERTS_NUM); return &m_verts[idx * 3]; }
	constexpr int getDirsNum() const { return m_dirsNum; }
	constexpr int getVertsNum() const { return m_vertsNum; }

private:
	int m_dirsNum;
	float m_dirs[3 * MAX_PLANES_NUM]; // last dir is y parallel, from less y to higher y
	int m_vertsNum;
	float m_verts[3 * MAX_VERTS_NUM];
};

template <int MAX_POLY_VERTS_NUM>
bool intersectionObbVsPoly(
	const OBBExt* be, const float* polyNorm, const float* polyPoints, const int polyPointsNum
) {
	float minMaxBp[2], minMaxPoly[2];
	float polyEdges[3 * MAX_POLY_VERTS_NUM];
	for (int i = 0; i < polyPointsNum * 3 - 3; i += 3)
		vsub(polyEdges + i, polyPoints + i, polyPoints + i + 3);
	vsub(polyEdges + polyPointsNum * 3 - 3, polyPoints + polyPointsNum * 3 - 3, polyPoints);

	// perps to faces
	for (int i = 0, n = OBBExt::DIRS_NUM; i < n; ++i) {
		calcProjection(be->getVerts(), OBBExt::VERTS_NUM, be->getDir(i), minMaxBp);
		calcProjection(polyPoints, polyPointsNum, be->getDir(i), minMaxPoly);
		if (minMaxBp[1] < minMaxPoly[0] || minMaxBp[0] > minMaxPoly[1]) return false;
	}

	// poly norm
	calcProjection(be->getVerts(), OBBExt::VERTS_NUM, polyNorm, minMaxBp);
	calcProjection(polyPoints, polyPointsNum, polyNorm, minMaxPoly);
	if (minMaxBp[1] < minMaxPoly[0] || minMaxBp[0] > minMaxPoly[1]) return false;

	// edges
	float cross[3];
	for (int i = 0, n = OBBExt::DIRS_NUM; i < n; ++i) {
		for (int j = 0, n = polyPointsNum * 3; j < n; j += 3) {
			vcross(cross, polyEdges + j, be->getDir(i));
			calcProjection(be->getVerts(), OBBExt::VERTS_NUM, cross, minMaxBp);
			calcProjection(polyPoints, polyPointsNum, cross, minMaxPoly);
			if (minMaxBp[1] < minMaxPoly[0] || minMaxBp[0] > minMaxPoly[1]) return false;
		}
	}

	return true;
}

template <typename OBP, int MAX_POLY_VERTS_NUM>
bool intersectionYaobpVsPolygon( // y aligned oriented bounding polyhedron
	const OBP* bp, const float* polyNorm, const float* polyPoints, const int polyPointsNum
) {
	float minMaxBp[2], minMaxPoly[2];
	float polyEdges[3 * MAX_POLY_VERTS_NUM];
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

void calcCenterAndHalfExtents(const float* verts, const int vertsSize, float* center, float* halfExtents);

void calcAabb(const float* verts, const int vertsSize, float* min, float* max);

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

struct CoordComparer
{
	CoordComparer(int axisNum_, const Aabb3D* boxes_) : axisNum(axisNum_), boxes(boxes_)
	{
		assert(axisNum_ >= 0 && axisNum_ <= 2);
	}

	bool operator() (const int l, const int r) const
	{
		return boxes[l].min[axisNum] < boxes[r].min[axisNum];
	}

	int axisNum;
	const Aabb3D* boxes;
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