#ifndef GEOMETRY_LIB_H
#define GEOMETRY_LIB_H

#include "Common.h"

#include <cstring>
#include <cmath>
#include <cassert>
#include <algorithm>
#include <type_traits>

#include <immintrin.h>

namespace geometry
{

// constants and PODs
struct Constants
{
	static constexpr float EPS = 1e-6f;
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

inline void vcopy(float* vres, const float* v)
{
	vres[0] = v[0];
	vres[1] = v[1];
	vres[2] = v[2];
}

inline void vsub(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0] - v2[0];
	dest[1] = v1[1] - v2[1];
	dest[2] = v1[2] - v2[2];
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

inline float rcVdotXz(const float* v1, const float* v2)
{
	return v1[0] * v2[0] + v1[2] * v2[2];
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

bool intersectObbTriangle(const struct OBBExt* be, const float* triPoints);

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
	float dir[9]; // 3 vectors
	float center[3];
	float halfWidth[3];
};

struct OBBExt
{
	float verts[3 * 8];
	OBB b;
};

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