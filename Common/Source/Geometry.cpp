
#include "Geometry.h"

namespace geometry
{

bool intersectSegmentTriangle(
	const float* sp, const float* sq, const float* a, const float* b, const float* c, float &t
) {
	float v, w;
    float ab[3], ac[3], qp[3], ap[3], norm[3], e[3];
	vsub(ab, b, a);
	vsub(ac, c, a);
	vsub(qp, sp, sq);
	
	// Compute triangle normal. Can be precalculated or cached if
	// intersecting multiple segments against the same triangle
	vcross(norm, ab, ac);
	
	// Compute denominator d. If d <= 0, segment is parallel to or points
	// away from triangle, so exit early
	float d = vdot(qp, norm);
	if (d /*<=*/== 0.0f) return false;
	float invD = 1.f / d;
	
	// Compute intersection t value of pq with plane of triangle. A ray
	// intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
	// dividing by d until intersection has been found to pierce triangle
	vsub(ap, sp, a);
	t = vdot(ap, norm) * invD;
	if (t < 0.0f) return false;
	if (t > /*d*/1.f) return false; // For segment; exclude this code line for a ray test
	
	// Compute barycentric coordinate components and test if within bounds
	vcross(e, qp, ap);
	v = vdot(ac, e) * invD;
	if (v < 0.0f || v > /*d*/1.f) return false;
	w = -vdot(ab, e) * invD;
	if (w < 0.0f || v + w > /*d*/1.f) return false;
	
	// Segment/ray intersects triangle. Perform delayed division
	//t /= d;
	
	return true;
}

bool isectSegAabb(
	const float* sp, const float* sq, const float* amin,
	const float* amax, float& tmin, float& tmax
) {
	float d[3];
	d[0] = sq[0] - sp[0];
	d[1] = sq[1] - sp[1];
	d[2] = sq[2] - sp[2];
	tmin = 0.0;
	tmax = 1.0f;

	for (int i = 0; i < 3; i++)
	{
		if (std::fabsf(d[i]) < Constants::EPS)
		{
			if (sp[i] < amin[i] || sp[i] > amax[i])
				return false;
		}
		else
		{
			const float ood = 1.0f / d[i];
			float t1 = (amin[i] - sp[i]) * ood;
			float t2 = (amax[i] - sp[i]) * ood;
			if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
			if (t1 > tmin) tmin = t1;
			if (t2 < tmax) tmax = t2;
			if (tmin > tmax) return false;
		}
	}

	return true;
}

void calcProjection(
	const float* points, int n, const float* axis, float(&minMax)[2]
) {
	assert(n >= 1);
	minMax[0] = vdot(points, axis);
	minMax[1] = minMax[0];
	for (int i = 1; i < n; ++i) {
		float val = vdot(points + i * 3, axis);
		minMax[0] = scalarMin(minMax[0], val);
		minMax[1] = scalarMax(minMax[1], val);
	}
}

void calcVerticalVertexProjectionOnPlane(
	const float* v, const float* n, const float d, float* vProj
) {
	float dir[3] = {0.f, -1.f, 0.f};
	float t = (-d - vdot(v, n)) / vdot(dir, n);
	vmad(vProj, v, dir, t);
}

void makeAabbPoints(float* aabbPoints, const float* bmin, const float* bmax)
{
	float dx = bmax[0] - bmin[0];
	float dy = bmax[1] - bmin[1];
	float dz = bmax[2] - bmin[2];
	float* pos = aabbPoints;
	vcopy(pos, bmin); // 0
	pos += 3; //1
	vcopy(pos, bmin);
	pos[2] += dz;
	pos += 3; // 2
	vcopy(pos, bmin);
	pos[0] += dx;
	pos[2] += dz;
	pos += 3; // 3
	vcopy(pos, bmin);
	pos[0] += dx;
	pos += 3; // 4
	vcopy(pos, bmin);
	pos[1] += dy;
	pos += 3; // 5
	vcopy(pos, bmin);
	pos[1] += dy;
	pos[2] += dz;
	pos += 3; // 6
	vcopy(pos, bmin);
	pos[0] += dx;
	pos[1] += dy;
	pos[2] += dz;
	pos += 3; // 7
	vcopy(pos, bmin);
	pos[0] += dx;
	pos[1] += dy;
}

bool intersectionAabbVsTriangle(
	const float* bmin, const float* bmax, const float* v0, const float* v1, const float* v2
) {
	// SAT
	float axis[3];
	float boxEdge0[3] = { 1, 0, 0 };
	float boxEdge1[3] = { 0, 1, 0 };
	float boxEdge2[3] = { 0, 0, 1 };
	float triEdge0[3], triEdge1[3], triEdge2[3];
	vsub(triEdge0, v0, v1);
	vsub(triEdge1, v1, v2);
	vsub(triEdge2, v2, v0);
	static const int AABB_NPOINTS = 8;
	static const int TRI_NPOINTS = 3;
	float triPoints[3 * TRI_NPOINTS];
	vcopy(triPoints, v0);
	vcopy(triPoints + 3, v1);
	vcopy(triPoints + 6, v2);
	float aabbPoints[3 * AABB_NPOINTS];
	makeAabbPoints(aabbPoints, bmin, bmax);
	float minMaxAabb[2];
	float minMaxTri[2];

	// 9 crosses of aabb edges - tri edges
	float boxEdges[3 * 3];
	vcopy(boxEdges, boxEdge0);
	vcopy(boxEdges + 3, boxEdge1);
	vcopy(boxEdges + 6, boxEdge2);
	float triEdges[3 * 3];
	vcopy(triEdges, triEdge0);
	vcopy(triEdges + 3, triEdge1);
	vcopy(triEdges + 6, triEdge2);
	for (int j = 0; j < 9; j += 3) {
		for (int i = 0; i < 9; i += 3) {
			vcross(axis, boxEdges + i, triEdges + j);
			calcProjection(aabbPoints, 8, axis, minMaxAabb);
			calcProjection(triPoints, 3, axis, minMaxTri);
			if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;
		}
	}

	// 3 box edges
	calcProjection(aabbPoints, 8, boxEdge0, minMaxAabb);
	calcProjection(triPoints, 3, boxEdge0, minMaxTri);
	if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;
	calcProjection(aabbPoints, 8, boxEdge1, minMaxAabb);
	calcProjection(triPoints, 3, boxEdge1, minMaxTri);
	if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;
	calcProjection(aabbPoints, 8, boxEdge2, minMaxAabb);
	calcProjection(triPoints, 3, boxEdge2, minMaxTri);
	if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;

	// tri normal
	vcross(axis, triEdge0, triEdge1);
	calcProjection(aabbPoints, 8, axis, minMaxAabb);
	calcProjection(triPoints, 3, axis, minMaxTri);
	if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;

	return true;
}

bool intersectionObbVsTriangle(const OBBExt* be, const float* triPoints)
{
	float minMaxObb[2], minMaxTri[2];
	float cross[3];
	float e[3 * 3];
	vsub(e + 0, triPoints + 3, triPoints);
	vsub(e + 3, triPoints + 6, triPoints + 3);
	vsub(e + 6, triPoints, triPoints + 6);

	// edges
	for (int i = 0; i < 3; ++i) {
		vcross(cross, e + i * 3, be->getDir(0));
		calcProjection(be->getVerts(), 8, cross, minMaxObb);
		calcProjection(triPoints, 3, cross, minMaxTri);
		if (minMaxObb[1] < minMaxTri[0] || minMaxObb[0] > minMaxTri[1]) return false;
		vcross(cross, e + i * 3, be->getDir(1));
		calcProjection(be->getVerts(), 8, cross, minMaxObb);
		calcProjection(triPoints, 3, cross, minMaxTri);
		if (minMaxObb[1] < minMaxTri[0] || minMaxObb[0] > minMaxTri[1]) return false;
		vcross(cross, e + i * 3, be->getDir(2));
		calcProjection(be->getVerts(), 8, cross, minMaxObb);
		calcProjection(triPoints, 3, cross, minMaxTri);
		if (minMaxObb[1] < minMaxTri[0] || minMaxObb[0] > minMaxTri[1]) return false;
	}
	// obb faces
	calcProjection(be->getVerts(), 8, be->getDir(0), minMaxObb);
	calcProjection(triPoints, 3, be->getDir(0), minMaxTri);
	if (minMaxObb[1] < minMaxTri[0] || minMaxObb[0] > minMaxTri[1]) return false;
	calcProjection(be->getVerts(), 8, be->getDir(1), minMaxObb);
	calcProjection(triPoints, 3, be->getDir(1), minMaxTri);
	if (minMaxObb[1] < minMaxTri[0] || minMaxObb[0] > minMaxTri[1]) return false;
	calcProjection(be->getVerts(), 8, be->getDir(2), minMaxObb);
	calcProjection(triPoints, 3, be->getDir(2), minMaxTri);
	if (minMaxObb[1] < minMaxTri[0] || minMaxObb[0] > minMaxTri[1]) return false;
	// tri
	vcross(cross, e + 0, e + 3);
	calcProjection(be->getVerts(), 8, cross, minMaxObb);
	calcProjection(triPoints, 3, cross, minMaxTri);
	if (minMaxObb[1] < minMaxTri[0] || minMaxObb[0] > minMaxTri[1]) return false;

	return true;
}

bool intersectionSegmentVsPlane(
	const float* n, const float d, const float* A, const float* B, float* P
) {
	float AB[3];
	vsub(AB, B, A);
	const float nabDot = vdot(n, AB);
	if (std::abs(nabDot) < Constants::EPS) {
		return false;
	}
	const float t = (d - vdot(n, A)) / nabDot;
	if (t <= 0.f || t >= 1.f) // except contact of segment and plane
		return false;
	vmad(P, A, AB, t);
	return true;
}

// not tested
static bool intersectionSegmentVsPlane(
	const float* v0, const float* v1, const float* v2, const float* A, const float* B, float* P
) {
	float e1[3], e2[3], n[3];
	vsub(e1, v1, v0);
	vsub(e2, v2, v0);
	vcross(n, e1, e2);
	float d = vdot(n, v0);
	if (d < 0) {
		d *= -1.f;
		n[0] *= -1.f;
		n[1] *= -1.f;
		n[2] *= -1.f;
	}
	return intersectionSegmentVsPlane(n, d, A, B, P);
}

void calcProjectionXz(
	const float* points, int n, const float* axis, float(&minMax)[2]
) {
	assert(n >= 1);
	minMax[0] = vdotXz(points, axis);
	minMax[1] = minMax[0];
	for (int i = 1; i < n; ++i) {
		float val = vdotXz(points + i * 3, axis);
		minMax[0] = scalarMin(minMax[0], val);
		minMax[1] = scalarMax(minMax[1], val);
	}
}

void makeAabbPointsXz(float* aabbPoints, const float* bmin, const float* bmax)
{
	float dx = bmax[0] - bmin[0];
	float dz = bmax[2] - bmin[2];
	float* pos = aabbPoints;
	vcopy(pos, bmin); // 0
	pos += 3; //1
	vcopy(pos, bmin);
	pos[0] += dx;
	pos += 3; // 2
	vcopy(pos, bmin);
	pos[0] += dx;
	pos[2] += dz;
	pos += 3; // 3
	vcopy(pos, bmin);
	pos[2] += dz;
}

// edge connection minds no collision ( <=, >=)
bool intersectionAabbVsTriangleXz(
	const float* bmin, const float* bmax, const float* v0, const float* v1, const float* v2
) {
	// SAT
	static const int AABB_NPOINTS = 4;
	static const int TRI_NPOINTS = 3;
	float triPoints[3 * TRI_NPOINTS];
	vcopy(triPoints, v0);
	vcopy(triPoints + 3, v1);
	vcopy(triPoints + 6, v2);
	float aabbPoints[3 * AABB_NPOINTS];
	makeAabbPointsXz(aabbPoints, bmin, bmax);
	float minMaxAabb[2];
	float minMaxTri[2];

	// 3 tri edge's normals
	float triEdge0[3], triEdge1[3], triEdge2[3];
	vsub(triEdge0, v0, v1);
	vsub(triEdge1, v1, v2);
	vsub(triEdge2, v2, v0);
	float axis[3] = { 0, 0, 0 };
	axis[0] = -triEdge0[2];
	axis[2] = triEdge0[0];
	calcProjectionXz(aabbPoints, AABB_NPOINTS, axis, minMaxAabb);
	calcProjectionXz(triPoints, TRI_NPOINTS, axis, minMaxTri);
	if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;
	axis[0] = -triEdge1[2];
	axis[2] = triEdge1[0];
	calcProjectionXz(aabbPoints, AABB_NPOINTS, axis, minMaxAabb);
	calcProjectionXz(triPoints, TRI_NPOINTS, axis, minMaxTri);
	if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;
	axis[0] = -triEdge2[2];
	axis[2] = triEdge2[0];
	calcProjectionXz(aabbPoints, AABB_NPOINTS, axis, minMaxAabb);
	calcProjectionXz(triPoints, TRI_NPOINTS, axis, minMaxTri);
	if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;

	// 2 axises (x, z)
	float xAxis[3] = { 1, 0, 0 };
	calcProjectionXz(aabbPoints, AABB_NPOINTS, xAxis, minMaxAabb);
	calcProjectionXz(triPoints, TRI_NPOINTS, xAxis, minMaxTri);
	if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;
	float zAxis[3] = { 0, 0, 1 };
	calcProjectionXz(aabbPoints, AABB_NPOINTS, zAxis, minMaxAabb);
	calcProjectionXz(triPoints, TRI_NPOINTS, zAxis, minMaxTri);
	if (minMaxAabb[1] < minMaxTri[0] || minMaxAabb[0] > minMaxTri[1]) return false;

	return true;
}

bool isPointInAabbXz(const float* p, const float* bmin, const float* bmax)
{
	return p[0] > bmin[0] && p[0] < bmax[0] && p[2] > bmin[2] && p[2] < bmax[2];
}

void calcIsectTriArgs(IsectTriArgs& args, const float* start, const float* end)
{
#ifdef USAGE_SSE_1_0
	args.start = _mm_setr_ps(start[0], start[1], start[2], 0.f);
	args.diff = _mm_sub_ps(args.start, _mm_setr_ps(end[0], end[1], end[2], 0.f));
#else
	args.start = start;
	rcVsub(args.diff, start, end);
#endif
}

bool intersectSegmentTriangleRed(
	const IsectTriArgs& args,
	const float* a,
	const float* b,
	const float* c,
	float& t
) {
#ifdef USAGE_SSE_1_0
	float v, w;
	__m128 ma = _mm_load_ps(a);
	__m128 mb = _mm_load_ps(b);
	__m128 mc = _mm_load_ps(c);
	__m128 ab = _mm_sub_ps(mb, ma);
	__m128 ac = _mm_sub_ps(mc, ma);

	// Compute triangle normal. Can be precalculated or cached if
	// intersecting multiple segments against the same triangle
	__m128 norm = sseVcross(ab, ac);

	// Compute denominator d. If d <= 0, segment is parallel to or points
	// away from triangle, so exit early
	float d = sseVdot(args.diff, norm);
	if (std::abs(d) < Constants::EPS) return false;
	float invd = 1.f / d;

	// Compute intersection t value of pq with plane of triangle. A ray
	// intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
	// dividing by d until intersection has been found to pierce triangle
	__m128 ap = _mm_sub_ps(args.start, ma);
	t = sseVdot(ap, norm) * invd;

	// Compute barycentric coordinate components and test if within bounds
	__m128 e = sseVcross(args.diff, ap);

	v = sseVdot(ac, e) * invd;
	w = -sseVdot(ab, e) * invd;
	if (
		t < 0.0f || t > 1.f || v < 0.0f || v > 1.f || w < 0.0f || v + w > 1.f
		) return false;

	return true;
}
#else
	float v, w;
	float ab[3], ac[3], ap[3], norm[3], e[3];
	vsub(ab, b, a);
	vsub(ac, c, a);

	// Compute triangle normal. Can be precalculated or cached if
	// intersecting multiple segments against the same triangle
	vcross(norm, ab, ac);

	// Compute denominator d. If d <= 0, segment is parallel to or points
	// away from triangle, so exit early
	float d = vdot(args.diff, norm);
	if (std::abs(d) < Constants::EPS) return false;
	float invd = 1.f / d;

	// Compute intersection t value of pq with plane of triangle. A ray
	// intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
	// dividing by d until intersection has been found to pierce triangle
	vsub(ap, args.start, a);
	t = vdot(ap, norm) * invd;
	if (t < 0.0f) return false;
	if (t > 1.f) return false; // For segment; exclude this code line for a ray test

	// Compute barycentric coordinate components and test if within bounds
	vcross(e, args.diff, ap);
	v = vdot(ac, e) * invd;
	if (v < 0.0f || v > 1.f) return false;
	w = -vdot(ab, e) * invd;
	if (w < 0.0f || v + w > 1.f) return false;

	return true;
}
#endif

void calcIsectAabbArgs(IsectAabbArgs& args, const float* start, const float* end)
{
	args.sp = start;
	args.d[0] = end[0] - start[0];
	args.d[1] = end[1] - start[1];
	args.d[2] = end[2] - start[2];
	args.ood[0] = 1.f / args.d[0];
	args.ood[1] = 1.f / args.d[1];
	args.ood[2] = 1.f / args.d[2];
#ifdef USAGE_SSE_1_0
	args.start = _mm_setr_ps(args.sp[0], args.sp[1], args.sp[2], 0.f);
	args.mul = _mm_setr_ps(args.ood[0], args.ood[1], args.ood[2], 0.f);
#endif
}

// TODO check fp robustness
bool isectSegXzAabbRed(
	const IsectAabbArgs& args, const float* amin, const float* amax
) {
#ifdef USAGE_SSE_1_0
	__m128 min = _mm_setr_ps(amin[0], 0.f, amin[2], 0.f);
	__m128 max = _mm_setr_ps(amax[0], 0.f, amax[2], 0.f);
	min = _mm_mul_ps(_mm_sub_ps(min, args.start), args.mul);
	max = _mm_mul_ps(_mm_sub_ps(max, args.start), args.mul);
	__m128 min1 = _mm_min_ps(min, max);
	__m128 max1 = _mm_max_ps(min, max);
	min1 = _mm_max_ps(min1, _mm_set1_ps(0.f));
	max1 = _mm_min_ps(max1, _mm_set1_ps(1.f));

	__m128 min2 = _mm_shuffle_ps(min1, min1, _MM_SHUFFLE(0, 1, 3, 2));
	__m128 max2 = _mm_shuffle_ps(max1, max1, _MM_SHUFFLE(0, 1, 3, 2));
	min = _mm_max_ss(min1, min2);
	max = _mm_min_ss(max1, max2);

	return _mm_comige_ss(max, min);
#else
	float ttmin = 0.f;
	float ttmax = 1.f;

	float tx1 = (amin[0] - args.sp[0]) * args.ood[0];
	float tx2 = (amax[0] - args.sp[0]) * args.ood[0];
	ttmin = std::max(ttmin, std::min(tx1, tx2));
	ttmax = std::min(ttmax, std::max(tx1, tx2));

	float tz1 = (amin[2] - args.sp[2]) * args.ood[2];
	float tz2 = (amax[2] - args.sp[2]) * args.ood[2];
	ttmin = std::max(ttmin, std::min(tz1, tz2));
	ttmax = std::min(ttmax, std::max(tz1, tz2));

	return ttmax >= ttmin;
#endif
}

// TODO check fp robustness
bool isectSegAabbRed(IsectAabbArgs& args, const float* amin, const float* amax)
{
#ifdef USAGE_SSE_1_0
	__m128 min = _mm_load_ps(amin);
	__m128 max = _mm_load_ps(amax);
	min = _mm_mul_ps(_mm_sub_ps(min, args.start), args.mul);
	max = _mm_mul_ps(_mm_sub_ps(max, args.start), args.mul);
	__m128 min1 = _mm_min_ps(min, max);
	__m128 max1 = _mm_max_ps(min, max);
	min1 = _mm_max_ps(min1, _mm_set1_ps(0.f));
	max1 = _mm_min_ps(max1, _mm_set1_ps(1.f));

	__m128 min2 = _mm_shuffle_ps(min1, min1, _MM_SHUFFLE(0, 3, 2, 1));
	__m128 max2 = _mm_shuffle_ps(max1, max1, _MM_SHUFFLE(0, 3, 2, 1));
	min = _mm_max_ss(min1, min2);
	max = _mm_min_ss(max1, max2);
	min2 = _mm_movehl_ps(min, min);
	max2 = _mm_movehl_ps(max, max);
	min = _mm_max_ss(min, min2);
	max = _mm_min_ss(max, max2);

	return _mm_comige_ss(max, min);
#else
	float tmin = 0.f;
	float tmax = 1.f;

	float tx1 = (amin[0] - args.sp[0]) * args.ood[0];
	float tx2 = (amax[0] - args.sp[0]) * args.ood[0];
	tmin = std::max(tmin, std::min(tx1, tx2));
	tmax = std::min(tmax, std::max(tx1, tx2));

	float ty1 = (amin[1] - args.sp[1]) * args.ood[1];
	float ty2 = (amax[1] - args.sp[1]) * args.ood[1];
	tmin = std::max(tmin, std::min(ty1, ty2));
	tmax = std::min(tmax, std::max(ty1, ty2));

	float tz1 = (amin[2] - args.sp[2]) * args.ood[2];
	float tz2 = (amax[2] - args.sp[2]) * args.ood[2];
	tmin = std::max(tmin, std::min(tz1, tz2));
	tmax = std::min(tmax, std::max(tz1, tz2));

	return tmax >= tmin;
#endif
}

bool checkPolyVsPolyXz(
	const float* first, int firstNum, const float* second, int secondNum
) {
	float e[3], norm[3];
	float minMaxFirst[2], minMaxSecond[2];

	// SAT
	for (int i = firstNum - 1, j = 0; i != firstNum - 1; i = j, ++j) {
		vsub(e, first + i * 3, first + j * 3);
		norm[0] = e[2];
		norm[1] = 0.f;
		norm[2] = -e[0];
		calcProjectionXz(first, firstNum, norm, minMaxFirst);
		calcProjectionXz(second, secondNum, norm, minMaxSecond);
		if (minMaxFirst[1] < minMaxSecond[0] || minMaxSecond[1] < minMaxFirst[0]) {
			return false;
		}
	}
	for (int i = secondNum - 1, j = 0; i != secondNum - 1; i = j, ++j) {
		vsub(e, second + i * 3, second + j * 3);
		norm[0] = e[2];
		norm[1] = 0.f;
		norm[2] = -e[0];
		calcProjectionXz(first, firstNum, norm, minMaxFirst);
		calcProjectionXz(second, secondNum, norm, minMaxSecond);
		if (minMaxFirst[1] < minMaxSecond[0] || minMaxSecond[1] < minMaxFirst[0]) {
			return false;
		}
	}

	return true;
}

bool calcDirOutOfPolyXz(const float* v1, const float* v2, const float* inPoly, float* dir)
{
	static const float EPS = 1e-4;
	float e[3], p[3];
	float toInPoly[3], toPoint[3];

	vsub(e, v2, v1);
	if (vlen(e) < EPS) {
		return false;
	}
	vcopy(dir, e);
	dir[1] = 0.f;
	float tmp = dir[0];
	dir[0] = dir[2];
	dir[2] = -tmp;

	// perp product (XZ plane) test to check direction of dir
	vsub(toInPoly, inPoly, v1);
	vadd(p, v2, dir);
	vsub(toPoint, p, v1);
	tmp = e[0] * toInPoly[2] - toInPoly[0] * e[2];
	float tmp1 = e[0] * toPoint[2] - toPoint[0] * e[2];
	if (tmp * tmp1 > 0)
	{
		dir[0] *= -1.f;
		dir[2] *= -1.f;
	}

	vnormalize(dir);
	return true;
}

// dirs consists 9 floats, points - 24 floats
void calcObbDirsAndPoints(
	const float* v1,
	const float* v2,
	const float* fwdDirNorm,
	const float fwdDst,
	const float height,
	float* dirs,
	float* points
) {
	vsub(dirs, v2, v1);
	const float dstSide = vlen(dirs);
	vnormalize(dirs);
	vcopy(dirs + 3, fwdDirNorm);
	dirs[6] = 0.f;
	dirs[7] = 1.f;
	dirs[8] = 0.f;

	const float* dirSide = dirs;
	const float* dirFwd = dirs + 3;
	const float* dirUp = dirs + 6;
	vcopy(points, v1);
	vcopy(points + 3, v2);
	vmad(points + 6, v2, dirFwd, fwdDst);
	vmad(points + 9, v1, dirFwd, fwdDst);
	float* v5 = points + 12;
	float* v6 = points + 15;
	vmad(v5, v1, dirUp, height);
	//vmad(points + 15, v5, dirSide, dstSide);
	vmad(v6, v2, dirUp, height);
	//vmad(points + 18, v5, dirFwd, fwdDst);
	//vmad(points + 21, points + 15, dirFwd, fwdDst);
	vmad(points + 18, v6, dirFwd, fwdDst);
	vmad(points + 21, v5, dirFwd, fwdDst);
}

void calcCenterAndHalfExtents(const float* verts, const int vertsSize, float* center, float* halfExtents)
{
	float min[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
	float max[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};

	const float* p = verts;
	const float* pe = verts + vertsSize * 3;
	for (; p < pe; p += 3)
	{
		vmin(min, p);
		vmax(max, p);
	}
	vsub(halfExtents, max, min);
	vadd(center, min, max);
	vmul(center, 0.5f);
	vmul(halfExtents, 0.5f);
}

void calcAabb(const float* verts, const int vertsSize, float* min, float* max)
{
	min[0] = min[1] = min[2] = FLT_MAX;
	max[0] = max[1] = max[2] = -FLT_MAX;

	const float* p = verts;
	const float* pe = verts + vertsSize * 3;
	for (; p < pe; p += 3)
	{
		vmin(min, p);
		vmax(max, p);
	}
}

} // namespace geometry