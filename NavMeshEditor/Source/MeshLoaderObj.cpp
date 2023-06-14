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

#include "MeshLoaderObj.h"

#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <cstdint>
#include <tuple>
#include <string>
#include <sstream>
#include <cassert>
#include <limits>
#include <vector>
#include <utility>
#include <unordered_map>
#include <functional>
#include <new>
#define _USE_MATH_DEFINES
#include <cmath>

static char* parseRow(char* buf, char* bufEnd, char* row, int len)
{
	bool start = true;
	bool done = false;
	int n = 0;
	while (!done && buf < bufEnd)
	{
		char c = *buf;
		buf++;
		// multirow
		switch (c)
		{
			case '\\':
				break;
			case '\n':
				if (start) break;
				done = true;
				break;
			case '\r':
				break;
			case '\t':
			case ' ':
				if (start) break;
				// else falls through
			default:
				start = false;
				row[n++] = c;
				if (n >= len-1)
					done = true;
				break;
		}
	}
	row[n] = '\0';
	if (n > 0 && row[n - 1] == ' ')
		row[n - 1] = '\0';
	return buf;
}

static int parseFace(char* row, int* data, int n, int vcnt, int* area)
{
	int j = 0;
	while (*row != '\0')
	{
		// Skip initial white space
		while (*row != '\0' && (*row == ' ' || *row == '\t'))
			row++;
		char* s = row;
		// Find vertex delimiter and terminated the string there for conversion.
		while (*row != '\0' && *row != ' ' && *row != '\t')
		{
			if (*row == '/') *row = '\0';
			row++;
		}
		if (*s == '\0')
			continue;
        if (!strncmp(s, "flg ", 4))
        {
            s += 4;
            *area = atoi(s);
            return j;
        }
		int vi = atoi(s);
		data[j++] = vi < 0 ? vi+vcnt : vi-1;
        if (j >= n)
            return j;
	}

	return j;
}

std::pair<std::unique_ptr<char[]>, int> MeshLoaderObjExt::loadFile(const char* filename)
{
	char* buf = 0;
	FILE* fp = fopen(filename, "rb");
	if (!fp)
		return {};
	if (fseek(fp, 0, SEEK_END) != 0)
	{
		fclose(fp);
		return {};
	}
	long bufSize = ftell(fp);
	if (bufSize < 0)
	{
		fclose(fp);
		return {};
	}
	if (fseek(fp, 0, SEEK_SET) != 0)
	{
		fclose(fp);
		return {};
	}
    buf = new(std::nothrow) char[bufSize];
	if (!buf)
	{
		fclose(fp);
		return {};
	}
	size_t readLen = fread(buf, bufSize, 1, fp);
	fclose(fp);

	if (readLen != 1)
	{
		delete[] buf;
		return {};
	}
	return {std::unique_ptr<char[]>(buf), bufSize};
}

int MeshLoaderObjExt::saveFile(
	const char* fileName, const std::unique_ptr<char[]>& data, int dataSize
) {
	char name[common::Constants::STR_SIZE + 4];
	std::strcpy(name, fileName);
	name[common::Constants::STR_SIZE - 1] = '\0';
	std::strcpy(name + strlen(fileName), ".tmp");
	FILE* fp = fopen(name, "wb");
	if (!fp) {
		return 1;
	}
	int ret = static_cast<int>(fwrite(data.get(), sizeof(char), dataSize, fp));
	if (ret != dataSize) {
        fclose(fp);
		return 2;
	}
	fclose(fp);
	remove(fileName);
	if (rename(name, fileName)) {
        return 3;
	}
	return 0;
}

uint8_t MeshLoaderObjExt::addVertex(
	std::unique_ptr<float[]>& verts, int& vertCount, float x, float y, float z, int& cap
) {
	if (vertCount + 1 > cap)
	{
		cap = !cap ? 8 : cap * 2;
        float* nv = new(std::nothrow) float[cap * 3];
		if (!nv)
			return 1;
		if (vertCount)
			std::memcpy(nv, verts.get(), vertCount * 3 * sizeof(float));
		verts.reset(nv);
	}
	float* dst = &verts[vertCount * 3];
	*dst++ = x;
	*dst++ = y;
	*dst++ = z;
	vertCount++;
	return 0;
}

uint8_t MeshLoaderObjExt::addTriangle(
	std::unique_ptr<int[]>& tris, std::unique_ptr<mesh::FlagType[]>& flags,
	int& triCount, int a, int b, int c, int& cap, bool isTri, int area
) {
	if (triCount + 1 > cap)
	{
		cap = !cap ? 8 : cap * 2;
        int* nv = new(std::nothrow) int[cap * 3];
        mesh::FlagType* flagsNew = new(std::nothrow) mesh::FlagType[cap];
        if (!nv || !flagsNew) {
            delete [] nv;
            delete [] flagsNew;
            return 1;
        }
		if (triCount) {
			std::memcpy(nv, tris.get(), triCount * 3 * sizeof(int));
			std::memcpy(flagsNew, flags.get(), triCount * sizeof(uint32_t));
		}
		tris.reset(nv);
		flags.reset(flagsNew);
	}
	int* dst = &tris[triCount * 3];
	*dst++ = a;
	*dst++ = b;
	*dst++ = c;
	mesh::FlagType flag = { isTri, 0, 0, 0, 0, 0, static_cast<uint8_t>(area) };
	flags[triCount] = flag;
	triCount++;
	return 0;
}

void MeshLoaderObjExt::calcResBboxes(
    float* minResBbox,
	float* maxResBbox,
	float xMinOffsetCut,
	float xMaxOffsetCut,
	float zMinOffsetCut,
	float zMaxOffsetCut
) const {
    const static float MAXF = std::numeric_limits<float>::max();
    const static float MINF = -std::numeric_limits<float>::max();
	minResBbox[0] = minResBbox[1] = minResBbox[2] = MAXF;
	maxResBbox[0] = maxResBbox[1] = maxResBbox[2] = MINF;

    for (int i = 0; i < m_triCount; ++i) {
        const int* vIds = m_tris.get() + i * 3;
        const float* v0 = &m_verts[vIds[0] * 3];
        const float* v1 = &m_verts[vIds[1] * 3];
        const float* v2 = &m_verts[vIds[2] * 3];
		rcVmin(minResBbox, v0);
		rcVmin(minResBbox, v1);
		rcVmin(minResBbox, v2);
		rcVmax(maxResBbox, v0);
		rcVmax(maxResBbox, v1);
		rcVmax(maxResBbox, v2);
    }
	minResBbox[0] += xMinOffsetCut;
	minResBbox[2] += zMinOffsetCut;
	maxResBbox[0] -= xMaxOffsetCut;
	maxResBbox[2] -= zMaxOffsetCut;
}

ErrorCode4 MeshLoaderObjExt::loadStaticMesh(
    std::unique_ptr<char[]>& staticData,
	int staticSize,
	float xMinOffsetCut,
	float xMaxOffsetCut,
	float zMinOffsetCut,
	float zMaxOffsetCut
) {
	if (xMinOffsetCut < 0.f || xMaxOffsetCut < 0.f || zMinOffsetCut < 0.f || zMaxOffsetCut < 0.f) {
		return {1, 0, 0, 0};
	}
	char* src = staticData.get();
	char* srcEnd = src + staticSize;
    ErrorCode4 ret = loadStatic(
		m_verts, m_tris, m_normals, m_flags, m_vertCount, m_triCount, src, srcEnd
	);
    if (!ret.isOk()) {
        return {2, ret.code0, ret.code1, ret.code2};
    }
    if (xMinOffsetCut == 0.f && xMaxOffsetCut == 0.f && zMinOffsetCut == 0.f && zMaxOffsetCut == 0.f) {
        return {};
    }

	float minBbox[3], maxBbox[3];
	calcResBboxes(minBbox, maxBbox, xMinOffsetCut, xMaxOffsetCut, zMinOffsetCut, zMaxOffsetCut);
	size_t newTriCount = 0;
	std::vector<int> trisForCutting;
	std::vector<float> verts(3 * m_vertCount);
	std::vector<int> tris(3 * m_triCount);
	std::vector<float> normals(3 * m_triCount);
	std::vector<mesh::FlagType> flags(m_triCount);
	std::memcpy(verts.data(), m_verts.get(), 3 * sizeof(float) * m_vertCount);

	for (int i = 0; i < m_triCount; ++i) {
	    const int* vIds = m_tris.get() + i * 3;
	    const float* v0 = &verts[vIds[0] * 3];
	    const float* v1 = &verts[vIds[1] * 3];
	    const float* v2 = &verts[vIds[2] * 3];

	    int inside0 = geometry::isPointInAabbXz(v0, minBbox, maxBbox);
	    int inside1 = geometry::isPointInAabbXz(v1, minBbox, maxBbox);
	    int inside2 = geometry::isPointInAabbXz(v2, minBbox, maxBbox);
		int nIns = inside0 + inside1 + inside2;
		if (nIns == 3) { // tri is inside
	        std::memcpy(tris.data() + newTriCount * 3, m_tris.get() + i * 3, 3 * sizeof(int));
	        std::memcpy(normals.data() + newTriCount * 3, m_normals.get() + i * 3, 3 * sizeof(float));
			flags[newTriCount] = m_flags[i];
	        ++newTriCount;
	    }
		else if (nIns > 0 && nIns < 3) { // tri and plane intersection
			trisForCutting.push_back(i);
		}
		else if (geometry::intersectionAabbVsTriangleXz(minBbox, maxBbox, v0, v1, v2)) {
			trisForCutting.push_back(i);
		}
		// else remove triangle
	}

	size_t nTrisCutted = 0;
	std::vector<int> trisCutted(3 * trisForCutting.size());
	std::vector<float> normalsCutted(3 * trisForCutting.size());
	std::vector<mesh::FlagType> flagsCutted(trisForCutting.size());
	for (int i : trisForCutting) {
		std::memcpy(trisCutted.data() + nTrisCutted * 3, m_tris.get() + i * 3, 3 * sizeof(int));
		std::memcpy(
			normalsCutted.data() + nTrisCutted * 3, m_normals.get() + i * 3, 3 * sizeof(float)
		);
		flagsCutted[nTrisCutted] = m_flags[i];
		++nTrisCutted;
	}
	size_t nTotalVerts = m_vertCount;
	cutPolygons(minBbox, maxBbox, nTotalVerts, verts, nTrisCutted, trisCutted, normalsCutted, flagsCutted);
	tris.resize(3 * (newTriCount + nTrisCutted));
	normals.resize(3 * (newTriCount + nTrisCutted));
	flags.resize(newTriCount + nTrisCutted);
	std::memcpy(
		tris.data() + 3 * newTriCount, trisCutted.data(), 3 * sizeof(int) * nTrisCutted
	);
	std::memcpy(
		normals.data() + 3 * newTriCount, normalsCutted.data(), 3 * sizeof(float) * nTrisCutted
	);
	std::memcpy(
		flags.data() + newTriCount, flagsCutted.data(), sizeof(mesh::FlagType) * nTrisCutted
	);
	newTriCount += nTrisCutted;

	removeUnusedVertices(nTotalVerts, verts, newTriCount, tris);

	m_verts.reset(new(std::nothrow) float[3 * nTotalVerts]);
	m_tris.reset(new(std::nothrow) int[3 * newTriCount]);
	m_normals.reset(new(std::nothrow) float[3 * newTriCount]);
	m_flags.reset(new(std::nothrow) mesh::FlagType[newTriCount]);
	if (!m_verts || !m_tris || !m_normals || !m_flags) {
		return { 3, 0, 0, 0 };
	}
	std::memcpy(m_verts.get(), verts.data(), 3 * sizeof(float) * nTotalVerts);
	std::memcpy(m_tris.get(), tris.data(), 3 * sizeof(int) * newTriCount);
	std::memcpy(m_normals.get(), normals.data(), 3 * sizeof(float) * newTriCount);
	std::memcpy(m_flags.get(), flags.data(), sizeof(mesh::FlagType) * newTriCount);
	m_vertCount = (int)nTotalVerts;
	m_triCount = (int)newTriCount;

    return {};
}

void MeshLoaderObjExt::cutPolygons(
	const float* minBbox,
	const float* maxBbox,
	size_t& nTotalVerts,
	std::vector<float>& verts,
	size_t& nTrisCutted,
	std::vector<int>& trisCutted,
	std::vector<float>& normalsCutted,
	std::vector<mesh::FlagType>& flagsCutted
) {
	static const size_t NUM_APPEND = 1000;
	std::vector<int> trisCuttedCp(3 * nTrisCutted);
	std::vector<float> normalsCuttedCp(3 * nTrisCutted);
	std::vector<mesh::FlagType> flagsCuttedCp(nTrisCutted);
	size_t nTrisCuttedCp = 0;

	geometry::Plane directions[4] = {
		{-1.f, 0.f, 0.f, -minBbox[0]},
		{1.f, 0.f, 0.f, maxBbox[0]},
		{0.f, 0.f, -1.f, -minBbox[2]},
		{0.f, 0.f, 1.f, maxBbox[2]}
	};
	float P0[3], P1[3], P2[3];
	auto copyVertex = [&verts, &nTotalVerts] (const float* P) {
		if (verts.size() < (nTotalVerts + 1) * 3) {
			verts.resize(verts.size() + NUM_APPEND * 3);
		}
		std::memcpy(verts.data() + 3 * nTotalVerts, P, 3 * sizeof(float));
		++nTotalVerts;
	};
	auto copyTriangle = [&minBbox, &maxBbox, &verts,
		&trisCuttedCp, &normalsCuttedCp, &flagsCuttedCp,
		&normalsCutted, &flagsCutted, &nTrisCuttedCp
	]
	(int idV0, int idV1, int idV2, size_t fromTriId) {
		if (trisCuttedCp.size() < (nTrisCuttedCp + 1) * 3) {
			trisCuttedCp.resize(trisCuttedCp.size() + NUM_APPEND * 3);
			normalsCuttedCp.resize(normalsCuttedCp.size() + NUM_APPEND * 3);
			flagsCuttedCp.resize(flagsCuttedCp.size() + NUM_APPEND);
		}

		trisCuttedCp[nTrisCuttedCp * 3] = idV0;
		trisCuttedCp[nTrisCuttedCp * 3 + 1] = idV1;
		trisCuttedCp[nTrisCuttedCp * 3 + 2] = idV2;
		std::memcpy(
			normalsCuttedCp.data() + 3 * nTrisCuttedCp,
			normalsCutted.data() + 3 * fromTriId,
			3 * sizeof(float)
		);
		flagsCuttedCp[nTrisCuttedCp] = flagsCutted[fromTriId];
		++nTrisCuttedCp;
	};

	for (int i = 0; i < 4; ++i)
	{
		const float* norm = directions[i].norm;
		const float dist = directions[i].dist;
		for (size_t j = 0; j < nTrisCutted; ++j) {
			const int* vIds = trisCutted.data() + j * 3;
			const float* v0 = &verts[vIds[0] * 3];
			const float* v1 = &verts[vIds[1] * 3];
			const float* v2 = &verts[vIds[2] * 3];
			const float d0 = rcVdot(v0, norm);
			const float d1 = rcVdot(v1, norm);
			const float d2 = rcVdot(v2, norm);
			bool res0 = geometry::intersectionSegmentVsPlane(norm, dist, v0, v1, P0);
			bool res1 = geometry::intersectionSegmentVsPlane(norm, dist, v1, v2, P1);
			bool res2 = geometry::intersectionSegmentVsPlane(norm, dist, v2, v0, P2);
			int cnt = res0 + res1 + res2;
			assert(cnt == 0 || cnt == 2);

			if (res0 && res1) {
				// add 2 new verts
				copyVertex(P0);
				copyVertex(P1);
				if (d1 < dist) {
					// replace 1 triangle
					copyTriangle(vIds[1], (int)nTotalVerts - 1, (int)nTotalVerts - 2, j);
				}
				else {
					assert(d0 < dist && d2 < dist);
					// replace 1 triangle
					copyTriangle(vIds[0], (int)nTotalVerts - 2, vIds[2], j);
					// add 1 new triangle
					copyTriangle((int)nTotalVerts - 2, (int)nTotalVerts - 1, vIds[2], j);
				}
			}
			else if (res1 && res2) {
				// add 2 new verts
				copyVertex(P1);
				copyVertex(P2);
				if (d2 < dist) {
					// replace 1 triangle
					copyTriangle(vIds[2], (int)nTotalVerts - 1, (int)nTotalVerts - 2, j);
				}
				else {
					assert(d0 < dist && d1 < dist);
					// replace 1 triangle
					copyTriangle(vIds[1], (int)nTotalVerts - 2, vIds[0], j);
					// add 1 new triangle
					copyTriangle((int)nTotalVerts - 2, (int)nTotalVerts - 1, vIds[0], j);
				}
			}
			else if (res2 && res0) {
				// add 2 new verts
				copyVertex(P2);
				copyVertex(P0);
				if (d0 < dist) {
					// replace 1 triangle
					copyTriangle(vIds[0], (int)nTotalVerts - 1, (int)nTotalVerts - 2, j);
				}
				else {
					assert(d1 < dist && d2 < dist);
					// replace 1 triangle
					copyTriangle(vIds[2], (int)nTotalVerts - 2, vIds[1], j);
					// add 1 new triangle
					copyTriangle((int)nTotalVerts - 2, (int)nTotalVerts - 1, vIds[1], j);
				}
			}
			else { // copy triangle
				int cnt = d0 < dist;
				cnt += d1 < dist;
				cnt += d2 < dist;
				if (cnt)
					copyTriangle(vIds[0], vIds[1], vIds[2], j);
			}
		}

		trisCutted.resize(3 * nTrisCuttedCp);
		normalsCutted.resize(3 * nTrisCuttedCp);
		flagsCutted.resize(nTrisCuttedCp);
		std::memcpy(trisCutted.data(), trisCuttedCp.data(), 3 * sizeof(float) * nTrisCuttedCp);
		std::memcpy(normalsCutted.data(), normalsCuttedCp.data(), 3 * sizeof(int) * nTrisCuttedCp);
		std::memcpy(flagsCutted.data(), flagsCuttedCp.data(), sizeof(mesh::FlagType) * nTrisCuttedCp);
		nTrisCutted = nTrisCuttedCp;
		nTrisCuttedCp = 0;
	}
}

void MeshLoaderObjExt::removeUnusedVertices(
	size_t& nTotalVerts,
	std::vector<float>& verts,
	size_t newTriCount,
	std::vector<int>& tris
) {
	std::unordered_map<int, std::vector<size_t>> vertsUsing;
	vertsUsing.max_load_factor(0.5f);

	for (size_t i = 0; i < newTriCount; ++i) {
		const int* vIds = tris.data() + 3 * i;
		vertsUsing[vIds[0]].push_back(i);
		vertsUsing[vIds[1]].push_back(i);
		vertsUsing[vIds[2]].push_back(i);
	}
	int vertCount = 0;
	std::vector<float> newVerts(3 * nTotalVerts);
	for (int i = 0, n = (int)nTotalVerts; i < n; ++i) {
		const auto& triIds = vertsUsing[i];
		if (triIds.empty()) {
			continue;
		}
		std::memcpy(newVerts.data() + 3 * vertCount, verts.data() + 3 * i, 3 * sizeof(float));
		for (size_t triId : triIds) {
			int* vIds = tris.data() + 3 * triId;
			if (vIds[0] == i) {
				vIds[0] = vertCount;
			}
			else if (vIds[1] == i) {
				vIds[1] = vertCount;
			}
			else {
				assert(vIds[2] == i);
				vIds[2] = vertCount;
			}
		}
		++vertCount;
	}
	verts.swap(newVerts);
	nTotalVerts = (size_t)vertCount;
}

ErrorCode4 MeshLoaderObjExt::loadStatic(
	std::unique_ptr<float[]>& verts,
	std::unique_ptr<int[]>& tris,
	std::unique_ptr<float[]>& normals,
	std::unique_ptr<mesh::FlagType[]>& flags,
	int& vertCount,
	int& triCount,
	char* src,
	char* srcEnd
) {
	char row[512];
	int face[32];
	float x, y, z;
	int nv;
	int vcap = 0;
	int tcap = 0;

	while (src < srcEnd)
	{
		// Parse one row
		row[0] = '\0';
		src = parseRow(src, srcEnd, row, sizeof(row)/sizeof(char));
		// Skip comments
		if (row[0] == '#') continue;
		if (row[0] == 'v' && row[1] != 'n' && row[1] != 't')
		{
			// Vertex pos
			sscanf(row+1, "%f %f %f", &x, &y, &z);
			if (uint8_t ret = addVertex(verts, vertCount, x, y, z, vcap)) {
				return {1, ret, 0, 0};
			}
		}
		if (row[0] == 'f')
		{
			// Faces
			int area = PolyAreaFlags::GROUND;
			nv = parseFace(row+1, face, 32, vertCount, &area);
			bool isTri = nv == 3;
			for (int i = 2; i < nv; ++i)
			{
				const int a = face[0];
				const int b = face[i-1];
				const int c = face[i];
				if (a < 0 || a >= vertCount ||
					b < 0 || b >= vertCount ||
					c < 0 || c >= vertCount
				)
					continue;
				if (uint8_t ret = addTriangle(tris, flags, triCount, a, b, c, tcap, isTri, area)) {
					return {2, ret, 0, 0};
				}
			}
		}
	}

	// Calculate normals.
    normals.reset(new(std::nothrow) float[triCount * 3]);
	if (!normals) {
		return {3, 0, 0, 0};
	}
	for (int i = 0; i < triCount * 3; i += 3)
	{
		const float* v0 = &verts[tris[i] * 3];
		const float* v1 = &verts[tris[i + 1] * 3];
		const float* v2 = &verts[tris[i + 2] * 3];
		float e0[3], e1[3];
		for (int j = 0; j < 3; ++j)
		{
			e0[j] = v1[j] - v0[j];
			e1[j] = v2[j] - v0[j];
		}
		float* n = &normals[i];
		n[0] = e0[1] * e1[2] - e0[2] * e1[1];
		n[1] = e0[2] * e1[0] - e0[0] * e1[2];
		n[2] = e0[0] * e1[1] - e0[1] * e1[0];
		float d = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
		d = 1.0f / d;
		//if (n[1] < 0.f) d *= -1.f;
		n[0] *= d;
		n[1] *= d;
		n[2] *= d;
	}

	return {};
}

char* MeshLoaderObjExt::fixStrEnd(char* src, const char* srcEnd) {
	while (src < srcEnd)
	{
		if (*src == '\n') {
			*src = '\0';
			break;
		}
		++src;
	}
	return src + 1;
}

std::vector<float> MeshLoaderObjExt::splitStrBySpacesToFloats(const char* str) {
	std::vector<float> ret;
	ret.reserve(16);
	std::istringstream iss(str);
	std::string buf;
	while(std::getline(iss, buf, ' ')) {
		if (buf.empty())
			continue;
		ret.push_back( std::stof(buf) );
	}
	return ret;
}

ErrorCode4 MeshLoaderObjExt::loadVobsAndMesh(
	std::unique_ptr<char[]>& vobsData, int vobsSize
) {
	int meshesNum = 0;
	int meshesCnt = 0;
	char* src = vobsData.get();
	char* srcEnd = src + vobsSize;

	// read meshes number
	char* newSrc = fixStrEnd(src, srcEnd);
	std::sscanf(src, "Meshes number: %d", &meshesNum);
	if (!meshesNum) {
        return {1, 0, 0, 0};
	}
	src = newSrc;
    m_vobMeshes.reset(new(std::nothrow) mesh::VobMeshEntry[meshesNum]);
    if (!m_vobMeshes) {
        return {2, 0, 0, 0};
    }
	if (!m_nameToVobMeshIndex.init(meshesNum)) {
        return {3, 0, 0, 0};
	}

	// read meshes data
	char* curEnd = nullptr;
	while (src < srcEnd)
	{
		bool out = false;
		if (meshesCnt >= meshesNum) {
            return {4, 0, 0, 0};
		}
		if (std::strncmp(src, "Visual name: ", 13)) {
			if (meshesCnt != meshesNum) {
                return {5, 0, 0, 0};
			}
			src += 3; // "\n\n\n"
			break;
		}
		src += 13;
		const char* name = src;
		src = fixStrEnd(src, srcEnd);
		mesh::VobMeshEntry& e = m_vobMeshes[meshesCnt];
		if (!e.allocVisualName()) {
            return {6, 0, 0, 0};
		}
        std::strncpy(e.visualName, name, common::Constants::NAME_SIZE);
		e.visualName[common::Constants::NAME_SIZE - 1] = '\0';
        m_nameToVobMeshIndex.put(e.visualName, meshesCnt);
		++meshesCnt;

		if (*src != '\n') {
			char* pos = std::strstr(src, "\n\n");
			if (pos) {
				pos[0] = '\0';
				curEnd = pos;
				pos += 2;
				if (pos < srcEnd && *pos == '\n') {
					pos += 1;
					out = true;
				}
			} else {
                return {7, 0, 0, 0};
			}
			std::unique_ptr<float[]> tmpVerts;
			std::unique_ptr<int[]> tmpTris;
			std::unique_ptr<float[]> tmpNormals;
			std::unique_ptr<mesh::FlagType[]> tmpFlags;
            ErrorCode4 ret = loadStatic(
				tmpVerts, tmpTris, tmpNormals, tmpFlags, e.vertCount, e.triCount, src, curEnd
			);
			e.verts = tmpVerts.release();
			e.tris = tmpTris.release();
			e.normals = tmpNormals.release();
			e.flags = tmpFlags.release();
            if (!ret.isOk()) {
                return {8, ret.code0, ret.code1, ret.code2};
            }
			if (e.vertCount > m_maxVertsPerVob)
				m_maxVertsPerVob = e.vertCount;
			if (e.triCount > m_maxTrisPerVob)
				m_maxTrisPerVob = e.triCount;
			src = pos;
			if (out) {
				break;
			}
		} else {
			// empty mesh
			// TODO log such case
			++src;
		}
	}
	m_vobMeshesCnt = meshesCnt;

	// read vobs data
	int vobsNum = 0;
	int vobsCnt = 0;
	newSrc = fixStrEnd(src, srcEnd);
	std::sscanf(src, "Vobs number: %d", &vobsNum);
	if (!vobsNum) {
        return {9, 0, 0, 0};
	}
    m_vobs.reset(new(std::nothrow) mesh::VobEntry[vobsNum]);
    if (!m_vobs) {
        return {10, 0, 0, 0};
    }

	while (src < srcEnd)
	{
		int manuallyDisabled = -1;
		if (vobsCnt >= vobsNum) {
            return {11, 0, 0, 0};
		}
		src = newSrc;
		newSrc = fixStrEnd(src, srcEnd);
		const char* vobName = src + 10; // "Vob name: "
		src = newSrc;
		newSrc = fixStrEnd(src, srcEnd);
		const char* visualName = src + 13; // "Visual name: "
		if (!std::strlen(visualName)) {
            return {12, 0, 0, 0};
		}
		int pos[1] = {-1};
        if (!m_nameToVobMeshIndex.get(visualName, pos)) {
            return {13, 0, 0, 0};
		}
		const mesh::VobMeshEntry& mesh = m_vobMeshes[pos[0]];
		if (mesh.isEmpty()) {
			// TODO: check mesh loading from the engine
			//assert(1 != 1)
			//return false;
			manuallyDisabled = 1;
		}
		src = newSrc;
		int vobType = -1;
		newSrc = fixStrEnd(src, srcEnd);
		std::sscanf(src, "Vob type: %d", &vobType);
		if (vobType == -1) {
            return {14, 0, 0, 0};
		}
		src = newSrc;
		newSrc = fixStrEnd(src, srcEnd);
		std::sscanf(src, "Manually disabled: %d", &manuallyDisabled);
		if (manuallyDisabled == -1) {
            return {15, 0, 0, 0};
		}
		src = newSrc;
        if ((vobType == 4 || vobType == 5/*is mover*/) && !std::strlen(vobName)) {
            // TODO log about empty mover name
            manuallyDisabled = 1;
        }

		if (manuallyDisabled) {
			char* pos = std::strstr(src, "\n\n");
			if (!pos) {
                return {16, 0, 0, 0};
			}
			src = pos + 2;
            newSrc = src;
			continue;
		}

		mesh::VobEntry& e = m_vobs[vobsCnt];
		if (std::strlen(vobName)) {
			if (!e.allocVobName()) {
                return {17, 0, 0, 0};
			}
            std::strncpy(e.vobName, vobName, common::Constants::NAME_SIZE);
			e.vobName[common::Constants::NAME_SIZE - 1] = '\0';
		}
		e.vobType = vobType;
		e.meshIndex = pos[0];
		if (!std::strncmp("Empty mover", src, 11)) {
			src = fixStrEnd(src, srcEnd);
		}
		int posCnt = 0;
		for (;; ++posCnt) {
			int posNum = -1;
			newSrc = fixStrEnd(src, srcEnd);
			std::sscanf(src, "Position %d:", &posNum);
			if (posNum == -1) {
				if (!posCnt) {
                    return {18, 0, 0, 0};
				} else {
					break;
				}
			}
			src = newSrc;
			if (posCnt == e.posCnt) {
				e.posCnt = posCnt * 2 + 1;
				mesh::VobPosition* newPos = new(std::nothrow) mesh::VobPosition[e.posCnt];
                if (!newPos) {
                    return {19, 0, 0, 0};
                }
                std::memcpy(newPos, e.positions, sizeof(mesh::VobPosition) * posCnt);
                delete [] e.positions;
                e.positions = newPos;
                newPos = nullptr;
			}
			mesh::VobPosition& pos = e.positions[posCnt];
			// aabb min
			newSrc = fixStrEnd(src, srcEnd);
			const char* aabbMin = src + 10; // "aabb min: "
			auto ret = splitStrBySpacesToFloats(aabbMin);
			if (ret.size() != 3) {
                return {20, 0, 0, 0};
			}
			std::memcpy(pos.aabbMin, ret.data(), sizeof(float) * 3);
			src = newSrc;
			// aabb max
			newSrc = fixStrEnd(src, srcEnd);
			const char* aabbMax = src + 10; // "aabb max: "
			ret = splitStrBySpacesToFloats(aabbMax);
			if (ret.size() != 3) {
                return {21, 0, 0, 0};
			}
			std::memcpy(pos.aabbMax, ret.data(), sizeof(float) * 3);
			src = newSrc;
			// trafo
			newSrc = fixStrEnd(src, srcEnd);
			const char* trafo = src + 7; // "trafo: "
			ret = splitStrBySpacesToFloats(trafo);
			if (ret.size() != 16) {
                return {22, 0, 0, 0};
			}
			std::memcpy(pos.trafo, ret.data(), sizeof(float) * 16);
			src = newSrc;
			// inv trafo
			newSrc = fixStrEnd(src, srcEnd);
			const char* invTrafo = src + 11; // "inv trafo: "
			ret = splitStrBySpacesToFloats(invTrafo);
			if (ret.size() != 16) {
                return {23, 0, 0, 0};
			}
			std::memcpy(pos.invTrafo, ret.data(), sizeof(float) * 16);
			src = newSrc;
		}
		e.posCnt = posCnt;
		++vobsCnt;
		++src;
	}
	m_vobsCnt = vobsCnt; // TODO

    return {};
}

static void calcVobAabbInPosition(const mesh::VobMeshEntry& mesh, const float* trafoMatrix, float* min, float* max)
{
	min[0] = min[1] = min[2] = FLT_MAX;
	max[0] = max[1] = max[2] = -FLT_MAX;

	const float* verts = mesh.verts;
	const float* vertsEnd = mesh.verts + mesh.vertCount * 3;
	float buf[3];
	for (; verts < vertsEnd; verts += 3)
	{
		geometry::transformVertex(verts, trafoMatrix, buf);
		buf[2] *= -1.f;
		geometry::vmin(min, buf);
		geometry::vmax(max, buf);
	}
}

ErrorCode4 MeshLoaderObjExt::addVobBboxesToStaticMesh()
{	
	static const int MAX_VERTS_NUM_PER_ITER = 3;
	static const int MAX_POLYS_NUM_PER_ITER = 1;
	int polysNum = m_vobsCnt * 2 * MAX_POLYS_NUM_PER_ITER;
	int vertsNum = m_vobsCnt * 2 * MAX_VERTS_NUM_PER_ITER;

    std::unique_ptr<float[]> verts(new(std::nothrow) float[vertsNum * 3]);
    std::unique_ptr<int[]> tris(new(std::nothrow) int[polysNum * 3]);
    std::unique_ptr<mesh::FlagType[]> flags(new(std::nothrow) mesh::FlagType[polysNum]);
    if (!verts || !tris || !flags)
	{
        return {1, 0, 0, 0};
    }

	int vertCnt = 0;
	int polyCnt = 0;
	for (int j = 0; j < m_vobsCnt; ++j)
	{
		mesh::VobEntry& vob = m_vobs[j];
		const mesh::VobMeshEntry& mesh = m_vobMeshes[vob.meshIndex];
		if (mesh.isEmpty())
		{
			//assert(1 != 1);
			// TODO log
			continue;
		}
		if (vob.isMover()) {
			++m_moversCnt;
		}

		vob.activePosIndex = 0; // TODO loading from an outer source

		for (int i = 0; i < vob.posCnt; ++i)
		{
			if (vertCnt + MAX_VERTS_NUM_PER_ITER > vertsNum) {
				vertsNum *= 2;
                float* vertsNew = new(std::nothrow) float[vertsNum * 3];
				if (!vertsNew)
                    return {2, 0, 0, 0};
				std::memcpy(vertsNew, verts.get(), vertCnt * 3 * sizeof(float));
				verts.reset(vertsNew);
			}
			if (polyCnt + MAX_POLYS_NUM_PER_ITER > polysNum) {
				polysNum *= 2;
				int* polysNew = new(std::nothrow) int[polysNum * 3];
				mesh::FlagType* flagsNew =
					new(std::nothrow) mesh::FlagType[polysNum];
				if (!polysNew || !flagsNew) {
					delete[] polysNew;
					delete[] flagsNew;
					return { 3, 0, 0, 0 };
				}
				std::memcpy(polysNew, tris.get(), polyCnt * 3 * sizeof(int));
				std::memcpy(flagsNew, flags.get(), polyCnt * sizeof(mesh::FlagType));
				tris.reset(polysNew);
				flags.reset(flagsNew);
			}

			mesh::VobPosition& pos = vob.positions[i];
			float aabbMin[3], aabbMax[3];
			calcVobAabbInPosition(mesh, pos.trafo, aabbMin, aabbMax);
			// renew aabb data
			geometry::vcopy(pos.aabbMin, aabbMin);
			geometry::vcopy(pos.aabbMax, aabbMax);
			const float dX = pos.aabbMax[0] - pos.aabbMin[0];
			const float dY = pos.aabbMax[1] - pos.aabbMin[1];
			const float dZ = pos.aabbMax[2] - pos.aabbMin[2];
			const float* min = &pos.aabbMin[0];

			int n = vertCnt * 3;
			int m = polyCnt * 3;
			// min vertex
			verts[n] = min[0]; verts[n + 1] = min[1]; verts[n + 2] = min[2]; // 0 - min
			n += 3;
			verts[n] = min[0] + dX; verts[n + 1] = min[1]; verts[n + 2] = min[2] + dZ; // 1
			n += 3;
			// max vertex
			verts[n] = min[0] + dX; verts[n + 1] = min[1] + dY; verts[n + 2] = min[2] + dZ; // 2 - max
			//n += 3;

			// 1 triangle that holds min and max aabb vertices
			tris[m] = vertCnt + 0; tris[m + 1] = vertCnt + 1; tris[m + 2] = vertCnt + 2;
			pos.aabbTri = polyCnt;
			//m += 3;

			mesh::FlagType& val = flags[polyCnt];
			val.isTriangle = 0;
			val.isVobPos = 1;
			val.isActiveVobPos = vob.activePosIndex == i;
			val.reserved = 0;
			val.vobIdOrCollFlags = static_cast<uint32_t>(j);
			val.isInhabited = 0;
			val.polyFlags = 0;

			vertCnt += MAX_VERTS_NUM_PER_ITER;
			polyCnt += MAX_POLYS_NUM_PER_ITER;
		}
	}

	int vertCountNew = m_vertCount + vertCnt;
	int triCountNew = m_triCount + polyCnt;
    float* newVerts = new(std::nothrow) float[vertCountNew * 3];
    int* newTris = new(std::nothrow) int[triCountNew * 3];
    mesh::FlagType* newFlags = new(std::nothrow) mesh::FlagType[triCountNew];
    if (!newVerts || !newTris || !newFlags) {
        delete [] newVerts;
        delete [] newTris;
        delete [] newFlags;
        return {4, 0, 0, 0};
    }
	std::memcpy(newVerts, m_verts.get(), m_vertCount * 3 * sizeof(float));
	std::memcpy(newVerts + m_vertCount * 3, verts.get(), vertCnt * 3 * sizeof(float));
	std::memcpy(newTris, m_tris.get(), m_triCount * 3 * sizeof(int));
	std::memcpy(newTris + m_triCount * 3, tris.get(), polyCnt * 3 * sizeof(int));
	int* trisPtr = newTris + m_triCount * 3;
	for (int i = 0, n = polyCnt * 3; i < n; i += 3) {
		trisPtr[i] += m_vertCount;
		trisPtr[i + 1] += m_vertCount;
		trisPtr[i + 2] += m_vertCount;
	}
	for (int i = 0; i < m_vobsCnt; ++i) {
		mesh::VobEntry& vob = m_vobs[i];
		for (int j = 0; j < vob.posCnt; ++j) {
			vob.positions[j].aabbTri += m_triCount;
		}
	}
	std::memcpy(newFlags, m_flags.get(), m_triCount * sizeof(mesh::FlagType));
	std::memcpy(newFlags + m_triCount, flags.get(), polyCnt * sizeof(mesh::FlagType));
	m_verts.reset(newVerts);
	m_tris.reset(newTris);
	m_flags.reset(newFlags);
    float* newNormals = new(std::nothrow) float[triCountNew * 3];
	if (!newNormals)
        return {5, 0, 0, 0};
	std::memcpy(newNormals, m_normals.get(), m_triCount * 3 * sizeof(float));
	std::memset(newNormals + m_triCount * 3, 0xff, polyCnt * 3 * sizeof(float));
	m_normals.reset(newNormals);
	m_vertCountStatic = m_vertCount;
	m_triCountStatic = m_triCount;
	m_vertCount = vertCountNew;
	m_triCount = triCountNew;

    return {};
}

uint8_t MeshLoaderObjExt::appendMarkedArea(
    std::unique_ptr<float[]>& verts, int vertsNum, float minh, float maxh, int area
) {
    if (m_markedCnt == m_markedSize) {
        m_markedSize = 1 + m_markedSize * 2;
		mesh::MarkedArea* markedNew = new(std::nothrow) mesh::MarkedArea[m_markedSize];
        if (!markedNew) {
            return 1;
        }
        for (int i = 0; i < m_markedCnt; ++i) {
			mesh::MarkedArea& mTo = markedNew[i];
			mesh::MarkedArea& mFrom = m_marked[i];
            mTo.vertsNum = mFrom.vertsNum;
            mTo.verts = mFrom.verts;
            mFrom.verts = nullptr;
            mTo.minh = mFrom.minh;
            mTo.maxh = mFrom.maxh;
            mTo.area = mFrom.area;
        }
        m_marked.reset(markedNew);
    }
	mesh::MarkedArea& mNew = m_marked[m_markedCnt];
    mNew.vertsNum = vertsNum;
    mNew.verts = verts.release();
    mNew.minh = minh;
    mNew.maxh = maxh;
    mNew.area = area;
    ++m_markedCnt;
    return 0;
}

std::vector<char*> MeshLoaderObjExt::splitByChar(char* s, char ch)
{
	std::vector<char*> ret;
	ret.reserve(16);
	char* prev = s;
	while (*s != '\0') {
		if (*s == ch) {
			*s = '\0';
			ret.push_back(prev);
			++s;
			prev = s;
			continue;
		}
		++s;
	}
	ret.push_back(prev);
	return ret;
}

std::unique_ptr<float[]> MeshLoaderObjExt::splitVerts(char* s, int nVerts)
{
	//1,2,3:3,4,5:2,2,2
	char* sprev = s;
	int datInd = 0, vInd = 0;
    std::unique_ptr<float[]> dat(new(std::nothrow) float[3 * nVerts]);
	if (!dat) {
		return {};
	}
	while (*s != '\0') {
		bool coordSep = (*s == ',');
		if (coordSep || *s == ':') {
			*s = '\0';
			dat[datInd * 3 + vInd] = static_cast<float>(std::atof(sprev));
			++s;
			sprev = s;
			if (coordSep) {
				++vInd;
			} else {
				assert(vInd == 2);
				vInd = 0;
				++datInd;
			}
			continue;
		}
		++s;
	}
	assert(vInd == 2);
	dat[datInd * 3 + vInd] = static_cast<float>(std::atof(sprev));
	return dat;
}

uint8_t MeshLoaderObjExt::loadMarked(const std::unique_ptr<char[]>& markedData, int size)
{
    return loadMarked(markedData, size, m_markedCnt, m_markedSize, m_marked);
}

uint8_t MeshLoaderObjExt::loadMarked(
	const std::unique_ptr<char[]>& markedData,
	int size,
	int& markedCnt,
    int& markedSize,
	std::unique_ptr<mesh::MarkedArea[]>& marked
) {
	static const int NUM_PARTS = 11;
	char* src = markedData.get();
	char* srcEnd = src + size;
	while (src < srcEnd)
	{
		char* strEnd = fixStrEnd(src, srcEnd);
		// comment
		if (*src == ';') {
			src = strEnd;
			continue;
		}
		std::vector<char*> parts = splitByChar(src, ' ');
		if (parts.size() < NUM_PARTS) {
			return 1;
		}
		if (
			std::strcmp(parts[0], "verts") ||
			std::strcmp(parts[3], "minh") ||
			std::strcmp(parts[5], "maxh") ||
			std::strcmp(parts[7], "flg") ||
			std::strcmp(parts[9], "disabled")
		) {
			return 2;
		}
		int vertsNum = 0, flag = -1, disabled = 0;
		float minh = std::numeric_limits<float>::max();
		float maxh = -minh;
		disabled = std::atoi(parts[10]);
		if (disabled) {
			src = strEnd;
			continue;
		}
		minh = static_cast<float>(std::atof(parts[4]));
		maxh = static_cast<float>(std::atof(parts[6]));
		if (
			minh == std::numeric_limits<float>::max() ||
			maxh == - std::numeric_limits<float>::max()
		) {
			return 3;
		}
		vertsNum = std::atoi(parts[1]);
		flag = std::atoi(parts[8]);
		if (flag == -1) {
			return 4;
		}
		char* rawVerts = parts[2];
		std::unique_ptr<float[]> verts = splitVerts(rawVerts, vertsNum);
		if (!verts) {
			return 5;
		}

		if (markedCnt == markedSize) {
			markedSize = 1 + markedSize * 2;
			mesh::MarkedArea* markedNew = new(std::nothrow) mesh::MarkedArea[markedSize];
			if (!markedNew) {
				return 6;
			}
			for (int i = 0; i < markedCnt; ++i) {
				mesh::MarkedArea& mTo = markedNew[i];
				mesh::MarkedArea& mFrom = marked[i];
				mTo.vertsNum = mFrom.vertsNum;
				mTo.verts = mFrom.verts;
				mFrom.verts = nullptr;
				mTo.minh = mFrom.minh;
				mTo.maxh = mFrom.maxh;
				mTo.area = mFrom.area;
			}
			marked.reset(markedNew);
		}
		mesh::MarkedArea& mNew = marked[markedCnt];
		mNew.vertsNum = vertsNum;
		mNew.verts = verts.release();
		mNew.minh = minh;
		mNew.maxh = maxh;
		mNew.area = flag;
		++markedCnt;

		src = strEnd;
	}
	return 0;
}

int MeshLoaderObjExt::addMarkedAreaToMemory(
	const mesh::MarkedArea& m, std::unique_ptr<char[]>& data, int& dataSize, int& dataNum
) {
	std::ostringstream oss;

	// verts 3 1,2,3:3,4,5:2,2,2 minh 111 maxh 333 flg 5 disabled 0
	assert(m.vertsNum > 2);
	oss << "verts ";
	oss << std::to_string(m.vertsNum) << " ";
	for (int i = 0; i < m.vertsNum; ++i) {
		const float* v = m.verts + i * 3;
		oss << std::to_string(v[0]) << ",";
		oss << std::to_string(v[1]) << ",";
		oss << std::to_string(v[2]);
		if (i + 1 != m.vertsNum)
			oss << ":";
		else
			oss << " ";
	}
	oss << "minh ";
	oss << std::to_string(m.minh) << " ";
	oss << "maxh ";
	oss << std::to_string(m.maxh) << " ";
	oss << "flg ";
	oss << std::to_string(m.area) << " ";
	oss << "disabled 0\n";

	std::string part = oss.str();
	int newNum = dataNum + static_cast<int>(part.size());
	if (dataSize < newNum) {
		dataSize = (dataSize + newNum) * 2;
        char* block = new(std::nothrow) char[dataSize];
		if (!block) {
			return 1;
		}
		std::memcpy(block, data.get(), dataNum);
		data.reset(block);
	}
	std::memcpy(data.get() + dataNum, part.data(), static_cast<int>(part.size()));
	dataNum += static_cast<int>(part.size());

	return 0;
}

ErrorCode4 MeshLoaderObjExt::load(
	const char* staticMeshName,
	const char* vobsMeshName,
	const char* markedMeshName,
	float xMinOffsetCut,
	float xMaxOffsetCut,
	float zMinOffsetCut,
	float zMaxOffsetCut
) {
	if (m_loaded) {
		return {1, 0, 0, 0};
	}

	int staticSize = 0;
	int vobsSize = 0;
	int markedSize = 0;
	std::unique_ptr<char[]> staticData;
	std::unique_ptr<char[]> vobsData;
	std::unique_ptr<char[]> markedData;
	std::tie(staticData, staticSize) = loadFile(staticMeshName);
	if (!staticData) {
		return {3, 0, 0, 0};
	}
	std::tie(vobsData, vobsSize) = loadFile(vobsMeshName);
	std::tie(markedData, markedSize) = loadFile(markedMeshName);

    ErrorCode4 ret = loadStaticMesh(
		staticData, staticSize, xMinOffsetCut, xMaxOffsetCut, zMinOffsetCut, zMaxOffsetCut
	);
	if (!ret.isOk()) {
        return {4, ret.code0, ret.code1, ret.code2};
	}
	if (vobsData) {
        ErrorCode4 ret = loadVobsAndMesh(vobsData, vobsSize);
        if (!ret.isOk()) {
            return {5, ret.code0, ret.code1, ret.code2};
		}
	}
	if (m_vobs) {
        ErrorCode4 ret = addVobBboxesToStaticMesh();
        if (!ret.isOk()) {
            return {6, ret.code0, ret.code1, ret.code2};
		}
	} else {
		m_vertCountStatic = m_vertCount;
		m_triCountStatic = m_triCount;
	}
	if (markedData) {
		if (uint8_t ret = loadMarked(markedData, markedSize)) {
            return {7, ret, 0, 0};
		}
	}
	m_loaded = true;

	return {};
}
