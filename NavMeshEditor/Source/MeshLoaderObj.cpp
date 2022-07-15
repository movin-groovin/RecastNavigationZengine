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
#include <new>
#define _USE_MATH_DEFINES
#include <math.h>

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

std::pair<std::unique_ptr<char[]>, int> rcMeshLoaderObjExt::loadFile(const char* filename)
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

int rcMeshLoaderObjExt::saveFile(
	const char* fileName, const std::unique_ptr<char[]>& data, int dataSize
) {
	char name[STR_SIZE + 4];
	std::strcpy(name, fileName);
	name[STR_SIZE - 1] = '\0';
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

uint8_t rcMeshLoaderObjExt::addVertex(
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

uint8_t rcMeshLoaderObjExt::addTriangle(
	std::unique_ptr<int[]>& tris, std::unique_ptr<PolyAreaFlags::FlagType[]>& flags,
	int& triCount, int a, int b, int c, int& cap, bool isTri, int area
) {
	if (triCount + 1 > cap)
	{
		cap = !cap ? 8 : cap * 2;
        int* nv = new(std::nothrow) int[cap * 3];
        PolyAreaFlags::FlagType* flagsNew = new(std::nothrow) PolyAreaFlags::FlagType[cap];
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
	PolyAreaFlags::FlagType flag = { isTri, 0, 0, 0, 0, 0, static_cast<uint8_t>(area) };
	flags[triCount] = flag;
	triCount++;
	return 0;
}

/*
static bool intersectionSegmentVsPlane(
    const float* n, const float d, const float* A, const float* B, float* P
) {
    float AB[3];
    rcVsub(AB, B, A);
    const float t = (d - rcVdot(n, A)) / rcVdot(n, AB);
    if (t <= 0.f || t >= 1.f) // except contact of segment and plane
        return false;
    rcVmad(P, A, AB, t);
    return true;
}
*/

/*
static bool intersectionSegmentVsPlane(
    const float* v0, const float* v1, const float* v2, const float* A, const float* B, float* P
) {
    float e1[3], e2[3], n[3];
    rcVsub(e1, v1, v0);
    rcVsub(e2, v2, v0);
    rcVcross(n, e1, e2);
    float d = rcVdot(n, v0);
    if (d < 0) {
        d *= -1.f;
        n[0] *= -1.f;
        n[1] *= -1.f;
        n[2] *= -1.f;
    }
    return intersectionSegmentVsPlane(n, d, A, B, P);
}
*/

static bool isPointInAabbXz(const float* p, const float* bmin, const float* bmax)
{
    return p[0] >= bmin[0] && p[0] <= bmax[0] && p[2] >= bmin[2] && p[2] <= bmax[2];
}

void rcMeshLoaderObjExt::calcResBboxes(
    float* minResBbox, float* maxResBbox, float offsetForLiquidCutting
) const {
    const static float MAXF = std::numeric_limits<float>::max();
    const static float MINF = -std::numeric_limits<float>::max();
    float minSolidBbox[3] = {MAXF, MAXF, MAXF};
    float maxSolidBbox[3] = {MINF, MINF, MINF};
    float minLiquidBbox[3] = {MAXF, MAXF, MAXF};
    float maxLiquidBbox[3] = {MINF, MINF, MINF};
    for (int i = 0; i < m_triCount; ++i) {
        const int* vIds = m_tris.get() + i * 3;
        const float* v0 = &m_verts[vIds[0] * 3];
        const float* v1 = &m_verts[vIds[1] * 3];
        const float* v2 = &m_verts[vIds[2] * 3];
        if ((uint8_t)m_flags[i].polyFlags <= PolyAreaFlags::LAVA) {
            rcVmin(minLiquidBbox, v0);
            rcVmin(minLiquidBbox, v1);
            rcVmin(minLiquidBbox, v2);
            rcVmax(maxLiquidBbox, v0);
            rcVmax(maxLiquidBbox, v1);
            rcVmax(maxLiquidBbox, v2);
        }
        else { // >= PolyAreaFlags::GROUND
            rcVmin(minSolidBbox, v0);
            rcVmin(minSolidBbox, v1);
            rcVmin(minSolidBbox, v2);
            rcVmax(maxSolidBbox, v0);
            rcVmax(maxSolidBbox, v1);
            rcVmax(maxSolidBbox, v2);
        }
    }
    minSolidBbox[0] -= offsetForLiquidCutting;
    minSolidBbox[2] -= offsetForLiquidCutting;
    maxSolidBbox[0] += offsetForLiquidCutting;
    maxSolidBbox[2] += offsetForLiquidCutting;

    minResBbox[0] = minSolidBbox[0];
    minResBbox[1] = std::min(minLiquidBbox[1], minSolidBbox[1]);
    minResBbox[2] = minSolidBbox[2];
    maxResBbox[0] = maxSolidBbox[0];
    maxResBbox[1] = std::max(maxLiquidBbox[1], maxSolidBbox[1]);
    maxResBbox[2] = maxSolidBbox[2];
}

// TODO tool for gui bbox mesh cutting (cutting, not polygon extraction, like now)
ErrorCode4 rcMeshLoaderObjExt::loadStaticMesh(
    std::unique_ptr<char[]>& staticData, int staticSize, float offsetForLiquidCutting
) {
	char* src = staticData.get();
	char* srcEnd = src + staticSize;
    ErrorCode4 ret = loadStatic(
		m_verts, m_tris, m_normals, m_flags, m_vertCount, m_triCount, src, srcEnd
	);
    if (!ret.isOk()) {
        return {1, ret.code0, ret.code1, ret.code2};
    }
    if (offsetForLiquidCutting == 0.f) {
        return {};
    }

    //std::unique_ptr<float[]> newVerts(new(std::nothrow) float[3 * m_vertCount]);
    std::unique_ptr<int[]> newTris(new(std::nothrow) int[3 * m_triCount]);
    std::unique_ptr<float[]> newNormals(new(std::nothrow) float[3 * m_triCount]);
    std::unique_ptr<PolyAreaFlags::FlagType[]> newFlags(
        new(std::nothrow) PolyAreaFlags::FlagType[m_triCount]
    );
    if (/*!newVerts || */!newTris || !newNormals || !newFlags) {
        return {2, 0, 0, 0};
    }
    float minResBbox[3], maxResBbox[3];
    calcResBboxes(minResBbox, maxResBbox, offsetForLiquidCutting);

    //std::memcpy(newVerts.get(), m_verts.get(), 3 * sizeof(int) * m_vertCount);
    int newTriCount = 0;
    for (int i = 0; i < m_triCount; ++i) {
        const int* vIds = m_tris.get() + i * 3;
        const float* v0 = &m_verts[vIds[0] * 3];
        const float* v1 = &m_verts[vIds[1] * 3];
        const float* v2 = &m_verts[vIds[2] * 3];

        int inside0 = isPointInAabbXz(v0, minResBbox, maxResBbox);
        int inside1 = isPointInAabbXz(v1, minResBbox, maxResBbox);
        int inside2 = isPointInAabbXz(v2, minResBbox, maxResBbox);
        if (inside0 + inside1 + inside2 > 0) {
            std::memcpy(newTris.get() + newTriCount * 3, m_tris.get() + i * 3, 3 * sizeof(int));
            std::memcpy(newNormals.get() + newTriCount * 3, m_normals.get() + i * 3, 3 * sizeof(float));
            newFlags[newTriCount] = m_flags[i];
            ++newTriCount;
        }
    }
    m_tris.reset(newTris.release());
    m_normals.reset(newNormals.release());
    m_flags.reset(newFlags.release());
    m_triCount = newTriCount;

    return {};
}

ErrorCode4 rcMeshLoaderObjExt::loadStatic(
	std::unique_ptr<float[]>& verts,
	std::unique_ptr<int[]>& tris,
	std::unique_ptr<float[]>& normals,
	std::unique_ptr<PolyAreaFlags::FlagType[]>& flags,
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

	if (m_enabledRendering) {
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
	}

	return {};
}

char* rcMeshLoaderObjExt::fixStrEnd(char* src, const char* srcEnd) {
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

std::vector<float> rcMeshLoaderObjExt::splitStrBySpacesToFloats(const char* str) {
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

ErrorCode4 rcMeshLoaderObjExt::loadVobsAndMesh(
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
    m_vobMeshes.reset(new(std::nothrow) MeshEntry[meshesNum]);
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
		MeshEntry& e = m_vobMeshes[meshesCnt];
		if (!e.allocVisualName()) {
            return {6, 0, 0, 0};
		}
        std::strncpy(e.visualName, name, NAME_SIZE);
		e.visualName[NAME_SIZE - 1] = '\0';
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
            ErrorCode4 ret = loadStatic(
				tmpVerts, e.tris, e.normals, e.flags, e.vertCount, e.triCount, src, curEnd
			);
			e.verts = tmpVerts.release();
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
    m_vobs.reset(new(std::nothrow) VobEntry[vobsNum]);
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
		const MeshEntry& mesh = m_vobMeshes[pos[0]];
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

		VobEntry& e = m_vobs[vobsCnt];
		if (std::strlen(vobName)) {
			if (!e.allocVobName()) {
                return {17, 0, 0, 0};
			}
            std::strncpy(e.vobName, vobName, NAME_SIZE);
			e.vobName[NAME_SIZE - 1] = '\0';
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
                Position* newPos = new(std::nothrow) Position[e.posCnt];
                if (!newPos) {
                    return {19, 0, 0, 0};
                }
                std::memcpy(newPos, e.positions, sizeof(Position) * posCnt);
                delete [] e.positions;
                e.positions = newPos;
                newPos = nullptr;
			}
			Position& pos = e.positions[posCnt];
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

ErrorCode4 rcMeshLoaderObjExt::addVobBboxesToStaticMesh()
{
	int polysNum = m_vobsCnt * 2 * 12;
	int vertsNum = m_vobsCnt * 2 * 8;
    std::unique_ptr<float[]> verts(new(std::nothrow) float[vertsNum * 3]);
    std::unique_ptr<int[]> tris(new(std::nothrow) int[polysNum * 3]);
    std::unique_ptr<PolyAreaFlags::FlagType[]> flags(
        new(std::nothrow) PolyAreaFlags::FlagType[polysNum]
    );
    if (!verts || !tris || !flags) {
        return {1, 0, 0, 0};
    }
	int vertCnt = 0;
	int polyCnt = 0;
	for (int j = 0; j < m_vobsCnt; ++j)
	{
		VobEntry& vob = m_vobs[j];
		const MeshEntry& mesh = m_vobMeshes[vob.meshIndex];
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
			Position& pos = vob.positions[i];
			const float dX = pos.aabbMax[0] - pos.aabbMin[0];
			const float dY = pos.aabbMax[1] - pos.aabbMin[1];
			const float dZ = pos.aabbMax[2] - pos.aabbMin[2];
			const float* v = &pos.aabbMin[0];

			if (vertCnt == vertsNum) {
				vertsNum *= 2;
                float* vertsNew = new(std::nothrow) float[vertsNum * 3];
				if (!vertsNew)
                    return {2, 0, 0, 0};
				std::memcpy(vertsNew, verts.get(), vertCnt * 3 * sizeof(float));
				verts.reset(vertsNew);
			}
			int n = vertCnt * 3;
			verts[n] = v[0]; verts[n + 1] = v[1] + dY; verts[n + 2] = v[2] + dZ; // 0
			n += 3;
			verts[n] = v[0]; verts[n + 1] = v[1] + dY; verts[n + 2] = v[2]; // 1
			n += 3;
			verts[n] = v[0]; verts[n + 1] = v[1]; verts[n + 2] = v[2] + dZ; // 2
			n += 3;
			verts[n] = v[0]; verts[n + 1] = v[1]; verts[n + 2] = v[2]; // 3
			n += 3;
			verts[n] = v[0] + dX; verts[n + 1] = v[1] + dY; verts[n + 2] = v[2] + dZ; // 4
			n += 3;
			verts[n] = v[0] + dX; verts[n + 1] = v[1] + dY; verts[n + 2] = v[2]; // 5
			n += 3;
			verts[n] = v[0] + dX; verts[n + 1] = v[1]; verts[n + 2] = v[2] + dZ; // 6
			n += 3;
			verts[n] = v[0] + dX; verts[n + 1] = v[1]; verts[n + 2] = v[2]; // 7
			//n += 3;

			if (polyCnt == polysNum) {
				polysNum *= 2;
                int* polysNew = new(std::nothrow) int[polysNum * 3];
                PolyAreaFlags::FlagType* flagsNew =
                    new(std::nothrow) PolyAreaFlags::FlagType[polysNum];
                if (!polysNew || !flagsNew) {
                    delete [] polysNew;
                    delete [] flagsNew;
                    return {3, 0, 0, 0};
                }
				std::memcpy(polysNew, tris.get(), polyCnt * 3 * sizeof(int));
				std::memcpy(flagsNew, flags.get(), polyCnt * sizeof(PolyAreaFlags::FlagType));
				tris.reset(polysNew);
				flags.reset(flagsNew);
			}
			int m = polyCnt * 3;
			tris[m] = vertCnt; tris[m+1] = vertCnt + 1; tris[m+2] = vertCnt + 2;
			pos.aabbTris[0] = polyCnt;
			m += 3;
			tris[m] = vertCnt + 1; tris[m+1] = vertCnt + 3; tris[m+2] = vertCnt + 2;
			pos.aabbTris[1] = polyCnt + 1;
			m += 3;
			tris[m] = vertCnt + 4; tris[m+1] = vertCnt + 5; tris[m+2] = vertCnt + 6;
			pos.aabbTris[2] = polyCnt + 2;
			m += 3;
			tris[m] = vertCnt + 5; tris[m+1] = vertCnt + 7; tris[m+2] = vertCnt + 6;
			pos.aabbTris[3] = polyCnt + 3;
			m += 3;
			tris[m] = vertCnt + 6; tris[m+1] = vertCnt + 7; tris[m+2] = vertCnt + 2;// 672 - 4
			pos.aabbTris[4] = polyCnt + 4;
			m += 3;
			tris[m] = vertCnt + 2; tris[m+1] = vertCnt + 7; tris[m+2] = vertCnt + 3;//273 - 5
			pos.aabbTris[5] = polyCnt + 5;
			m += 3;
			tris[m] = vertCnt + 4; tris[m+1] = vertCnt + 5; tris[m+2] = vertCnt;
			pos.aabbTris[6] = polyCnt + 6;
			m += 3;
			tris[m] = vertCnt; tris[m+1] = vertCnt + 5; tris[m+2] = vertCnt + 1;
			pos.aabbTris[7] = polyCnt + 7;
			m += 3;
			tris[m] = vertCnt + 2; tris[m+1] = vertCnt + 6; tris[m+2] = vertCnt;
			pos.aabbTris[8] = polyCnt + 8;
			m += 3;
			tris[m] = vertCnt; tris[m+1] = vertCnt + 6; tris[m+2] = vertCnt + 4;
			pos.aabbTris[9] = polyCnt + 9;
			m += 3;
			tris[m] = vertCnt + 1; tris[m+1] = vertCnt + 3; tris[m+2] = vertCnt + 7;
			pos.aabbTris[10] = polyCnt + 10;
			m += 3;
			tris[m] = vertCnt + 1; tris[m+1] = vertCnt + 7; tris[m+2] = vertCnt + 5;
			pos.aabbTris[11] = polyCnt + 11;
			//m += 3;

			for (int k = polyCnt; k < polyCnt + 12; ++k) {
				flags[k] =
					{ 0, 1, vob.activePosIndex == i, 0, static_cast<uint32_t>(j), 0, 0 };
			}
			if (vob.hasNavmeshFlagsInfluence()) {
				uint8_t flagValue;
				if (vob.isLadder()) {
					flagValue = PolyAreaFlags::LADDER;
				} else if (vob.isDoor()) {
					flagValue = PolyAreaFlags::DOOR;
				} else {
					assert(1 != 1);
					flagValue = 0;
				}
                flags[polyCnt + 4].polyFlags = flagValue;
                flags[polyCnt + 5].polyFlags = flagValue;
                /*
                // Doors and ladders by mean convex areas
                std::unique_ptr<float[]> bottomVerts(new(std::nothrow) float [3 * 4]);
                if (!bottomVerts)
                    return {7, 0, 0, 0};
                // bottom, verts: 2 3 7 6
                float* vertsTo = bottomVerts.get();
                std::memcpy(vertsTo, verts.get() + (vertCnt + 2) * 3, 3 * sizeof(float));
                std::memcpy(vertsTo + 3, verts.get() + (vertCnt + 3) * 3, 3 * sizeof(float));
                std::memcpy(vertsTo + 6, verts.get() + (vertCnt + 7) * 3, 3 * sizeof(float));
                std::memcpy(vertsTo + 9, verts.get() + (vertCnt + 6) * 3, 3 * sizeof(float));
                uint8_t ret = appendMarkedArea(bottomVerts, 4, v[1], v[1] + 10.f, flagValue);
                if (ret)
                    return {8, ret, 0, 0};
                */
			}

			vertCnt += 8;
			polyCnt += 12;
		}
	}

	int vertCountNew = m_vertCount + vertCnt;
	int triCountNew = m_triCount + polyCnt;
    float* newVerts = new(std::nothrow) float[vertCountNew * 3];
    int* newTris = new(std::nothrow) int[triCountNew * 3];
    PolyAreaFlags::FlagType* newFlags = new(std::nothrow) PolyAreaFlags::FlagType[triCountNew];
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
	std::memcpy(newFlags, m_flags.get(), m_triCount * sizeof(PolyAreaFlags::FlagType));
	std::memcpy(newFlags + m_triCount, flags.get(), polyCnt * sizeof(PolyAreaFlags::FlagType));
	m_verts.reset(newVerts);
	m_tris.reset(newTris);
	m_flags.reset(newFlags);
	if (m_enabledRendering) {
        float* newNormals = new(std::nothrow) float[triCountNew * 3];
		if (!newNormals)
            return {5, 0, 0, 0};
		std::memcpy(newNormals, m_normals.get(), m_triCount * 3 * sizeof(float));
		std::memset(newNormals + m_triCount * 3, 0xff, polyCnt * 3 * sizeof(float));
		m_normals.reset(newNormals);
	}
	m_vertCountStatic = m_vertCount;
	m_triCountStatic = m_triCount;
	m_vertCount = vertCountNew;
	m_triCount = triCountNew;

    return {};
}

uint8_t rcMeshLoaderObjExt::appendMarkedArea(
    std::unique_ptr<float[]>& verts, int vertsNum, float minh, float maxh, int area
) {
    if (m_markedCnt == m_markedSize) {
        m_markedSize = 1 + m_markedSize * 2;
        MarkedEntry* markedNew = new(std::nothrow) MarkedEntry[m_markedSize];
        if (!markedNew) {
            return 1;
        }
        for (int i = 0; i < m_markedCnt; ++i) {
            MarkedEntry& mTo = markedNew[i];
            MarkedEntry& mFrom = m_marked[i];
            mTo.vertsNum = mFrom.vertsNum;
            mTo.verts = mFrom.verts;
            mFrom.verts = nullptr;
            mTo.minh = mFrom.minh;
            mTo.maxh = mFrom.maxh;
            mTo.area = mFrom.area;
        }
        m_marked.reset(markedNew);
    }
    MarkedEntry& mNew = m_marked[m_markedCnt];
    mNew.vertsNum = vertsNum;
    mNew.verts = verts.release();
    mNew.minh = minh;
    mNew.maxh = maxh;
    mNew.area = area;
    ++m_markedCnt;
    return 0;
}

std::vector<char*> rcMeshLoaderObjExt::splitByChar(char* s, char ch)
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

std::unique_ptr<float[]> rcMeshLoaderObjExt::splitVerts(char* s, int nVerts)
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

uint8_t rcMeshLoaderObjExt::loadMarked(const std::unique_ptr<char[]>& markedData, int size)
{
    return loadMarked(markedData, size, m_markedCnt, m_markedSize, m_marked);
}

uint8_t rcMeshLoaderObjExt::loadMarked(
	const std::unique_ptr<char[]>& markedData,
	int size,
	int& markedCnt,
    int& markedSize,
	std::unique_ptr<MarkedEntry[]>& marked
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
            MarkedEntry* markedNew = new(std::nothrow) MarkedEntry[markedSize];
			if (!markedNew) {
				return 6;
			}
			for (int i = 0; i < markedCnt; ++i) {
				MarkedEntry& mTo = markedNew[i];
				MarkedEntry& mFrom = marked[i];
				mTo.vertsNum = mFrom.vertsNum;
				mTo.verts = mFrom.verts;
				mFrom.verts = nullptr;
				mTo.minh = mFrom.minh;
				mTo.maxh = mFrom.maxh;
				mTo.area = mFrom.area;
			}
			marked.reset(markedNew);
		}
		MarkedEntry& mNew = marked[markedCnt];
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

int rcMeshLoaderObjExt::addMarkedAreaToMemory(
	const MarkedEntry& m, std::unique_ptr<char[]>& data, int& dataSize, int& dataNum
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

ErrorCode4 rcMeshLoaderObjExt::load(
	const char* navMeshName,
	const char* staticMeshName,
	const char* vobsMeshName,
	const char* markedMeshName,
    float offsetForLiquidCutting,
	bool enabledRendering
) {
	if (m_loaded) {
		return {1, 0, 0, 0};
	}
    m_navMeshName.reset(new(std::nothrow) char[1024]);
	if (!m_navMeshName) {
		return {2, 0, 0, 0};
	}
	std::strcpy(m_navMeshName.get(), navMeshName);
	m_navMeshName[1024 - 1] = '\0';

	m_enabledRendering = enabledRendering;
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

    ErrorCode4 ret = loadStaticMesh(staticData, staticSize, offsetForLiquidCutting);
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

void rcMeshLoaderObjExt::calcBounds(float* bMin, float* bMax) const
{
	rcVcopy(bMin, &m_verts[0]);
	rcVcopy(bMax, &m_verts[0]);
	for (int i = 3; i < m_vertCount * 3; i += 3)
	{
		const float* v = &m_verts[i];
		rcVmin(bMin, v);
		rcVmax(bMax, v);
	}
}
