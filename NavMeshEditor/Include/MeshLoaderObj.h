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

#ifndef MESHLOADER_OBJ
#define MESHLOADER_OBJ

#include <string>
#include <memory>
#include <cstring>
#include <functional>
#include <vector>
#include <new>
#include <cmath>

#include "Recast.h"
#include "Common.h"
#include "Mesh.h"
#include "Geometry.h"

struct ErrorCode4
{
	uint8_t code0 = 0;
	uint8_t code1 = 0;
	uint8_t code2 = 0;
	uint8_t code3 = 0;

	bool isOk() const { return !((uint32_t)(code0) + code1 + code2 + code3); }
};

class MeshLoaderObjExt: public mesh::MeshLoaderInterface
{
public:
	MeshLoaderObjExt() = default;
	~MeshLoaderObjExt() = default;

	const float* getVerts() const override { return m_verts.get(); }
	const float* getNormals() const override { return m_normals.get(); }
	const int* getTris() const override { return m_tris.get(); }
	const mesh::FlagType* getFlags() const override { return m_flags.get(); }
	int getVertCount() const override { return m_vertCount; }
	int getTriCount() const override { return m_triCount; }
	int getVertCountStatic() const override { return m_vertCountStatic; }
	int getTriCountStatic() const override { return m_triCountStatic; }
	int getMaxVertsPerVob() const override { return m_maxVertsPerVob; }
	int getMaxTrisPerVob() const override { return m_maxTrisPerVob; }
	int getVobsCnt() const override { return m_vobsCnt; }
	int getMoversCnt() const override { return m_moversCnt; }
	int getVobMeshesCnt() const override { return m_vobMeshesCnt; }
	const mesh::VobEntry* getVobs() const override { return m_vobs.get(); }
	const common::HashMultiStrToInt&
		getNameToVobMeshIndex() const override { return m_nameToVobMeshIndex; }
	const mesh::VobMeshEntry* getVobMeshes() const override { return m_vobMeshes.get(); }
	const mesh::MarkedArea* getMarked() const override { return m_marked.get(); }
	const int getMarkedCount() const override { return m_markedCnt; }
	bool isLoaded() const override { return m_loaded; }

	ErrorCode4 load(
		const char* staticMeshName,
		const char* vobsMeshName,
		const char* markedMeshName,
		float xMinOffsetCut,
		float xMaxOffsetCut,
		float zMinOffsetCut,
		float zMaxOffsetCut
	);

	static uint8_t loadMarked(
		const std::unique_ptr<char[]>& markedData,
		int size,
		int& markedCnt,
        int& markedSize,
		std::unique_ptr<mesh::MarkedArea[]>& marked
	);
	static std::pair<std::unique_ptr<char[]>, int> loadFile(const char* filename);
	static int saveFile(
		const char* fileName, const std::unique_ptr<char[]>& data, int dataSize
	);
	static int addMarkedAreaToMemory(
		const mesh::MarkedArea& m, std::unique_ptr<char[]>& data, int& dataSize, int& dataNum
	);

private:
    ErrorCode4 loadStaticMesh(
        std::unique_ptr<char[]>& staticData,
		int staticSize,
		float xMinOffsetCut,
		float xMaxOffsetCut,
		float zMinOffsetCut,
		float zMaxOffsetCut
    );
	ErrorCode4 loadStatic(
		std::unique_ptr<float[]>& verts,
		std::unique_ptr<int[]>& tris,
		std::unique_ptr<float[]>& normals,
		std::unique_ptr<mesh::FlagType[]>& flags,
		int& vertCount,
		int& triCount,
		char* src,
		char* srcEnd
	);
	void cutPolygons(
		const float* minBbox,
		const float* maxBbox,
		size_t& nTotalVerts,
		std::vector<float>& verts,
		size_t& nTrisCutted,
		std::vector<int>& trisCutted,
		std::vector<float>& normalsCutted,
		std::vector<mesh::FlagType>& flagsCutted
	);
	void removeUnusedVertices(
		size_t& nTotalVerts,
		std::vector<float>& verts,
		size_t newTriCount,
		std::vector<int>& tris
	);
    ErrorCode4 loadVobsAndMesh(std::unique_ptr<char[]>& vobsData, int vobsSize);
    uint8_t loadMarked(const std::unique_ptr<char[]>& markedData, int size);
    ErrorCode4 addVobBboxesToStaticMesh();
    uint8_t appendMarkedArea(
        std::unique_ptr<float[]>& verts, int vertsNum, float minh, float maxh, int area
    );
    void calcResBboxes(
		float* minResBbox,
		float* maxResBbox,
		float xMinOffsetCut,
		float xMaxOffsetCut,
		float zMinOffsetCut,
		float zMaxOffsetCut
	) const;

	static uint8_t addTriangle(
		std::unique_ptr<int[]>& tris, std::unique_ptr<mesh::FlagType[]>& flags,
		int& triCount, int a, int b, int c, int& cap, bool isTri, int area
	);
	static uint8_t addVertex(
		std::unique_ptr<float[]>& verts, int& vertCount,
		float x, float y, float z, int& cap
	);
	static std::vector<float> splitStrBySpacesToFloats(const char* str);
	static std::vector<char*> splitByChar(char* s, char ch);
	static std::unique_ptr<float[]> splitVerts(char* s, int nVerts);
	static char* fixStrEnd(char* src, const char* srcEnd);

private:
	bool m_loaded = false;
	// static mesh
	std::unique_ptr<float[]> m_verts;
	std::unique_ptr<int[]> m_tris;
	std::unique_ptr<float[]> m_normals;
	std::unique_ptr<mesh::FlagType[]> m_flags;
	int m_vertCount = 0;
	int m_triCount = 0;
	int m_vertCountStatic = 0;
	int m_triCountStatic = 0;
	// vobs mesh
	int m_vobsCnt = 0;
	int m_moversCnt = 0;
	int m_vobMeshesCnt = 0;
	int m_maxVertsPerVob = 0;
	int m_maxTrisPerVob = 0;
	std::unique_ptr<mesh::VobEntry[]> m_vobs;
	common::HashMultiStrToInt m_nameToVobMeshIndex;
	std::unique_ptr<mesh::VobMeshEntry[]> m_vobMeshes;
	// marked mesh
	int m_markedCnt = 0;
    int m_markedSize = 0;
	std::unique_ptr<mesh::MarkedArea[]> m_marked;
};

#endif // MESHLOADER_OBJ
