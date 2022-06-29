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

struct StringHash
{
	int operator()(const char* s, int M) const
	{
		int h, a = 31415, b = 27183, Mmin1 = M - 1;
		for (h = 0; *s != '\0'; ++s, a = a * b % Mmin1)
			h = (a * h + *s) % M;
		return (h < 0) ? (h + M) : h;
	}
};

template <typename Hash = StringHash>
class LinearHashMultiStrToInt
{
public:
	using key_type = const char*;
	using value_type = int;
	using hash_type = Hash;

private:
	struct Bucket {
		key_type k;
		value_type v;
	};
	static const value_type INVALID = -1;

public:
	LinearHashMultiStrToInt(): m_numBuckets(), m_loadFactor(), m_data() {}
	~LinearHashMultiStrToInt() { release(); }
	LinearHashMultiStrToInt(const LinearHashMultiStrToInt&) = delete;
	LinearHashMultiStrToInt& operator=(const LinearHashMultiStrToInt&) = delete;

	bool init(const size_t numBuckets, const float loadFactor = 0.25f) {
		if (loadFactor >= 1.f || loadFactor <= 0.f) {
			return false;
		}
		m_loadFactor = loadFactor;
		m_numBuckets = static_cast<size_t>(std::ceil(1.f / m_loadFactor)) * numBuckets;
        m_data = new(std::nothrow) Bucket[m_numBuckets];
		if (!m_data)
			return false;
		std::memset(m_data, 0xff, m_numBuckets * sizeof(Bucket));
		return true;
	}

	size_t getMemSize() const {
		return sizeof(Bucket) * m_numBuckets;
	}

	void release() {
		delete [] m_data;
		m_hash = {};
		m_numBuckets = {};
		m_loadFactor = {};
	}

	template <size_t I>
	size_t find(key_type k) const {
		size_t pos[I];
		return findPos(k, pos);
	}

	template <size_t I>
	size_t get(key_type k, value_type(&v)[I]) const {
		size_t pos[I];
		size_t n = findPos(k, pos);
		for (size_t i = 0; i < n; ++i) {
			v[i] = m_data[pos[i]].v;
		}
		return n;
	}

	bool put(key_type k, value_type v) {
		size_t bid = m_hash(k, m_numBuckets);
		for (size_t i = bid; i < m_numBuckets; ++i) {
			Bucket& b = m_data[i];
			if (b.v == INVALID) {
				b.k = k;
				b.v = v;
				return true;
			}
		}
		for (size_t i = 0; i < bid; ++i) {
			Bucket& b = m_data[i];
			if (b.v == INVALID) {
				b.k = k;
				b.v = v;
				return true;
			}
		}
		return false;
	}

private:
	template <size_t I>
	size_t findPos(const char* k, size_t(&pos)[I]) const {
		static_assert(I > 0, "Incorrect I");
		size_t n = 0;
		size_t bid = m_hash(k, m_numBuckets);
		for (size_t i = bid; i < m_numBuckets; ++i) {
			const Bucket& b = m_data[i];
			if (b.v == INVALID)
				return n;
			if (!std::strcmp(b.k, k)) {
				pos[n++] = i;
				if (n == I)
					return n;
			}
		}
		for (size_t i = 0; i < bid; ++i) {
			const Bucket& b = m_data[i];
			if (b.v == INVALID)
				return n;
			if (!std::strcmp(b.k, k)) {
				pos[n++] = i;
				if (n == I)
					return n;
			}
		}
		return n;
	}

private:
	hash_type m_hash;
	size_t m_numBuckets;
	float m_loadFactor;
	Bucket* m_data;
};

struct ErrorCode4
{
	uint8_t code0 = 0;
	uint8_t code1 = 0;
	uint8_t code2 = 0;
	uint8_t code3 = 0;

	bool isOk() const { return !((uint32_t)(code0) + code1 + code2 + code3); }
};

class rcMeshLoaderObjExt
{
public:
	static const int NAME_SIZE = 256;
	static const int STR_SIZE = 1024;
    // TODO replace to common header (also from InputGeom)
    static const int REGULAR_VERTS_BLOCK = 3;
#ifdef USAGE_SSE_1_0
    static const int CUR_VERTS_BLOCK = 4;
#else
    static const int CUR_VERTS_BLOCK = 3;
#endif

public:
	struct MarkedEntry {
		MarkedEntry() = default;
		~MarkedEntry() {
			delete [] verts;
			verts = nullptr;
		}

		bool copy(MarkedEntry& to) {
			to.vertsNum = vertsNum;
			to.minh = minh;
			to.maxh = maxh;
			to.area = area;
            to.verts = new(std::nothrow) float[3 * to.vertsNum];
            if (!to.verts) {
				return false;
			}
			std::memcpy(to.verts, verts, sizeof(float) * 3 * to.vertsNum);
			return true;
		}

		int vertsNum = 0;
		float* verts = nullptr;
		float minh = 0;
		float maxh = 0;
		int area = - 1;
	};

	struct Position
	{
		static const int POS_TRIS_NUM = 12;

		float aabbMin[3];
		float aabbMax[3];
		float trafo[4 * 4];
		float invTrafo[4 * 4];
		int aabbTris[POS_TRIS_NUM];
	};

	struct VobEntry
	{
		static const int INIT_POS_CNT = 1;

		VobEntry():
            vobName(nullptr), vobType(-1), meshIndex(-1), posCnt(0),
            activePosIndex(-1), positions(nullptr), vertsPosRendering(-1) {}

        ~VobEntry() {
            delete [] vobName;
            vobName = nullptr;
            delete [] positions;
            positions = nullptr;
        }

		bool init(int posNum = INIT_POS_CNT) {
			posCnt = posNum;
            positions = new((std::nothrow)) Position[posCnt];
            return positions;
		}

		// TODO change to human constant names from zengine
		bool isMover() const {
			return vobType == 4 || vobType == 5;
		}

		bool isDoor() const {
			return vobType == 3;
		}

		bool isLadder() const {
			return vobType == 2;
		}

		bool hasNavmeshFlagsInfluence() const { // ladder or door
			return isLadder() || isDoor();
		}

		bool allocVobName() {
            vobName = new(std::nothrow) char[NAME_SIZE];
            return vobName;
		}

		bool copy(VobEntry& to) const {
			if (this == &to)
				return false;
			if (!posCnt)
				return false;

			if (vobName) {
				if (!to.allocVobName()) {
					return false;
				}
                std::strncpy(to.vobName, vobName, NAME_SIZE);
			}
			to.vobType = vobType;
			to.meshIndex = meshIndex;
			to.posCnt = posCnt;
			to.activePosIndex = activePosIndex;
            to.positions = new(std::nothrow) Position[to.posCnt];
            if (!to.positions)
                return false;
			for (int i = 0; i < to.posCnt; ++i) {
				std::memcpy(&to.positions[i], &positions[i], sizeof(Position));
			}
			to.vertsPosRendering = vertsPosRendering;

			return true;
		}

        char* vobName;
		int vobType;
		int meshIndex;
		int posCnt;
		int activePosIndex;
        Position* positions;
		int vertsPosRendering;
	};

	struct MeshEntry
	{
        MeshEntry() = default;
        ~MeshEntry() {
            delete [] visualName;
            visualName = nullptr;
			if (alignedVerts)
				freeAlignedArr(verts, 0);
			else
				delete [] verts;
			verts = nullptr;
        }

        bool allocVisualName() {
            visualName = new(std::nothrow) char[NAME_SIZE];
            return visualName;
		}

        bool copy(MeshEntry& to) const {
			if (this == &to)
				return false;
			if (!vertCount || !triCount || !visualName)
				return false;

			if (!to.allocVisualName()) {
				return false;
			}
            std::strncpy(to.visualName, visualName, NAME_SIZE - 1);
			to.visualName[NAME_SIZE - 1] = '\0';
			to.verts = allocAlignedArr<float>(vertCount * CUR_VERTS_BLOCK, 16);
			to.alignedVerts = true;
            to.tris.reset( new(std::nothrow) int[triCount * 3] );
			if (normals)
                to.normals.reset( new(std::nothrow) float[triCount * 3] );
            to.flags.reset( new(std::nothrow) PolyAreaFlags::FlagType[triCount] );
			if (!to.verts || !to.tris || (!to.normals && normals) || !to.flags) {
				return false;
			}
			for (int i = 0; i < vertCount; ++i) {
                std::memcpy(to.verts + i * CUR_VERTS_BLOCK, verts + i * 3, 3 * sizeof(float));
#ifdef USAGE_SSE_1_0
				to.verts[i * CUR_VERTS_BLOCK + 3] = 0.f;
#endif
			}
            int* trisTo = to.tris.get();
            const int* trisFrom = tris.get();
            for (int i = 0; i < 3 * triCount; ++i) {
                trisTo[i] = trisFrom[i] * CUR_VERTS_BLOCK;
            }
			if (normals)
                std::memcpy(to.normals.get(), normals.get(), triCount * 3 * sizeof(float));
            std::memcpy(to.flags.get(), flags.get(), triCount * sizeof(PolyAreaFlags::FlagType));
            to.vertCount = vertCount;
			to.triCount = triCount;
			return true;
		}

		bool isEmpty() const { return !vertCount; }

        char* visualName = nullptr;
        float* verts = nullptr;
		bool alignedVerts = false; // TODO remove this crutch
		// TODO replace other data to raw ptrs
        std::unique_ptr<int[]> tris;
        std::unique_ptr<float[]> normals;
        std::unique_ptr<PolyAreaFlags::FlagType[]> flags;
		int vertCount = 0;
		int triCount = 0;
	};

public:
	rcMeshLoaderObjExt() = default;
	~rcMeshLoaderObjExt() = default;

	ErrorCode4 load(
		const char* navMeshName,
		const char* staticMeshName,
		const char* vobsMeshName,
		const char* markedMeshName,
        float offsetForLiquidCutting,
		bool enabledRendering
	);

	const char* getNavMeshName() const { return m_navMeshName.get(); }
	bool isEnabledRendering() const { return m_enabledRendering; }
	const float* getVerts() const { return m_verts.get(); }
	const float* getNormals() const { return m_normals.get(); }
	const int* getTris() const { return m_tris.get(); }
	const PolyAreaFlags::FlagType* getFlags() const { return m_flags.get(); }
	int getVertCount() const { return m_vertCount; }
	int getTriCount() const { return m_triCount; }
	int getVertCountStatic() const { return m_vertCountStatic; }
	int getTriCountStatic() const { return m_triCountStatic; }
	int getMaxVertsPerVob() const { return m_maxVertsPerVob; }
	int getMaxTrisPerVob() const { return m_maxTrisPerVob; }
	int getVobsCnt() const { return m_vobsCnt; }
	int getMoversCnt() const { return m_moversCnt; }
	int getVobMeshesCnt() const { return m_vobMeshesCnt; }
	const VobEntry* getVobs() const { return m_vobs.get(); }
	const LinearHashMultiStrToInt<>&
		getNameToVobMeshIndex() const { return m_nameToVobMeshIndex; }
	const MeshEntry* getVobMeshes() const { return m_vobMeshes.get(); }
	const MarkedEntry* getMarked() const { return m_marked.get(); }
	const int getMarkedCount() const { return m_markedCnt; }
	bool isLoaded() const { return m_loaded; }

    void calcBounds(float* bMin, float* bMax) const;
	static uint8_t loadMarked(
		const std::unique_ptr<char[]>& markedData,
		int size,
		int& markedCnt,
        int& markedSize,
		std::unique_ptr<MarkedEntry[]>& marked
	);
	static std::pair<std::unique_ptr<char[]>, int> loadFile(const char* filename);
	static int saveFile(
		const char* fileName, const std::unique_ptr<char[]>& data, int dataSize
	);
	static int addMarkedAreaToMemory(
		const MarkedEntry& m, std::unique_ptr<char[]>& data, int& dataSize, int& dataNum
	);

private:
    ErrorCode4 loadStaticMesh(
        std::unique_ptr<char[]>& staticData, int staticSize, float offsetForLiquidCutting
    );
	ErrorCode4 loadStatic(
		std::unique_ptr<float[]>& verts,
		std::unique_ptr<int[]>& tris,
		std::unique_ptr<float[]>& normals,
		std::unique_ptr<PolyAreaFlags::FlagType[]>& flags,
		int& vertCount,
		int& triCount,
		char* src,
		char* srcEnd
	);
    ErrorCode4 loadVobsAndMesh(std::unique_ptr<char[]>& vobsData, int vobsSize);
    uint8_t loadMarked(const std::unique_ptr<char[]>& markedData, int size);
    ErrorCode4 addVobBboxesToStaticMesh();
    uint8_t appendMarkedArea(
        std::unique_ptr<float[]>& verts, int vertsNum, float minh, float maxh, int area
    );
    void calcResBboxes(float* minResBbox, float* maxResBbox, float offsetForLiquidCutting) const;

	static uint8_t addTriangle(
		std::unique_ptr<int[]>& tris, std::unique_ptr<PolyAreaFlags::FlagType[]>& flags,
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
	bool m_enabledRendering = false;
	std::unique_ptr<char[]> m_navMeshName;
	// static mesh
	std::unique_ptr<float[]> m_verts;
	std::unique_ptr<int[]> m_tris;
	std::unique_ptr<float[]> m_normals;
	std::unique_ptr<PolyAreaFlags::FlagType[]> m_flags;
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
	std::unique_ptr<VobEntry[]> m_vobs;
	LinearHashMultiStrToInt<> m_nameToVobMeshIndex;
	std::unique_ptr<MeshEntry[]> m_vobMeshes;
	// marked mesh
	int m_markedCnt = 0;
    int m_markedSize = 0;
	std::unique_ptr<MarkedEntry[]> m_marked;
};

#endif // MESHLOADER_OBJ
