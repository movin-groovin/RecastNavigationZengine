#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <cstdint>
#include <utility>
#include <cstdlib>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cassert>
#ifdef WIN32
#include <malloc.h>
#else
#include <mm_malloc.h>
#endif // WIN32
#include <exception>

static const unsigned char RC_NULL_AREA = 0;

static const unsigned char RC_WALKABLE_AREA = 63;

namespace PolyAreaFlags
{
// extended behavior flags
enum: uint8_t
{
	TECHNICAL_LOW = RC_NULL_AREA,

	WATER_COMMON,
	WATER_SHALLOW, // walking as at the ground
	WATER_MIDDLE,  // fording
	WATER_DEEP,    // swimming
	LAVA,
	GROUND, // 6
	ROAD,
	FOREST,
	DOOR,
	LADDER, // 10

	INHABITED_AREA = 0x20,

	TECHNICAL_HIGH = RC_WALKABLE_AREA
};

static const unsigned int POLY_AREA_BITS = 0x3f;
static const unsigned int INHABITED_POS = 5;
static const unsigned int IS_TRI_POS = 7;

inline uint32_t clearInhabitedFlag(uint32_t val) { return val & ~INHABITED_AREA; }
inline uint32_t appendInhabitedFlag(uint32_t val) { return val | INHABITED_AREA; }
inline bool isInhabitedFlag(uint32_t val) { return val & INHABITED_AREA; }
inline uint32_t clearIsTriFlag(uint32_t val) { return val & ~(1 << IS_TRI_POS); }
inline uint32_t setIsTriFlag(uint32_t val) { return val | (1 << IS_TRI_POS); }
}

namespace NavmeshPolyTransferFlags
{
enum TransferType: uint32_t
{
	NO_ACTION,

	WALKING,
	JUMP_DOWN,
	JUMP_FORWARD,
	CLIMB,
	CLIMB_OVERLAPPED_POLYS,

	MAX_ACTION
};
}

namespace common
{

// constants and PODs
struct Constants
{
	// text, names
#ifdef WIN32
	static const char SEP = '\\';
#else
	static const char SEP = '/';
#endif
	static const int STR_SIZE = 4096;
	static const int NAME_SIZE = 256;
};

enum class LogCategory: int
{
	LOG_PROGRESS,
	LOG_WARNING,
	LOG_ERROR
};

struct NavmeshGenParams
{
	float cellSize = 0.f;
	float cellHeight = 0.f;
	float agentHeight = 0.f;
	float agentLiquidWalk = 0.f;
	float agentLiquidFord = 0.f;
	float agentLiquidSwim = 0.f;
	float agentRadius = 0.f;
	float agentMaxClimb = 0.f;
	float agentMaxSlope = 0.f;
	float regionMinSize = 0.f;
	float regionMergeSize = 0.f;
	float edgeMaxLen = 0.f;
	float edgeMaxError = 0.f;
	float vertsPerPoly = 0.f;
	float detailSampleDist = 0.f;
	float detailSampleMaxError = 0.f;
	int partitionType = 0;
	bool filterLowHangingObstacles = false;
	bool filterLedgeSpans = false;
	bool filterWalkableLowHeightSpans = false;
	bool erodeBorderSpans = false;
	float tileSize = 0.f;
};

// These are just sample areas to use consistent values across the samples.
// The use should specify these base on his needs.
struct  SamplePolyAreas
{
	enum {
		SAMPLE_POLYAREA_GROUND,
		SAMPLE_POLYAREA_ROAD,
		SAMPLE_POLYAREA_FOREST,
		SAMPLE_POLYAREA_DOOR,
		SAMPLE_POLYAREA_LADDER,
		SAMPLE_POLYAREA_WATER,
		SAMPLE_POLYAREA_WATER_WALKING,
		SAMPLE_POLYAREA_WATER_FORDING,
		SAMPLE_POLYAREA_WATER_SWIMMING,
		SAMPLE_POLYAREA_LAVA,

		SAMPLE_POLYAREA_MAX = 0x3f
	};
};

struct SamplePolyFlags
{
	enum {
		SAMPLE_POLYFLAGS_WALK = 0x01,		// Ability to walk (ground, grass, road)
		SAMPLE_POLYFLAGS_WATER_WALKING = 0x02,
		SAMPLE_POLYFLAGS_WATER_FORDING = 0x04,
		SAMPLE_POLYFLAGS_WATER_SWIMMING = 0x08,		// Ability to swim (water).
		SAMPLE_POLYFLAGS_ROAD = 0x10,		// Ability to move through doors.
		SAMPLE_POLYFLAGS_FOREST = 0x20,
		SAMPLE_POLYFLAGS_DOOR = 0x40,		// Ability to move through doors.
		SAMPLE_POLYFLAGS_LADDER = 0x80,
		SAMPLE_POLYFLAGS_INHABITED = 0x4000,
		SAMPLE_POLYFLAGS_DISABLED = 0x8000,	// Disabled polygon
		SAMPLE_POLYFLAGS_ALL = 0xffff     // All abilities.
	};
};

struct VobType
{
	enum: int
	{
		INVALID,
		SKIPPED,
		LADDER,
		DOOR,
		MOVER_UNIDIRECTION,
		MOVER_BIDIRECTION,
		OTHER
	};
};

// functions
template<class T> void ignoreUnused(const T&) {}

template<typename T>
T* allocAlignedImpl(size_t num, size_t alignment)
{
	void* p;
	if (p = _mm_malloc(sizeof(T) * num, alignment))
	{
		T* cur = nullptr;
		T* ptr = static_cast<T*>(p);
#ifdef CPP_EXCEPTIONS_ON
		std::exception_ptr ePtr;
		try {
#endif // CPP_EXCEPTIONS_ON
			for (cur = ptr; cur < ptr + num; ++cur)
			{
				new(cur) T();
			}
#ifdef CPP_EXCEPTIONS_ON
		}
		catch (...) {
			ePtr = std::current_exception();
		}
		if (ePtr) {
			for (T* s = ptr; s < cur; ++s)
			{
				try {
					s->~T();
				}
				catch (...) {
					std::get_terminate()();
				}
			}
			_mm_free(p);
			std::rethrow_exception(ePtr);
		}
#endif // CPP_EXCEPTIONS_ON
	}
	return static_cast<T*>(p);
}

template<typename T>
T* allocAligned(size_t alignment)
{
	return allocAlignedImpl<T>(1, alignment);
}

template<typename T>
T* allocAlignedArr(size_t num, size_t alignment)
{
	return allocAlignedImpl<T>(num, alignment);
}

template<typename T>
inline void freeAlignedImpl(T* p, size_t num)
{
	if (!p)
		return;
#ifdef CPP_EXCEPTIONS_ON
	std::exception_ptr ePtr;
#endif // CPP_EXCEPTIONS_ON
	T* cur = nullptr;
	T* ptr = static_cast<T*>(p);
	for (cur = ptr; cur < ptr + num; ++cur)
	{
#ifdef CPP_EXCEPTIONS_ON
		try {
#endif // CPP_EXCEPTIONS_ON
			cur->~T();
#ifdef CPP_EXCEPTIONS_ON
		}
		catch (...) {
			if (std::uncaught_exception() || ePtr)
			{
				std::get_terminate()();
			}
			ePtr = std::current_exception();
		}
#endif // CPP_EXCEPTIONS_ON
	}
	_mm_free(p);
#ifdef CPP_EXCEPTIONS_ON
	if (ePtr)
		std::rethrow_exception(ePtr);
#endif // CPP_EXCEPTIONS_ON
}

template<typename T>
inline void freeAligned(T* p)
{
	freeAlignedImpl<T>(p, 1);
}

template<typename T>
inline void freeAlignedArr(T* p, size_t num)
{
	freeAlignedImpl<T>(p, num);
}

// structs and classes
class BaseLogger
{
public:
	BaseLogger() = default;
	virtual ~BaseLogger() = default;

	void log(LogCategory category, const char* format, ...)
	{
		static const int MSG_SIZE = 1024;
		char msg[MSG_SIZE];
		va_list ap;
		va_start(ap, format);
		int len = vsnprintf(msg, MSG_SIZE, format, ap);
		if (len >= MSG_SIZE)
		{
			len = MSG_SIZE - 1;
			msg[MSG_SIZE - 1] = '\0';
		}
		va_end(ap);
		doLogMessage(category, msg, len);
	}

private:
	virtual void doLogMessage(LogCategory category, const char* msg, int len) = 0;
};

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
	LinearHashMultiStrToInt() : m_numBuckets(), m_loadFactor(), m_data() {}
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
		delete[] m_data;
		m_data = nullptr;
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
		size_t bid = m_hash(k, static_cast<int>(m_numBuckets));
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
		size_t bid = m_hash(k, static_cast<int>(m_numBuckets));
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

using HashMultiStrToInt = LinearHashMultiStrToInt<>;

template <typename T>
struct ArrayBuffer
{
	ArrayBuffer() = default;
	~ArrayBuffer() { release(); }
	void release() {
		delete[] data;
		data = nullptr;
		size = 0;
		num = 0;
	}

	int size = 0;
	int num = 0;
	T* data = nullptr;
};

template <typename F>
class ScopeExitImpl final {
public:
	ScopeExitImpl(F&& f) : m_f(std::forward<F>(f)) {}
	~ScopeExitImpl() noexcept(true) {
		try {
			if (m_active) m_f();
		}
		catch (...) {
			// TODO static compile check for noexception of 'f'
		}
	}
	void activate() const { m_active = true; }
	void deactivate() const { m_active = false; }

private:
	F m_f;
	mutable bool m_active = true;
};

struct NonCopyable
{
public:
	NonCopyable() = default;
	~NonCopyable() = default;
	NonCopyable(const NonCopyable&) = delete;
	NonCopyable& operator= (const NonCopyable&) = delete;
};

} // namespace common

#endif // COMMON_LIB_H