#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <cstdint>
#include <utility>
#include <cstdlib>
#ifdef WIN32
#include <malloc.h>
#else
#include <mm_malloc.h>
#endif // WIN32
#include <exception>

/// Represents the null area.
/// When a data element is given this value it is considered to no longer be 
/// assigned to a usable area.  (E.g. It is unwalkable.)
static const unsigned char RC_NULL_AREA = 0;

/// The default area id used to indicate a walkable polygon. 
/// This is also the maximum allowed area id, and the only non-null area id 
/// recognized by some steps in the build process. 
static const unsigned char RC_WALKABLE_AREA = 63;

namespace PolyAreaFlags
{
	static const unsigned int POLY_AREA_BITS = 0x3f;
	static const unsigned int INHABITED_POS = 5;
	static const unsigned int IS_TRI_POS = 7;

	struct FlagType
	{
		uint32_t isTriangle : 1;
		uint32_t isVobPos : 1;
		uint32_t isActiveVobPos : 1;
		uint32_t reserved : 3;
		uint32_t vobIdOrCollFlags : 20;
		uint32_t isInhabited : 1;
		uint32_t polyFlags : 5;
	};

	// extended behavior flags
	enum : uint8_t
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

	inline uint32_t clearInhabitedFlag(uint32_t val) { return val & ~INHABITED_AREA; }
	inline uint32_t appendInhabitedFlag(uint32_t val) { return val | INHABITED_AREA; }
	inline bool isInhabitedFlag(uint32_t val) { return val & INHABITED_AREA; }
	inline uint32_t clearIsTriFlag(uint32_t val) { return val & ~(1 << IS_TRI_POS); }
	inline uint32_t setIsTriFlag(uint32_t val) { return val | (1 << IS_TRI_POS); }
}

namespace PolyFlagsCollision
{
	enum
	{
		WATER = 1,
		SOLID = 2,
		LAVA = 4
	};
}


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


inline float rcVdotXz(const float* v1, const float* v2)
{
	return v1[0] * v2[0] + v1[2] * v2[2];
}

#endif // COMMON_LIB_H