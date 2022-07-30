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

#ifndef SAMPLEINTERFACES_H
#define SAMPLEINTERFACES_H

#include "DebugDraw.h"
#include "Recast.h"
#include "RecastDump.h"
#include "PerfTimer.h"

#include <vector>
#include <tuple>
#include <cstdint>
#include <stdexcept>
#include <cassert>
#include <utility>
#include <mutex>
#include <memory>

#include "SDL_opengl.h"

bool initOpenglMethods();

// These are example implementations of various interfaces used in Recast and Detour.

/// Recast build context.
class BuildContext : public rcContext
{
protected:
	static const int MAX_MESSAGES = 256 * 1024;
	static const int TEXT_POOL_SIZE = 32 * 1024 * 1024;

private:
	TimeVal m_startTime[RC_MAX_TIMERS];
	TimeVal m_accTime[RC_MAX_TIMERS];

	std::unique_ptr<const char*[]> m_messages;
	int m_messageSize;
	int m_messageCount;
	std::unique_ptr<char[]> m_textPool;
	int m_textPoolSize;
	int m_textPoolCount;

	std::mutex m_synch;
	
public:
	BuildContext(int messageSize = MAX_MESSAGES, int textPoolSize = TEXT_POOL_SIZE);
	
	/// Dumps the log to stdout.
	void dumpLog(int startMsg, const char* format, ...);
	/// Returns number of log messages.
	int getLogCount() const;
	/// Returns log message text.
	const char* getLogText(const int i) const;
	virtual void doLog(const rcLogCategory category, const char* msg, const int len);
	
protected:	
	/// Virtual functions for custom implementations.
	///@{
	virtual void doResetLog();
	virtual void doResetTimers();
	virtual void doStartTimer(const rcTimerLabel label);
	virtual void doStopTimer(const rcTimerLabel label);
	virtual int doGetAccumulatedTime(const rcTimerLabel label) const;
	///@}
};

/// OpenGL debug draw implementation.
class DebugDrawGL : public duDebugDraw
{
public:
	virtual void depthMask(bool state);
	virtual void texture(bool state);
	virtual void begin(duDebugDrawPrimitives prim, float size);
	virtual void vertex(const float* pos, unsigned int color);
	virtual void vertex(const float x, const float y, const float z, unsigned int color);
	virtual void vertex(const float* pos, unsigned int color, const float* uv);
	virtual void vertex(const float x, const float y, const float z, unsigned int color, const float u, const float v);
	virtual void end();
};

#ifdef ZENGINE_NAVMESH
struct VboDataEntry final
{
	static const std::size_t MEM_ADD = 2 * 1024 * 1024;
	std::size_t size = 0;
	std::size_t dataSize = 0;
	// per entry: 3 floats of vertex, 2 floats of texture, 1 uint of color
	std::unique_ptr<float[]> vertices;
	std::unique_ptr<float[]> textures;
	std::unique_ptr<unsigned int[]> colors;

	void adjustSize(bool resTextures)
	{
		if (size == dataSize) {
			dataSize += MEM_ADD;
			std::unique_ptr<float[]> tmpVertices = std::make_unique<float[]>(dataSize * 3);
			std::unique_ptr<unsigned int[]> tmpColors = std::make_unique<unsigned int[]>(dataSize);
			std::memcpy(tmpVertices.get(), vertices.get(), 3 * sizeof(float) * size);
			std::memcpy(tmpColors.get(), colors.get(), sizeof(unsigned int) * size);
			vertices = std::move(tmpVertices);
			colors = std::move(tmpColors);
			if (resTextures) {
				std::unique_ptr<float[]> tmpTextures = std::make_unique<float[]>(dataSize * 2);
				std::memcpy(tmpTextures.get(), textures.get(), 2 * sizeof(float) * size);
				textures = std::move(tmpTextures);
			}
		}
	}
	void reset() { size = 0; }
};

class RenderingDataCollector: public duDebugDraw
{
private:
    duDebugDrawPrimitives m_currPrim = DU_DRAW_NOT_INITED;
    unsigned int(*funcAreatToCol)(unsigned int);
	VboDataEntry m_quads;
	VboDataEntry m_tris;
	VboDataEntry m_trisWithTextures;
	VboDataEntry m_lines;
	VboDataEntry m_points;
private:
	VboDataEntry* getData(bool hasTextures = false);

public:
	explicit RenderingDataCollector(unsigned int(*func)(unsigned int)) { funcAreatToCol = func; }
    
	const VboDataEntry& getQuads() const { return m_quads; }
	const VboDataEntry& getTris() const { return m_tris; }
	const VboDataEntry& getTrisWithTextures() const { return m_trisWithTextures; }
	const VboDataEntry& getLines() const { return m_lines; }
	const VboDataEntry& getPoints() const { return m_points; }

	void depthMask(bool v) override;
	void texture(bool v) override;
	void begin(duDebugDrawPrimitives prim, float) override;
    void vertex(const float* pos, unsigned int color) override;
    void vertex(const float x, const float y, const float z, unsigned int color) override;
    void vertex(const float* pos, unsigned int color, const float* uv) override;
    void vertex(
        const float x,
		const float y,
		const float z,
        unsigned int color,
		const float u,
		const float v
    ) override;
	void end() override;
	unsigned int areaToCol(unsigned int area) override;

	void reset();
};

class VboDebugDraw: public duDebugDraw
{
private:
	RenderingDataCollector m_collector;
	mutable unsigned int m_vboIdQuads = 0;
	mutable unsigned int m_vboIdTris = 0;
	mutable unsigned int m_vboIdTrisWithTextures = 0;
	mutable unsigned int m_vboIdLines = 0;
	mutable unsigned int m_vboIdPoints = 0;
	mutable bool m_reseted = true;

public:
	explicit VboDebugDraw(unsigned int(*func)(unsigned int)): m_collector(func) {}

	void depthMask(bool v) override;
	void texture(bool v) override;
	void begin(duDebugDrawPrimitives prim, float v) override;
	void vertex(const float* pos, unsigned int color) override;
	void vertex(const float x, const float y, const float z, unsigned int color) override;
	void vertex(const float* pos, unsigned int color, const float* uv) override;
	void vertex(
		const float x,
		const float y,
		const float z,
		unsigned int color,
		const float u,
		const float v
	) override;
	void end() override;
	unsigned int areaToCol(unsigned int area) override;

	void draw() const;
	void generate() const;
	void reset();
	operator bool() const;

private:
	static GLuint genVbo(
		std::size_t size,
		const std::unique_ptr<float[]>& vertices,
		const std::unique_ptr<float[]>& textures,
		const std::unique_ptr<unsigned int[]>& colors
	);
	static void drawVbo(
		int primitive, unsigned int vboId, std::size_t vertsNum, bool withTextures
	);
	static void freeVbo(unsigned int vboId);
	static void checkGlError(std::size_t nLine);
};
#endif // ZENGINE_NAVMESH

/// stdio file implementation.
class FileIO : public duFileIO
{
	FILE* m_fp;
	int m_mode;
public:
	FileIO();
	virtual ~FileIO();
	bool openForWrite(const char* path);
	bool openForRead(const char* path);
	virtual bool isWriting() const;
	virtual bool isReading() const;
	virtual bool write(const void* ptr, const size_t size);
	virtual bool read(void* ptr, const size_t size);
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	FileIO(const FileIO&);
	FileIO& operator=(const FileIO&);
};

#endif // SAMPLEINTERFACES_H

