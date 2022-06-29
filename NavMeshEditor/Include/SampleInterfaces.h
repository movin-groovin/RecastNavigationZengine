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

bool initOpenglMethods();

// These are example implementations of various interfaces used in Recast and Detour.

/// Recast build context.
class BuildContext : public rcContext
{
	TimeVal m_startTime[RC_MAX_TIMERS];
	TimeVal m_accTime[RC_MAX_TIMERS];

	static const int MAX_MESSAGES = 16 * 1024;//1000;
	const char* m_messages[MAX_MESSAGES];
	int m_messageCount;
	static const int TEXT_POOL_SIZE = 1024 * 1024;//8000;
	char m_textPool[TEXT_POOL_SIZE];
	int m_textPoolSize;

	std::mutex m_synch;
	
public:
	BuildContext();
	
	/// Dumps the log to stdout.
	void dumpLog(int startMsg, const char* format, ...);
	/// Returns number of log messages.
	int getLogCount() const;
	/// Returns log message text.
	const char* getLogText(const int i) const;
	
protected:	
	/// Virtual functions for custom implementations.
	///@{
	virtual void doResetLog();
	virtual void doLog(const rcLogCategory category, const char* msg, const int len);
	virtual void doResetTimers();
	virtual void doStartTimer(const rcTimerLabel label);
	virtual void doStopTimer(const rcTimerLabel label);
	virtual int doGetAccumulatedTime(const rcTimerLabel label) const;
	///@}
};

/// OpenGL debug draw implementation.
class DebugDrawGL : public duDebugDraw
{
private:
    unsigned int m_vbo_id_mesh = -1;
    unsigned int m_vbo_id_navmesh_tris = -1;
    unsigned int m_vbo_id_navmesh_lines = -1;
    unsigned int m_vbo_id_navmesh_points = -1;

public:
	virtual void depthMask(bool state);
	virtual void texture(bool state);
	virtual void begin(duDebugDrawPrimitives prim, float size = 1.0f);
	virtual void vertex(const float* pos, unsigned int color);
	virtual void vertex(const float x, const float y, const float z, unsigned int color);
	virtual void vertex(const float* pos, unsigned int color, const float* uv);
	virtual void vertex(const float x, const float y, const float z, unsigned int color, const float u, const float v);
	virtual void end();

    virtual void draw_vbo_mesh(int vertices_number);
    virtual void draw_vbo_navmesh(int ver_num_tris, int ver_num_lines, int ver_num_points);
    virtual void gen_vbo_mesh(
        const std::vector<float>& vertices,
        const std::vector<float>& textures,
        const std::vector<unsigned int>& colors
    );
    virtual void gen_vbo_navmesh(const EntryRet& dat);
    virtual bool is_vbo_mesh_inited() const;
    virtual bool is_vbo_navmesh_inited() const;
    virtual void reset_vbo();

    virtual EntryRet fetch_collected_data() {assert(1 != 1); return {};}

private:
    unsigned int gen_vbo(
        const std::vector<float>& vertices,
        const std::vector<float>& textures,
        const std::vector<unsigned int>& colors
    );
};

// rendering data collector
class DataCollector : public duDebugDraw
{
private:
    duDebugDrawPrimitives m_curr_prim = DU_DRAW_NOT_INITED;
    EntryRet m_ret;
    unsigned int(*funcAreatToCol)(unsigned int);

private:
    std::tuple<std::vector<float>*, std::vector<float>*, std::vector<unsigned int>*>
    get_data();

public:
    DataCollector(unsigned int(*func)(unsigned int)) { funcAreatToCol = func; }
    virtual EntryRet fetch_collected_data() { return std::move(m_ret); }

    virtual void depthMask(bool) {}
    virtual void texture(bool) {}
    virtual void begin(duDebugDrawPrimitives prim, float = 1.0f) {m_curr_prim = prim;}
    virtual void vertex(const float* pos, unsigned int color);
    virtual void vertex(const float x, const float y, const float z, unsigned int color);
    virtual void vertex(const float* pos, unsigned int color, const float* uv);
    virtual void vertex(
        const float x, const float y, const float z,
        unsigned int color, const float u, const float v
    );
    virtual void end() {m_curr_prim = DU_DRAW_NOT_INITED;}

    virtual void draw_vbo_mesh(int) {assert(1 != 1);}
    virtual void draw_vbo_navmesh(int, int, int) {assert(1 != 1);}
    virtual void gen_vbo_mesh(
        const std::vector<float>&,
        const std::vector<float>&,
        const std::vector<unsigned int>&
    ) {assert(1 != 1);}
    virtual void gen_vbo_navmesh(const EntryRet&) {assert(1 != 1);}
    virtual bool is_vbo_mesh_inited() const {assert(1 != 1); return {};}
    virtual bool is_vbo_navmesh_inited() const {assert(1 != 1); return {};}
    virtual void reset_vbo() {assert(1 != 1);}

    virtual unsigned int areaToCol(unsigned int area)
    {
        return funcAreatToCol(area);
    }
};

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

