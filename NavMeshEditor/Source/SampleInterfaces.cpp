#define _USE_MATH_DEFINES

#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <assert.h>
#include "SampleInterfaces.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"
#include "PerfTimer.h"
#include "SDL.h"
#include "SDL_opengl.h"

#ifdef WIN32
	#define snprintf _snprintf
#else
	#define GL_GLEXT_PROTOTYPES
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////
static PFNGLBINDBUFFERPROC glBindBufferPtr;
static PFNGLGENBUFFERSPROC glGenBuffersPtr;
static PFNGLBUFFERDATAPROC glBufferDataPtr;
static PFNGLBUFFERSUBDATAPROC glBufferSubDataPtr;
static PFNGLDELETEBUFFERSPROC glDeleteBuffersPtr;

bool initOpenglMethods()
{
#ifdef WIN32
	if (!wglGetCurrentContext())
	{
		std::printf("Can't initialise opengl ptr methods. Opengl subsystem is not initialised");
		return false;
	}
	glBindBufferPtr = (PFNGLBINDBUFFERPROC)wglGetProcAddress("glBindBuffer");
	glGenBuffersPtr = (PFNGLGENBUFFERSPROC)wglGetProcAddress("glGenBuffers");
	glBufferDataPtr = (PFNGLBUFFERDATAPROC)wglGetProcAddress("glBufferData");
	glBufferSubDataPtr = (PFNGLBUFFERSUBDATAPROC)wglGetProcAddress("glBufferSubData");
	glDeleteBuffersPtr = (PFNGLDELETEBUFFERSPROC)wglGetProcAddress("glDeleteBuffers");
	if (!(
		glBindBufferPtr && glGenBuffersPtr && glBufferDataPtr && glBufferSubDataPtr && glDeleteBuffersPtr
	)) {
		std::printf(
			"Can't initialise opengl ptr methods. One of methods is not initialised, "
			"glBindBufferPtr: %d, glGenBuffersPtr: %d, glBufferDataPtr: %d, glBufferSubDataPtr: %d, glDeleteBuffersPtr: %d",
			(int)glBindBufferPtr, (int)glGenBuffersPtr, (int)glBufferDataPtr, (int)glBufferSubDataPtr, (int)glDeleteBuffersPtr
		);
		return false;
	}
#else
	glBindBufferPtr = &glBindBuffer;
	glGenBuffersPtr = &glGenBuffers;
	glBufferDataPtr = &glBufferData;
	glBufferSubDataPtr = &glBufferSubData;
	glDeleteBuffersPtr = &glDeleteBuffers;
#endif
	return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////

BuildContext::BuildContext() :
	m_messageCount(0),
	m_textPoolSize(0)
{
	memset(m_messages, 0, sizeof(char*) * MAX_MESSAGES);

	resetTimers();
}

// Virtual functions for custom implementations.
void BuildContext::doResetLog()
{
	m_messageCount = 0;
	m_textPoolSize = 0;
}

void BuildContext::doLog(const rcLogCategory category, const char* msg, const int len)
{
	if (!len) return;
	if (m_messageCount >= MAX_MESSAGES)
		return;

	std::lock_guard<std::mutex> lock(m_synch);
	char* dst = &m_textPool[m_textPoolSize];
	int n = TEXT_POOL_SIZE - m_textPoolSize;
	if (n < 2)
		return;
	char* cat = dst;
	char* text = dst+1;
	const int maxtext = n-1;
	// Store category
	*cat = (char)category;
	// Store message
	const int count = rcMin(len+1, maxtext);
	memcpy(text, msg, count);
	text[count-1] = '\0';
	m_textPoolSize += 1 + count;
	m_messages[m_messageCount++] = dst;
}

void BuildContext::doResetTimers()
{
	for (int i = 0; i < RC_MAX_TIMERS; ++i)
		m_accTime[i] = -1;
}

void BuildContext::doStartTimer(const rcTimerLabel label)
{
	m_startTime[label] = getPerfTime();
}

void BuildContext::doStopTimer(const rcTimerLabel label)
{
	const TimeVal endTime = getPerfTime();
	const TimeVal deltaTime = endTime - m_startTime[label];
	if (m_accTime[label] == -1)
		m_accTime[label] = deltaTime;
	else
		m_accTime[label] += deltaTime;
}

int BuildContext::doGetAccumulatedTime(const rcTimerLabel label) const
{
	return getPerfTimeUsec(m_accTime[label]);
}

void BuildContext::dumpLog(int startMsg, const char* format, ...)
{
	// Print header.
	va_list ap;
	va_start(ap, format);
	vprintf(format, ap);
	va_end(ap);
	printf("\n");
	
	// Print messages
	const int TAB_STOPS[4] = { 28, 36, 44, 52 };
	for (int i = rcMin(startMsg, m_messageCount); i < m_messageCount; ++i)
	{
		const char* msg = m_messages[i]+1;
		int n = 0;
		while (*msg)
		{
			if (*msg == '\t')
			{
				int count = 1;
				for (int j = 0; j < 4; ++j)
				{
					if (n < TAB_STOPS[j])
					{
						count = TAB_STOPS[j] - n;
						break;
					}
				}
				while (--count)
				{
					putchar(' ');
					n++;
				}
			}
			else
			{
				putchar(*msg);
				n++;
			}
			msg++;
		}
		putchar('\n');
	}
}

int BuildContext::getLogCount() const
{
	return m_messageCount;
}

const char* BuildContext::getLogText(const int i) const
{
	return m_messages[i]+1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

class GLCheckerTexture
{
	unsigned int m_texId;
public:
	GLCheckerTexture() : m_texId(0)
	{
	}
	
	~GLCheckerTexture()
	{
		if (m_texId != 0)
			glDeleteTextures(1, &m_texId);
	}
	void bind()
	{
		if (m_texId == 0)
		{
			// Create checker pattern.
			const unsigned int col0 = duRGBA(215,215,215,255);
			const unsigned int col1 = duRGBA(255,255,255,255);
			static const int TSIZE = 64;
			unsigned int data[TSIZE*TSIZE];
			
			glGenTextures(1, &m_texId);
			glBindTexture(GL_TEXTURE_2D, m_texId);

			int level = 0;
			int size = TSIZE;
			while (size > 0)
			{
				for (int y = 0; y < size; ++y)
					for (int x = 0; x < size; ++x)
						data[x+y*size] = (x==0 || y==0) ? col0 : col1;
				glTexImage2D(GL_TEXTURE_2D, level, GL_RGBA, size,size, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
				size /= 2;
				level++;
			}
			
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		}
		else
		{
			glBindTexture(GL_TEXTURE_2D, m_texId);
		}
	}
};
GLCheckerTexture g_tex;


void DebugDrawGL::depthMask(bool state)
{
	glDepthMask(state ? GL_TRUE : GL_FALSE);
}

void DebugDrawGL::texture(bool state)
{
	if (state)
	{
		glEnable(GL_TEXTURE_2D);
		g_tex.bind();
	}
	else
	{
		glDisable(GL_TEXTURE_2D);
	}
}

void DebugDrawGL::begin(duDebugDrawPrimitives prim, float size)
{
	switch (prim)
	{
		case DU_DRAW_POINTS:
			glPointSize(size);
			glBegin(GL_POINTS);
			break;
		case DU_DRAW_LINES:
			glLineWidth(size);
			glBegin(GL_LINES);
			break;
		case DU_DRAW_TRIS:
			glBegin(GL_TRIANGLES);
			break;
		case DU_DRAW_QUADS:
			glBegin(GL_QUADS);
			break;
        case DU_DRAW_NOT_INITED:
            assert(1 != 1);
            ;
	};
}

std::tuple<std::vector<float>*, std::vector<float>*, std::vector<unsigned int>*>
DataCollector::get_data()
{
    assert(m_curr_prim != DU_DRAW_NOT_INITED && m_curr_prim != DU_DRAW_QUADS);
    if (m_curr_prim == DU_DRAW_POINTS) {
        return std::make_tuple(
            &m_ret.m_vertices_points,
            &m_ret.m_textures_points,
            &m_ret.m_colors_points
        );
    }
    if (m_curr_prim == DU_DRAW_LINES) {
        return std::make_tuple(
            &m_ret.m_vertices_lines,
            &m_ret.m_textures_lines,
            &m_ret.m_colors_lines
        );
    }
    if (m_curr_prim == DU_DRAW_TRIS) {
        return std::make_tuple(
            &m_ret.m_vertices_tris,
            &m_ret.m_textures_tris,
            &m_ret.m_colors_tris
        );
    }
    return std::make_tuple(nullptr, nullptr, nullptr);
}

void DataCollector::vertex(const float* pos, unsigned int color)
{
    std::vector<float>* vertices{};
    //std::vector<float>* textures{};
    std::vector<unsigned int>* colors{};
    std::tie(vertices, std::ignore, colors) = get_data();
    vertices->push_back(pos[0]);
    vertices->push_back(pos[1]);
    vertices->push_back(pos[2]);
    colors->push_back(color);
}

void DataCollector::vertex(const float x, const float y, const float z, unsigned int color)
{
    std::vector<float>* vertices{};
    //std::vector<float>* textures{};
    std::vector<unsigned int>* colors{};
    std::tie(vertices, std::ignore, colors) = get_data();
    vertices->push_back(x);
    vertices->push_back(y);
    vertices->push_back(z);
    colors->push_back(color);
}

void DataCollector::vertex(const float* pos, unsigned int color, const float* uv)
{
    std::vector<float>* vertices{};
    std::vector<float>* textures{};
    std::vector<unsigned int>* colors{};
    std::tie(vertices, textures, colors) = get_data();
    vertices->push_back(pos[0]);
    vertices->push_back(pos[1]);
    vertices->push_back(pos[2]);
    textures->push_back(uv[0]);
    textures->push_back(uv[1]);
    colors->push_back(color);
}

void DebugDrawGL::vertex(const float* pos, unsigned int color)
{
	glColor4ubv((GLubyte*)&color);
	glVertex3fv(pos);
}

void DebugDrawGL::vertex(const float x, const float y, const float z, unsigned int color)
{
	glColor4ubv((GLubyte*)&color);
	glVertex3f(x,y,z);
}

void DebugDrawGL::vertex(const float* pos, unsigned int color, const float* uv)
{
	glColor4ubv((GLubyte*)&color);
	glTexCoord2fv(uv);
	glVertex3fv(pos);
}

void DebugDrawGL::draw_vbo_mesh(int vertices_number)
{
    glEnable(GL_TEXTURE_2D);
    g_tex.bind();

    glBindBufferPtr(GL_ARRAY_BUFFER, m_vbo_id_mesh);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(3, GL_FLOAT, 0, 0);
    glTexCoordPointer(2, GL_FLOAT, 0,
        (void*)((char*)nullptr + vertices_number * 3 * 4));
    glColorPointer(4, GL_UNSIGNED_BYTE, 0,
        (void*)((char*)nullptr + vertices_number * (3 * 4 + 2 * 4)));
    glDrawArrays(GL_TRIANGLES, 0, vertices_number);

    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    glLineWidth(1.0f);
    glPointSize(1.0f);
    glDisable(GL_TEXTURE_2D);
    glBindBufferPtr(GL_ARRAY_BUFFER, 0);
}

void DebugDrawGL::draw_vbo_navmesh(int ver_num_tris, int ver_num_lines, int ver_num_points)
{
    glDepthMask(GL_FALSE);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glBindBufferPtr(GL_ARRAY_BUFFER, m_vbo_id_navmesh_tris);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glColorPointer(4, GL_UNSIGNED_BYTE, 0,
        (void*)((char*)nullptr + ver_num_tris * (3 * 4)));
    glDrawArrays(GL_TRIANGLES, 0, ver_num_tris);
    //
    //glLineWidth(2.0);
    glBindBufferPtr(GL_ARRAY_BUFFER, m_vbo_id_navmesh_lines);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glColorPointer(4, GL_UNSIGNED_BYTE, 0,
        (void*)((char*)nullptr + ver_num_lines * (2 * 4)));
    glDrawArrays(GL_LINES, 0, ver_num_lines);
    //
    //glPointSize(2.0);
    glBindBufferPtr(GL_ARRAY_BUFFER, m_vbo_id_navmesh_points);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glColorPointer(4, GL_UNSIGNED_BYTE, 0,
        (void*)((char*)nullptr + ver_num_points * (1 * 4)));
    glDrawArrays(GL_POINTS, 0, ver_num_points);

    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    glDepthMask(GL_TRUE);
    glBindBufferPtr(GL_ARRAY_BUFFER, 0);
}

unsigned int DebugDrawGL::gen_vbo(
    const std::vector<float>& vertices,
    const std::vector<float>& textures,
    const std::vector<unsigned int>& colors
) {
    //Generate the VBO
    assert(vertices.size() / 3 == colors.size() / 1);
    assert(textures.empty() || vertices.size() / 3 == textures.size() / 2);

    size_t verticesSize = vertices.size() * sizeof(float);
    size_t texturesSize = textures.size() * sizeof(float);
    size_t colorsSize = colors.size() * sizeof(unsigned int);
    GLuint VBOID;
	glGenBuffersPtr(1, &VBOID);
	glBindBufferPtr(GL_ARRAY_BUFFER, VBOID);
	glBufferDataPtr(
        GL_ARRAY_BUFFER, verticesSize + texturesSize + colorsSize, 0, GL_STATIC_DRAW);
	glBufferSubDataPtr(GL_ARRAY_BUFFER, 0, verticesSize, vertices.data());
    if (!textures.empty())
		glBufferSubDataPtr(
            GL_ARRAY_BUFFER, verticesSize, texturesSize, textures.data()
        );
    glBufferSubDataPtr(
        GL_ARRAY_BUFFER, verticesSize + texturesSize, colorsSize, colors.data());
	glBindBufferPtr(GL_ARRAY_BUFFER, 0);
    return VBOID;
}

void DebugDrawGL::gen_vbo_mesh(
    const std::vector<float>& vertices,
    const std::vector<float>& textures,
    const std::vector<unsigned int>& colors
) {
    m_vbo_id_mesh = gen_vbo(vertices, textures, colors);
}

void DebugDrawGL::gen_vbo_navmesh(const EntryRet &dat) {
    m_vbo_id_navmesh_tris =
        gen_vbo(dat.m_vertices_tris, dat.m_textures_tris, dat.m_colors_tris);
    m_vbo_id_navmesh_lines =
        gen_vbo(dat.m_vertices_lines, dat.m_textures_lines, dat.m_colors_lines);
    m_vbo_id_navmesh_points =
        gen_vbo(dat.m_vertices_points, dat.m_textures_points, dat.m_colors_points);
}

bool DebugDrawGL::is_vbo_mesh_inited() const
{
    return m_vbo_id_mesh != static_cast<unsigned int>(-1);
}

bool DebugDrawGL::is_vbo_navmesh_inited() const
{
    return m_vbo_id_navmesh_tris != static_cast<unsigned int>(-1) &&
           m_vbo_id_navmesh_lines != static_cast<unsigned int>(-1) &&
           m_vbo_id_navmesh_points != static_cast<unsigned int>(-1);
}

void DebugDrawGL::reset_vbo()
{
    glDeleteBuffersPtr(1, &m_vbo_id_mesh);
    glDeleteBuffersPtr(1, &m_vbo_id_navmesh_tris);
    glDeleteBuffersPtr(1, &m_vbo_id_navmesh_lines);
    glDeleteBuffersPtr(1, &m_vbo_id_navmesh_points);
    m_vbo_id_mesh = -1;
    m_vbo_id_navmesh_tris = -1;
    m_vbo_id_navmesh_lines = -1;
    m_vbo_id_navmesh_points = -1;
}

void DataCollector::vertex(
    const float x, const float y, const float z, unsigned int color,
    const float u, const float v
) {
    std::vector<float>* vertices{};
    std::vector<float>* textures{};
    std::vector<unsigned int>* colors{};
    std::tie(vertices, textures, colors) = get_data();
    vertices->push_back(x);
    vertices->push_back(y);
    vertices->push_back(z);
    textures->push_back(u);
    textures->push_back(v);
    colors->push_back(color);
}

void DebugDrawGL::vertex(const float x, const float y, const float z, unsigned int color, const float u, const float v)
{
	glColor4ubv((GLubyte*)&color);
	glTexCoord2f(u,v);
	glVertex3f(x,y,z);
}

void DebugDrawGL::end()
{
	glEnd();
	glLineWidth(1.0f);
	glPointSize(1.0f);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

FileIO::FileIO() :
	m_fp(0),
	m_mode(-1)
{
}

FileIO::~FileIO()
{
	if (m_fp) fclose(m_fp);
}

bool FileIO::openForWrite(const char* path)
{
	if (m_fp) return false;
	m_fp = fopen(path, "wb");
	if (!m_fp) return false;
	m_mode = 1;
	return true;
}

bool FileIO::openForRead(const char* path)
{
	if (m_fp) return false;
	m_fp = fopen(path, "rb");
	if (!m_fp) return false;
	m_mode = 2;
	return true;
}

bool FileIO::isWriting() const
{
	return m_mode == 1;
}

bool FileIO::isReading() const
{
	return m_mode == 2;
}

bool FileIO::write(const void* ptr, const size_t size)
{
	if (!m_fp || m_mode != 1) return false;
	fwrite(ptr, size, 1, m_fp);
	return true;
}

bool FileIO::read(void* ptr, const size_t size)
{
	if (!m_fp || m_mode != 2) return false;
	size_t readLen = fread(ptr, size, 1, m_fp);
	return readLen == 1;
}


