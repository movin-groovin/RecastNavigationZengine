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
#include <stdexcept>
#include <sstream>

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
	auto fptrToStr = [] (const void* p) {
		return p ? "inited" : "not inited";
	};
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
			"glBindBufferPtr: %s, glGenBuffersPtr: %s, glBufferDataPtr: %s, glBufferSubDataPtr: %s, glDeleteBuffersPtr: %s",
			fptrToStr(glBindBufferPtr), fptrToStr(glGenBuffersPtr), fptrToStr(glBufferDataPtr), fptrToStr(glBufferSubDataPtr),
			fptrToStr(glDeleteBuffersPtr)
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

BuildContext::BuildContext(int messageSize, int textPoolSize) :
	m_messages(std::make_unique<const char* []>(messageSize)),
	m_messageSize(messageSize),
	m_messageCount(0),
	m_textPool(std::make_unique<char[]>(textPoolSize)),
	m_textPoolSize(textPoolSize),
	m_textPoolCount(0)
{
	memset(m_messages.get(), 0, sizeof(char*) * m_messageSize);
	resetTimers();
}

// Virtual functions for custom implementations.
void BuildContext::doResetLog()
{
	m_messageCount = 0;
	m_textPoolCount = 0;
}

void BuildContext::doLog(const rcLogCategory category, const char* msg, const int len)
{
	if (!len) return;
	if (m_messageCount >= MAX_MESSAGES)
		return;

	std::lock_guard<std::mutex> lock(m_synch);
	char* dst = &m_textPool[m_textPoolCount];
	int n = m_textPoolSize - m_textPoolCount;
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
	m_textPoolCount += 1 + count;
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

#ifdef ZENGINE_NAVMESH
VboDataEntry* RenderingDataCollector::getData(bool hasTextures)
{
    assert(m_currPrim != DU_DRAW_NOT_INITED);

	if (m_currPrim == DU_DRAW_QUADS) {
		return &m_quads;
	}
	if (m_currPrim == DU_DRAW_TRIS) {
		if (hasTextures)
			return &m_trisWithTextures;
		return &m_tris;
	}
	if (m_currPrim == DU_DRAW_LINES) {
		return &m_lines;
	}
	if (m_currPrim == DU_DRAW_POINTS) {
		return &m_points;
	}

	return nullptr;
}

void RenderingDataCollector::depthMask(bool v)
{
	//glDepthMask(v ? GL_TRUE : GL_FALSE);
}

void RenderingDataCollector::texture(bool v)
{
	//if (v)
	//{
	//	glEnable(GL_TEXTURE_2D);
	//	g_tex.bind();
	//}
	//else
	//{
	//	glDisable(GL_TEXTURE_2D);
	//}
}

void RenderingDataCollector::begin(duDebugDrawPrimitives prim, float)
{
	m_currPrim = prim;
}

void RenderingDataCollector::vertex(const float* pos, unsigned int color)
{
	VboDataEntry& ret = *getData();
	ret.adjustSize(false);
    ret.vertices[ret.size * 3] = pos[0];
	ret.vertices[ret.size * 3 + 1] = pos[1];
	ret.vertices[ret.size * 3 + 2] = pos[2];
	ret.colors[ret.size] = color;
	++ret.size;
}

void RenderingDataCollector::vertex(const float x, const float y, const float z, unsigned int color)
{
	VboDataEntry& ret = *getData();
	ret.adjustSize(false);
	ret.vertices[ret.size * 3] = x;
	ret.vertices[ret.size * 3 + 1] = y;
	ret.vertices[ret.size * 3 + 2] = z;
	ret.colors[ret.size] = color;
	++ret.size;
}

void RenderingDataCollector::vertex(const float* pos, unsigned int color, const float* uv)
{
	assert(m_currPrim == DU_DRAW_TRIS);
	VboDataEntry& ret = *getData(true);
	ret.adjustSize(true);
	ret.vertices[ret.size * 3] = pos[0];
	ret.vertices[ret.size * 3 + 1] = pos[1];
	ret.vertices[ret.size * 3 + 2] = pos[2];
	ret.textures[ret.size * 2] = uv[0];
	ret.textures[ret.size * 2 + 1] = uv[1];
	ret.colors[ret.size] = color;
	++ret.size;
}

void RenderingDataCollector::vertex(
	const float x,
	const float y,
	const float z,
	unsigned int color,
	const float u,
	const float v
) {
	assert(m_currPrim == DU_DRAW_TRIS);
	VboDataEntry& ret = *getData(true);
	ret.adjustSize(true);
	ret.vertices[ret.size * 3] = x;
	ret.vertices[ret.size * 3 + 1] = y;
	ret.vertices[ret.size * 3 + 2] = z;
	ret.textures[ret.size * 2] = u;
	ret.textures[ret.size * 2 + 1] = v;
	ret.colors[ret.size] = color;
	++ret.size;
}

void RenderingDataCollector::reset()
{
	m_quads.reset();
	m_tris.reset();
	m_trisWithTextures.reset();
	m_lines.reset();
	m_points.reset();
}

void RenderingDataCollector::end()
{
	m_currPrim = DU_DRAW_NOT_INITED;
}

unsigned int RenderingDataCollector::areaToCol(unsigned int area)
{
	return funcAreatToCol(area);
}

void VboDebugDraw::depthMask(bool v)
{
	m_collector.depthMask(v);
}

void VboDebugDraw::texture(bool v)
{
	m_collector.texture(v);
}

void VboDebugDraw::begin(duDebugDrawPrimitives prim, float v)
{
	m_collector.begin(prim, v);
}

void VboDebugDraw::vertex(const float* pos, unsigned int color)
{
	m_collector.vertex(pos, color);
}

void VboDebugDraw::vertex(const float x, const float y, const float z, unsigned int color)
{
	m_collector.vertex(x, y, z, color);
}

void VboDebugDraw::vertex(const float* pos, unsigned int color, const float* uv)
{
	m_collector.vertex(pos, color, uv);
}

void VboDebugDraw::vertex(
	const float x,
	const float y,
	const float z,
	unsigned int color,
	const float u,
	const float v
) {
	m_collector.vertex(x, y, z, color, u, v);
}

void VboDebugDraw::end()
{
	m_collector.end();
}

unsigned int VboDebugDraw::areaToCol(unsigned int area)
{
	return m_collector.areaToCol(area);
}

void VboDebugDraw::drawVbo(int primitive, unsigned int vboId, std::size_t vertsNum, bool withTextures)
{
	if (!vboId)
		return;
	
	if (withTextures) {
		glEnable(GL_TEXTURE_2D);
		g_tex.bind();
	}

	glBindBufferPtr(GL_ARRAY_BUFFER, vboId);
	checkGlError(__LINE__);
	glEnableClientState(GL_VERTEX_ARRAY);
	checkGlError(__LINE__);
	if (withTextures)
	{
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
		checkGlError(__LINE__);
	}
	glEnableClientState(GL_COLOR_ARRAY);
	checkGlError(__LINE__);

	glVertexPointer(3, GL_FLOAT, /*3 * sizeof(float)*/0, 0);
	checkGlError(__LINE__);
	if (withTextures) {
		glTexCoordPointer(2, GL_FLOAT, /*2 * sizeof(float)*/0,
			(void*)((char*)nullptr + vertsNum * 3 * sizeof(float)));
		checkGlError(__LINE__);
		glColorPointer(4, GL_UNSIGNED_BYTE, 0,
			(void*)((char*)nullptr + vertsNum * (3 * sizeof(float) + 2 * sizeof(float))));
		checkGlError(__LINE__);
	}
	else {
		glColorPointer(4, GL_UNSIGNED_BYTE, /*sizeof(unsigned int)*/0,
			(void*)((char*)nullptr + vertsNum * 3 * sizeof(float)));
		checkGlError(__LINE__);
	}
	glDrawArrays(primitive, 0, static_cast<int>(vertsNum));
	checkGlError(__LINE__);

	glDisableClientState(GL_COLOR_ARRAY);
	checkGlError(__LINE__);
	if (withTextures)
	{
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		checkGlError(__LINE__);
	}
	glDisableClientState(GL_VERTEX_ARRAY);
	checkGlError(__LINE__);

	//glLineWidth(1.0f);
	//glPointSize(1.0f);
	if (withTextures)
		glDisable(GL_TEXTURE_2D);
	glBindBufferPtr(GL_ARRAY_BUFFER, 0);
}

GLuint VboDebugDraw::genVbo(
	std::size_t size,
	const std::unique_ptr<float[]>& vertices,
	const std::unique_ptr<float[]>& textures,
	const std::unique_ptr<unsigned int[]>& colors
) {
	if (!size)
		return 0;
	
	std::size_t verticesSize = size * 3 * sizeof(float);
	std::size_t texturesSize = textures ? size * 2 * sizeof(float) : 0;
	std::size_t colorsSize = size * sizeof(unsigned int);
	GLuint VBOID;
	glGenBuffersPtr(1, &VBOID);
	checkGlError(__LINE__);
	glBindBufferPtr(GL_ARRAY_BUFFER, VBOID);
	checkGlError(__LINE__);
	glBufferDataPtr(
		GL_ARRAY_BUFFER, verticesSize + texturesSize + colorsSize, 0, GL_STATIC_DRAW);
	checkGlError(__LINE__);
	glBufferSubDataPtr(GL_ARRAY_BUFFER, 0, verticesSize, vertices.get());
	checkGlError(__LINE__);
	if (texturesSize)
	{
		glBufferSubDataPtr(
			GL_ARRAY_BUFFER, verticesSize, texturesSize, textures.get()
		);
		checkGlError(__LINE__);
	}
	glBufferSubDataPtr(
		GL_ARRAY_BUFFER, verticesSize + texturesSize, colorsSize, colors.get());
	checkGlError(__LINE__);
	glBindBufferPtr(GL_ARRAY_BUFFER, 0);
	return VBOID;
}

void VboDebugDraw::freeVbo(unsigned int vboId)
{
	glDeleteBuffersPtr(1, &vboId);
}

void VboDebugDraw::checkGlError(std::size_t nLine)
{
	GLenum err;
	if ((err = glGetError()) != GL_NO_ERROR) {
		std::ostringstream oss;
		oss << "Opengl err code: " << err << ", code line: " << nLine;
		throw std::runtime_error(oss.str());
	}
}

void VboDebugDraw::draw() const
{
	if (m_reseted)
		generate();
	const VboDataEntry& quads = m_collector.getQuads();
	drawVbo(GL_QUADS, m_vboIdQuads, quads.size, false);
	const VboDataEntry& tris = m_collector.getTris();
	drawVbo(GL_TRIANGLES, m_vboIdTris, tris.size, false);
	const VboDataEntry& trisTexts = m_collector.getTrisWithTextures();
	drawVbo(GL_TRIANGLES, m_vboIdTrisWithTextures, trisTexts.size, true);
	const VboDataEntry& lines = m_collector.getLines();
	drawVbo(GL_LINES, m_vboIdLines, lines.size, false);
	const VboDataEntry& points = m_collector.getPoints();
	drawVbo(GL_POINTS, m_vboIdPoints, points.size, false);
}

void VboDebugDraw::generate() const
{
	const VboDataEntry& quads = m_collector.getQuads();
	m_vboIdQuads = genVbo(quads.size, quads.vertices, quads.textures, quads.colors);
	const VboDataEntry& tris = m_collector.getTris();
	m_vboIdTris = genVbo(tris.size, tris.vertices, tris.textures, tris.colors);
	const VboDataEntry& trisTexts = m_collector.getTrisWithTextures();
	m_vboIdTrisWithTextures =
		genVbo(trisTexts.size, trisTexts.vertices, trisTexts.textures, trisTexts.colors);
	const VboDataEntry& lines = m_collector.getLines();
	m_vboIdLines = genVbo(lines.size, lines.vertices, lines.textures, lines.colors);
	const VboDataEntry& points = m_collector.getPoints();
	m_vboIdPoints = genVbo(points.size, points.vertices, points.textures, points.colors);
	m_reseted = false;
}

void VboDebugDraw::reset()
{
	m_collector.reset();
	if (m_vboIdQuads)
		freeVbo(m_vboIdQuads);
	if (m_vboIdTris)
		freeVbo(m_vboIdTris);
	if (m_vboIdTrisWithTextures)
		freeVbo(m_vboIdTrisWithTextures);
	if (m_vboIdLines)
		freeVbo(m_vboIdLines);
	if (m_vboIdPoints)
		freeVbo(m_vboIdPoints);
	m_vboIdQuads = 0;
	m_vboIdTris = 0;
	m_vboIdTrisWithTextures = 0;
	m_vboIdLines = 0;
	m_vboIdPoints = 0;
	m_reseted = true;
}

VboDebugDraw::operator bool() const
{
	return !m_reseted;
}
#endif // ZENGINE_NAVMESH

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


