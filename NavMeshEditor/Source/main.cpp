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

#include <cstdio>
#define _USE_MATH_DEFINES
#include <cmath>

#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif
#ifdef WIN32
	#include <winuser.h>
#endif

#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstring>
#include <exception>

#include "imgui.h"
#include "imguiRenderGL.h"

#include "Recast.h"
#include "RecastDebugDraw.h"
#include "InputGeom.h"
#include "TestCase.h"
#include "Filelist.h"
#include "Sample_TileMesh.h"
#include "Sample_Debug.h"
#include "NavMeshTesterTool.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "SampleInterfaces.h"
#include "Common.h"

#ifdef WIN32
	#define snprintf _snprintf
	#define putenv _putenv
#endif

using std::string;
using std::vector;

struct SampleItem
{
	Sample* (*create)();
	const string name;
};
//Sample* createSolo() { return new Sample_SoloMesh(); }
Sample* createTile() { return new Sample_TileMesh(); }
//Sample* createTempObstacle() { return new Sample_TempObstacles(); }
Sample* createDebug() { return new Sample_Debug(); }
static SampleItem g_samples[] =
{
	//{ createSolo, "Solo Mesh" },
	{ createTile, "Tile Mesh" }//,
	//{ createTempObstacle, "Temp Obstacles" },
};
static const int g_nsamples = sizeof(g_samples) / sizeof(SampleItem);

/*
namespace {

static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader
{
    int magic;
    int version;
    int numTiles;
    dtNavMeshParams params;
};

struct NavMeshTileHeader
{
    dtTileRef tileRef;
    int dataSize;
};

static dtNavMesh* loadAll(const char* path)
{
    FILE* fp = fopen(path, "rb");
    if (!fp) return 0;

    // Read header.
    NavMeshSetHeader header;
    size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
    if (readLen != 1)
    {
        fclose(fp);
        return 0;
    }
    if (header.magic != NAVMESHSET_MAGIC)
    {
        fclose(fp);
        return 0;
    }
    if (header.version != NAVMESHSET_VERSION)
    {
        fclose(fp);
        return 0;
    }

    dtNavMesh* mesh = dtAllocNavMesh();
    if (!mesh)
    {
        fclose(fp);
        return 0;
    }
    dtStatus status = mesh->init(&header.params);
    if (dtStatusFailed(status))
    {
        fclose(fp);
        return 0;
    }

    // Read tiles.
    for (int i = 0; i < header.numTiles; ++i)
    {
        NavMeshTileHeader tileHeader;
        readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
        if (readLen != 1)
        {
            fclose(fp);
            return 0;
        }

        if (!tileHeader.tileRef || !tileHeader.dataSize)
            break;

        unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
        if (!data) break;
        memset(data, 0, tileHeader.dataSize);
        readLen = fread(data, tileHeader.dataSize, 1, fp);
        if (readLen != 1)
        {
            dtFree(data);
            fclose(fp);
            return 0;
        }

        mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
    }

    fclose(fp);

    return mesh;
}

}

static void runProfile() {
    rcIgnoreUnused(&loadAll);
    const char* meshName = "/home/bookman/sources/g2/recast/"
                       "recastnavigation/RecastDemo/Bin/Meshes/irdorat_total_mesh.obj";
    const char* navMeshName = "/home/bookman/sources/g2/recast/recastnavigation/RecastDemo/"
                              "Bin/irdorat_total_mesh.bin";
    InputGeom geom;
    if (!geom.load(0, meshName)) {
        std::printf("Error of InputGeom load\n");
        return;
    }
    //const Grid2dBvh& space = geom.getSpace();
    NavMeshTesterTool testerTool(&geom, nullptr);
    dtNavMesh* navMesh = loadAll(navMeshName);
    if (!navMesh) {
        std::printf("Error of Sample::loadAll\n");
        return;
    }
    dtNavMeshQuery navQuery;
    navQuery.init(navMesh, 8 * 1024);
    testerTool.init(navMesh, &navQuery);
    constexpr int k = 18;
    const int n = 500000;//2 * 999 * 1000 * 1000; // 18
    int i = 0;
    float t = 0;
    float start[3 * k] = {
//        -9432.506836, 392.428955, 16677.597656,
//        -9432.506836, 392.428955, 16677.597656,
//        -9432.506836, 392.428955, 16677.597656
//
        -9467.645508, 392.438721, 16670.212891,
        -9467.645508, 392.438721, 16670.212891,
        -9467.645508, 392.438721, 16670.212891,
        -9467.645508, 392.438721, 16670.212891,
        -9467.645508, 392.438721, 16670.212891,
        -9467.645508, 392.438721, 16670.212891,
        -19364.687500, 1575.120605, 13027.791016,
        -18721.492188, 1253.816162, 8278.615234,
        -18721.492188, 1253.816162, 8278.615234,
        -21333.683594, 1403.324707, 4139.777832,
        -21333.683594, 1403.324707, 4139.777832,
        -25215.046875, 2207.533691, -1500.194580,
        -25215.046875, 2207.533691, -1500.194580,
        -25215.046875, 2207.533691, -1500.194580,
        -23497.128906, 2153.987305, -12986.458008,
        -18463.462891, 2259.595703, -17441.173828,
        -18463.462891, 2259.595703, -17441.173828,
        -18463.462891, 2259.595703, -17441.173828
    };
    float end[3 * k] = {
//        -9440.590820, -112.956299, 15120.764648,
//        -9392.461914, -112.967041, 15141.758789,
//        -9364.313477, -112.973145, 15148.122070
//
        -9483.497070, -112.945068, 15118.356445,
        -9727.037109, -112.940674, 13190.312500,
        -9455.690430, 462.890869, 12195.484375,
        -9875.134766, 543.274414, 11077.719727,
        -15379.892578, 336.661621, 16988.339844,
        -19800.949219, 1798.300781, 12715.777344,
        -19800.949219, 1798.300781, 12715.777344,
        -19800.949219, 1798.300781, 12715.777344,
        -21997.527344, 1447.350586, 5143.147949,
        -21997.527344, 1447.350586, 5143.147949,
        -24870.828125, 1887.185547, 1434.722656,
        -24870.828125, 1887.185547, 1434.722656,
        -18777.638672, 2109.167480, -8428.564453,
        -23430.123047, 2167.256836, -10337.596680,
        -23430.123047, 2167.256836, -10337.596680,
        -19400.531250, 2259.596680, -17524.410156,
        -17348.794922, 2259.596680, -19867.220703,
        -18898.181641, 2109.599609, -20516.421875
    };
    int cnt = 0;
    float polyPickExt[3] = {4.f, 40.f, 4.f};
    rcIgnoreUnused(polyPickExt);
    testerTool.m_collDet.enable();
    while (i < n) {
        auto t1 = std::chrono::steady_clock::now();
        dtStatus st = navQuery.findNearestPoly(
            &start[(i % k) * 3], polyPickExt, &testerTool.m_filter, &testerTool.m_startRef, 0);
        if (dtStatusFailed(st)) {
            std::printf("Error of findNearestPoly 1\n");
            return;
        }
        st = navQuery.findNearestPoly(
            &end[(i % k) * 3], polyPickExt, &testerTool.m_filter, &testerTool.m_endRef, 0);
        if (dtStatusFailed(st)) {
            std::printf("Error of findNearestPoly 2\n");
            return;
        }
        cnt += testerTool.findPathWithJumps(
            testerTool.m_startRef, testerTool.m_endRef, &start[i % k], &end[i % k]
        );
        //cnt += space.segTriCollision(&start[(i % k) * 3], &end[(i % k) * 3], t);
        //cnt += geom.raycastMesh(&start[(i % k) * 3], &end[(i % k) * 3], t);
        auto t2 = std::chrono::steady_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        std::printf("Test call, microsecs delta: %zu\n", diff);
        ++i;
        if (!(i % k)) {
            std::printf("\n");
        }
        if (i == k * 3) break;
    }
    std::printf("N: %d, %f\n", cnt, t);
    return;
}
*/

static const int ERR_MSG_BUFF_SIZE = 1024;
static char errMsg[ERR_MSG_BUFF_SIZE];
static bool shrinkBvhAabb = false;
static float offsetSize = 2000.f;
static float bvhGridSize = 1024.f;
static bool done = false;
static bool showLevels = false;
static bool showSample = false;
static bool showTestCases = false;
static bool presentationMode = false;
static bool showMenu = !presentationMode;
static float scrollZoom = 0;
static bool mouseOverMenu = false;
static int mouseScroll = 0;
static bool rotate = false;
static bool movedDuringRotate = false;
static int mousePos[2] = {0, 0};
static int origMousePos[2] = {0, 0}; // Used to compute mouse movement totals across frames.
static float cameraEulers[] = {45, -45};
static float origCameraEulers[] = {0, 0}; // Used to compute rotational changes across frames.
static int width;
static int height;
static std::vector<std::string> files;
static const std::string testCasesFolder = "TestCases";
static InputGeom* geom = 0;
static Sample* sample = 0;

void processMouseKeyboardEvents(
	bool& processHitTest, bool& processHitTestShift, unsigned char& mouseButtonMask
) {
	SDL_Event event;
	while (SDL_PollEvent(&event))
	{
		switch (event.type)
		{
			case SDL_KEYDOWN:
				// Handle any key presses here.
				if (event.key.keysym.sym == SDLK_ESCAPE)
				{
					done = true;
				}
				else if (event.key.keysym.sym == SDLK_t)
				{
					showLevels = false;
					showSample = false;
					showTestCases = true;
					scanDirectory(testCasesFolder, ".txt", files);
				}
				else if (event.key.keysym.sym == SDLK_TAB)
				{
					showMenu = !showMenu;
				}
				else if (event.key.keysym.sym == SDLK_SPACE)
				{
					if (sample)
						sample->handleToggle();
				}
				else if (event.key.keysym.sym == SDLK_1)
				{
					if (sample)
						sample->handleStep();
				}
				break;

			case SDL_MOUSEWHEEL:
				if (event.wheel.y < 0)
				{
					// wheel down
					if (mouseOverMenu)
					{
						mouseScroll++;
					}
					else
					{
						scrollZoom += 1.0f;
					}
				}
				else
				{
					if (mouseOverMenu)
					{
						mouseScroll--;
					}
					else
					{
						scrollZoom -= 1.0f;
					}
				}
				break;
			case SDL_MOUSEBUTTONDOWN:
				if (event.button.button == SDL_BUTTON_RIGHT)
				{
					if (!mouseOverMenu)
					{
						// Rotate view
						rotate = true;
						movedDuringRotate = false;
						origMousePos[0] = mousePos[0];
						origMousePos[1] = mousePos[1];
						origCameraEulers[0] = cameraEulers[0];
						origCameraEulers[1] = cameraEulers[1];
					}
				}
				break;

			case SDL_MOUSEBUTTONUP:
				// Handle mouse clicks here.
				if (event.button.button == SDL_BUTTON_RIGHT)
				{
					rotate = false;
					if (!mouseOverMenu)
					{
						if (!movedDuringRotate)
						{
							processHitTest = true;
							processHitTestShift = true;
						}
					}
				}
				else if (event.button.button == SDL_BUTTON_LEFT)
				{
					if (!mouseOverMenu)
					{
						processHitTest = true;
						processHitTestShift = (SDL_GetModState() & KMOD_SHIFT) ? true : false;
					}
				}

				break;

			case SDL_MOUSEMOTION:
				mousePos[0] = event.motion.x;
				mousePos[1] = height-1 - event.motion.y;

				if (rotate)
				{
					int dx = mousePos[0] - origMousePos[0];
					int dy = mousePos[1] - origMousePos[1];
					cameraEulers[0] = origCameraEulers[0] - dy * 0.25f;
					cameraEulers[1] = origCameraEulers[1] + dx * 0.25f;
					if (dx * dx + dy * dy > 3 * 3)
					{
						movedDuringRotate = true;
					}
				}
				break;

			case SDL_QUIT:
				done = true;
				break;

			default:
				break;
		}
	}

	if (SDL_GetMouseState(0, 0) & SDL_BUTTON_LMASK)
		mouseButtonMask |= IMGUI_MBUT_LEFT;
	if (SDL_GetMouseState(0, 0) & SDL_BUTTON_RMASK)
		mouseButtonMask |= IMGUI_MBUT_RIGHT;
}

void finalizeRendering(SDL_Window* window)
{
	imguiEndFrame();
	imguiRenderGLDraw();
	glEnable(GL_DEPTH_TEST);
	SDL_GL_SwapWindow(window);
}

void renderGui()
{
	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, width, 0, height);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void renderLog(int& logScroll, const BuildContext& ctx, bool duplicateToConsole)
{
	if (imguiBeginScrollArea("Log", 250 + 20, 10, width - 300 - 250, 200, &logScroll))
		mouseOverMenu = true;
	if (duplicateToConsole) {
		for (int i = 0; i < ctx.getLogCount(); ++i) {
			imguiLabel(ctx.getLogText(i));
			std::printf("%s\n", ctx.getLogText(i));
		}
	}
	else {
		for (int i = 0; i < ctx.getLogCount(); ++i) {
			imguiLabel(ctx.getLogText(i));
		}
	}
	imguiEndScrollArea();
}

void saveLog(const char* path, BuildContext& ctx)
{
	std::string fullName(path);
	fullName += "/";
	fullName += "log.txt";
	FILE* fp = fopen(fullName.c_str(), "w");
	if (!fp) {
		ctx.log(RC_LOG_ERROR, "Error of saving log file");
		return;
	}
	for (int i = 0; i < ctx.getLogCount(); ++i)
	{
		const char* str = ctx.getLogText(i);
		fwrite(str, sizeof(char), std::strlen(str), fp);
		fwrite("\n", sizeof(char), 1, fp);
	}
	fclose(fp);
}

static void errorPrint(const char* s)
{
#ifdef WIN32
	MessageBoxA(NULL, s, "Fatal error message", MB_OK);
#else
	std::printf("%s\n", s);
#endif
}

int mainInternal(int /*argc*/, char** /*argv*/)
{
    //if (argc > 1 && !std::strcmp(argv[1], "profile")) {
    //    if (!std::strcmp(argv[1], "profile"))
    //            runProfile();
    //    return 0;
    //}

    // Init SDL
	if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
	{
		snprintf(errMsg, ERR_MSG_BUFF_SIZE, "Could not initialise SDL, msg: %s\n", SDL_GetError());
		errorPrint(errMsg);
		return -1;
	}

	// Use OpenGL render driver.
	(SDL_HINT_RENDER_DRIVER, "opengl");

	// Enable depth buffer.
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
	
	// Set color channel depth.
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
	
	// 4x MSAA.
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

	SDL_DisplayMode displayMode;
	SDL_GetCurrentDisplayMode(0, &displayMode);

	Uint32 flags = SDL_WINDOW_OPENGL;
	if (presentationMode)
	{
		// Create a fullscreen window at the native resolution.
		width = displayMode.w;
		height = displayMode.h;
		flags |= SDL_WINDOW_FULLSCREEN;
	}
	else
	{
		float aspect = 16.0f / 9.0f;
		width = rcMin(displayMode.w, (int)(displayMode.h * aspect)) - 80;
		height = displayMode.h - 80;
	}
	
	SDL_Window* window;
	SDL_Renderer* renderer;
	int errorCode = SDL_CreateWindowAndRenderer(width, height, flags, &window, &renderer);

	if (errorCode != 0 || !window || !renderer)
	{
		snprintf(errMsg, ERR_MSG_BUFF_SIZE, "Could not initialise SDL opengl, msg: %s\n", SDL_GetError());
		errorPrint(errMsg);
		return -1;
	}

	SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
	SDL_GL_CreateContext(window);
	if (!initOpenglMethods())
	{
		errorPrint("Could not initialise ptrs of opengl methods\n");
		return -1;
	}

	if (!imguiRenderGLInit("DroidSans.ttf"))
	{
		snprintf(errMsg, ERR_MSG_BUFF_SIZE,
			"Could not init GUI renderer, error of font loading: %s\n", "DroidSans.ttf");
		errorPrint(errMsg);
		SDL_Quit();
		return -1;
	}
	
	float t = 0.0f;
	float timeAcc = 0.0f;
	Uint32 prevFrameTime = SDL_GetTicks();
	
	float cameraPos[] = {0, 0, 0};
	float camr = 1000;
	
	float moveFront = 0.0f, moveBack = 0.0f, moveLeft = 0.0f, moveRight = 0.0f, moveUp = 0.0f, moveDown = 0.0f;
	
	float rayStart[3];
	float rayEnd[3];
	
	bool duplicateToConsole = false;
	bool showLog = false;
	bool showTools = true;

	// Window scroll positions.
	int asyncScroll = 0;
	int propScroll = 0;
	int logScroll = 0;
	int toolsScroll = 0;
	
	string sampleName = "Choose Sample...";
	const string meshesFolder = "Meshes";
	string meshName = "Choose Mesh...";
	
	float markerPosition[3] = {0, 0, 0};
	bool markerPositionSet = false;

	TestCase* test = 0;

	BuildContext ctx;
	
	// Fog.
	float fogColor[4] = { 0.32f, 0.31f, 0.30f, 1.0f };
	glEnable(GL_FOG);
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_START, camr * 0.1f);
	glFogf(GL_FOG_END, camr * 1.25f);
	glFogfv(GL_FOG_COLOR, fogColor);
	
	glEnable(GL_CULL_FACE);
	glDepthFunc(GL_LEQUAL);

	bool asyncBuilding = false;
	while(!done)
	{
        float cam_speed = sample ? sample->getCamSpeed() : 1.f;
        // Handle input events.
		bool processHitTest = false;
		bool processHitTestShift = false;
		unsigned char mouseButtonMask = 0;

		processMouseKeyboardEvents(processHitTest, processHitTestShift, mouseButtonMask);
		
		Uint32 time = SDL_GetTicks();
		float dt = (time - prevFrameTime) / 1000.0f;
		prevFrameTime = time;
		t += dt;

		// Hit test mesh.
		if (processHitTest && geom && sample)
		{
			float hitTime;
			bool hit = geom->raycastMesh(rayStart, rayEnd, hitTime, true);
			
			if (hit)
			{
				if (SDL_GetModState() & KMOD_CTRL)
				{
					// Marker
					markerPositionSet = true;
					markerPosition[0] = rayStart[0] + (rayEnd[0] - rayStart[0]) * hitTime;
					markerPosition[1] = rayStart[1] + (rayEnd[1] - rayStart[1]) * hitTime;
					markerPosition[2] = rayStart[2] + (rayEnd[2] - rayStart[2]) * hitTime;
				}
				else
				{
					float pos[3];
					pos[0] = rayStart[0] + (rayEnd[0] - rayStart[0]) * hitTime;
					pos[1] = rayStart[1] + (rayEnd[1] - rayStart[1]) * hitTime;
					pos[2] = rayStart[2] + (rayEnd[2] - rayStart[2]) * hitTime;
					sample->handleClick(rayStart, pos, processHitTestShift);
				}
			}
			else
			{
				if (SDL_GetModState() & KMOD_CTRL)
				{
					// Marker
					markerPositionSet = false;
				}
			}
		}
		
		// Update sample simulation.
		const float SIM_RATE = 20;
		const float DELTA_TIME = 1.0f / SIM_RATE;
		timeAcc = rcClamp(timeAcc + dt, -1.0f, 1.0f);
		int simIter = 0;
		while (timeAcc > DELTA_TIME)
		{
			timeAcc -= DELTA_TIME;
			if (!asyncBuilding && simIter < 5 && sample)
			{
				sample->handleUpdate(DELTA_TIME);
			}
			simIter++;
		}

		// Clamp the framerate so that we do not hog all the CPU.
		const float MIN_FRAME_TIME = 1.0f / 40.0f;
		if (dt < MIN_FRAME_TIME)
		{
			int ms = (int)((MIN_FRAME_TIME - dt) * 1000.0f);
			if (ms > 10) ms = 10;
			if (ms >= 0) SDL_Delay(ms);
		}
		
		// Set the viewport.
		glViewport(0, 0, width, height);
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);
		
		// Clear the screen
		glClearColor(0.3f, 0.3f, 0.32f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_TEXTURE_2D);
		glEnable(GL_DEPTH_TEST);
		
		// Compute the projection matrix.
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(50.0f, (float)width/(float)height, 1.0f, camr);
		GLdouble projectionMatrix[16];
		glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
		
		// Compute the modelview matrix.
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glRotatef(cameraEulers[0], 1, 0, 0);
		glRotatef(cameraEulers[1], 0, 1, 0);
		glTranslatef(-cameraPos[0], -cameraPos[1], -cameraPos[2]);
		GLdouble modelviewMatrix[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, modelviewMatrix);

		// Get hit ray position and direction.
		GLdouble x, y, z;
		gluUnProject(mousePos[0], mousePos[1], 0.0f, modelviewMatrix, projectionMatrix, viewport, &x, &y, &z);
		rayStart[0] = (float)x;
		rayStart[1] = (float)y;
		rayStart[2] = (float)z;
		gluUnProject(mousePos[0], mousePos[1], 1.0f, modelviewMatrix, projectionMatrix, viewport, &x, &y, &z);
		rayEnd[0] = (float)x;
		rayEnd[1] = (float)y;
		rayEnd[2] = (float)z;
		
		// Handle keyboard movement.
		const Uint8* keystate = SDL_GetKeyboardState(NULL);
		moveFront	= rcClamp(moveFront	+ dt * 4 * ((keystate[SDL_SCANCODE_W] || keystate[SDL_SCANCODE_UP		]) ? 1 : -1), 0.0f, 1.0f);
        moveLeft	= rcClamp(moveLeft	+ dt * 4 * ((keystate[SDL_SCANCODE_A] || keystate[SDL_SCANCODE_LEFT		]) ? 1 : -1), 0.0f, 1.0f);
		moveBack	= rcClamp(moveBack	+ dt * 4 * ((keystate[SDL_SCANCODE_S] || keystate[SDL_SCANCODE_DOWN		]) ? 1 : -1), 0.0f, 1.0f);
		moveRight	= rcClamp(moveRight	+ dt * 4 * ((keystate[SDL_SCANCODE_D] || keystate[SDL_SCANCODE_RIGHT	]) ? 1 : -1), 0.0f, 1.0f);
		moveUp		= rcClamp(moveUp	+ dt * 4 * ((keystate[SDL_SCANCODE_Q] || keystate[SDL_SCANCODE_PAGEUP	]) ? 1 : -1), 0.0f, 1.0f);
		moveDown	= rcClamp(moveDown	+ dt * 4 * ((keystate[SDL_SCANCODE_E] || keystate[SDL_SCANCODE_PAGEDOWN	]) ? 1 : -1), 0.0f, 1.0f);
		
		float keybSpeed = 22.0f;
		if (SDL_GetModState() & KMOD_SHIFT)
		{
            keybSpeed *= 4.0f;
		}
		
        float movex = (moveRight - moveLeft) * keybSpeed * dt * cam_speed;
        float movey =
            (moveBack - moveFront) * keybSpeed * dt * cam_speed + scrollZoom * 2.0f;
		scrollZoom = 0;
		
		cameraPos[0] += movex * (float)modelviewMatrix[0];
		cameraPos[1] += movex * (float)modelviewMatrix[4];
		cameraPos[2] += movex * (float)modelviewMatrix[8];
		cameraPos[0] += movey * (float)modelviewMatrix[2];
		cameraPos[1] += movey * (float)modelviewMatrix[6];
		cameraPos[2] += movey * (float)modelviewMatrix[10];
        cameraPos[1] += (moveUp - moveDown) * keybSpeed * dt * cam_speed;

		glEnable(GL_FOG);
		if (!asyncBuilding && sample)
            sample->handleRender(cameraPos);
		if (!asyncBuilding && test)
            test->handleRender();
		glDisable(GL_FOG);

		// Render GUI
		renderGui();
		imguiBeginFrame(mousePos[0], mousePos[1], mouseButtonMask, mouseScroll);
		mouseScroll = 0;
		mouseOverMenu = false;

		if (asyncBuilding) {
			if (
				imguiBeginScrollArea(
					"Async navmesh building", width-250-10, 10, 250, height-20, &asyncScroll
			)) {
				mouseOverMenu = true;
			}
			imguiSeparator();
			char buf[32];
			snprintf(buf, 32, "Progress: %.2f %%", sample->getAsyncBuildingProgress());
			imguiLabel(buf);
			imguiSeparator();
			if (imguiButton("Stop")) {
				sample->interruptAsyncBuilding();
			}
			else {
				asyncBuilding = sample->isAsyncBuilding();
			}
			imguiEndScrollArea();
			renderLog(logScroll, ctx, duplicateToConsole);
			finalizeRendering(window);
			continue;
		}

		if (sample)
		{
			sample->handleRenderOverlay((double*)projectionMatrix, (double*)modelviewMatrix, (int*)viewport);
		}
		if (test)
		{
			if (test->handleRenderOverlay((double*)projectionMatrix, (double*)modelviewMatrix, (int*)viewport))
				mouseOverMenu = true;
		}

		// Help text.
		if (showMenu)
		{
			const char msg[] = "W/S/A/D: Move  RMB: Rotate";
			imguiDrawText(280, height-20, IMGUI_ALIGN_LEFT, msg, imguiRGBA(255,255,255,128));
		}
		
		if (showMenu)
		{
			if (imguiBeginScrollArea("Properties", width-250-10, 10, 250, height-20, &propScroll))
				mouseOverMenu = true;

			if (imguiCheck("Duplicate log to console", duplicateToConsole))
				duplicateToConsole = !duplicateToConsole;
			if (imguiCheck("Show Log", showLog))
				showLog = !showLog;
			if (imguiCheck("Show Tools", showTools))
				showTools = !showTools;

			imguiSeparator();
			if (imguiButton("Clear log"))
			{
				ctx.resetLog();
			}
			imguiSeparator();
			if (imguiButton("Save log"))
			{
				if (sample) {
					saveLog(sample->getInputGeom()->getBaseMeshName(), ctx);
				}
			}
			imguiSeparator();
			imguiLabel("Sample");
			if (imguiButton(sampleName.c_str()))
			{
				if (showSample)
				{
					showSample = false;
				}
				else
				{
					showSample = true;
					showLevels = false;
					showTestCases = false;
				}
			}
			
			imguiSeparator();
			if (imguiCheck("Shrink bvh aabb", shrinkBvhAabb))
				shrinkBvhAabb = !shrinkBvhAabb;
            imguiSlider("Offset size", &offsetSize, 0.f, 5000.f, 100.f);
            imguiSlider("Bvh grid size", &bvhGridSize, 256.f, 16384.f, 256.f);
			imguiLabel("Input Mesh");
			if (imguiButton(meshName.c_str()))
			{
				if (showLevels)
				{
					showLevels = false;
				}
				else
				{
					showSample = false;
					showTestCases = false;
					showLevels = true;
                    //scanDirectory(meshesFolder, ".obj", files);
                    scanDirectory(meshesFolder, files);
					scanDirectoryAppend(meshesFolder, ".gset", files);
				}
			}
			imguiSeparator();
			if (imguiButton("Save binary mesh"))
			{
				geom->saveBinaryMesh();
			}
			if (geom)
			{
				char text[128];
				const auto& rndData = geom->getSpace().getRenderingData();
				snprintf(
					text, 128, "Rendering v: %.1fK; t: %.1fK",
					rndData.vertsNumCurrent /1000.0f,
					rndData.trisNumCurrent /1000.0f
				);
				imguiValue(text);
			}
			imguiSeparator();

			if (geom && sample)
			{
				imguiSeparatorLine();
				
				sample->handleSettings();

				if (imguiButton("Build"))
				{
					ctx.resetLog();
					bool res = sample->handleBuild();
					if (!res) {
						showLog = true;
						logScroll = 0;
					}
					ctx.log(RC_LOG_PROGRESS, "Build started for mesh: '%s'", meshName.c_str());
					
					// Clear test.
					delete test;
					test = 0;

					asyncBuilding = sample->isAsyncBuilding() && res;
					if (asyncBuilding)
					{
						imguiEndScrollArea();
						finalizeRendering(window);
						continue;
					}
				}

				imguiSeparator();
			}
			
			if (sample)
			{
				imguiSeparatorLine();
				sample->handleDebugMode();
			}

			imguiEndScrollArea();
		}

		// Sample selection dialog.
		if (showSample)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea("Choose Sample", width-10-250-10-200, height-10-250, 200, 250, &levelScroll))
				mouseOverMenu = true;

			Sample* newSample = 0;
			for (int i = 0; i < g_nsamples; ++i)
			{
				if (imguiItem(g_samples[i].name.c_str()))
				{
					newSample = g_samples[i].create();
					if (newSample)
						sampleName = g_samples[i].name;
				}
			}
			if (newSample)
			{
				delete sample;
				sample = newSample;
				sample->setContext(&ctx);
				if (geom)
				{
					sample->handleMeshChanged(geom);
				}
				showSample = false;
			}

			if (geom || sample)
			{
				const float* bmin = 0;
				const float* bmax = 0;
				if (geom)
				{
					bmin = geom->getNavMeshBoundsMin();
					bmax = geom->getNavMeshBoundsMax();
				}
				// Reset camera and fog to match the mesh bounds.
				if (bmin && bmax)
				{
					camr = sqrtf(rcSqr(bmax[0]-bmin[0]) +
								 rcSqr(bmax[1]-bmin[1]) +
								 rcSqr(bmax[2]-bmin[2])) / 2;
					cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
					cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
					cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
					camr *= 3;
				}
				cameraEulers[0] = 45;
				cameraEulers[1] = -45;
				glFogf(GL_FOG_START, camr*0.1f);
				glFogf(GL_FOG_END, camr*1.25f);
			}
			
			imguiEndScrollArea();
		}

		// Level selection dialog.
		if (showLevels)
		{
			static int levelScroll = 0;
			if (imguiBeginScrollArea("Choose Level", width - 10 - 250 - 10 - 200, height - 10 - 450, 200, 450, &levelScroll))
				mouseOverMenu = true;
			
			vector<string>::const_iterator fileIter = files.begin();
			vector<string>::const_iterator filesEnd = files.end();
			vector<string>::const_iterator levelToLoad = filesEnd;
			for (; fileIter != filesEnd; ++fileIter)
			{
				if (imguiItem(fileIter->c_str()))
				{
					levelToLoad = fileIter;
				}
			}
			
			if (levelToLoad != filesEnd)
			{
				meshName = *levelToLoad;
				showLevels = false;
				
				freeAligned<InputGeom>(geom);
				geom = 0;
				
				string path = meshesFolder + "/" + meshName;
				
				geom = allocAligned<InputGeom>(16);
				//if (!geom->load(&ctx, path))
                if (!geom->loadFromDir(&ctx, path.c_str(), offsetSize, bvhGridSize, shrinkBvhAabb))
				{
					freeAligned<InputGeom>(geom);
					geom = 0;

					// Destroy the sample if it already had geometry loaded, as we've just deleted it!
					if (sample && sample->getInputGeom())
					{
						delete sample;
						sample = 0;
					}
					
					showLog = true;
					logScroll = 0;
					ctx.log(RC_LOG_PROGRESS, "Geom load for mesh: %s", meshName.c_str());
				}
				if (sample && geom)
				{
					sample->handleMeshChanged(geom);
				}

				if (geom || sample)
				{
					const float* bmin = 0;
					const float* bmax = 0;
					if (geom)
					{
						bmin = geom->getNavMeshBoundsMin();
						bmax = geom->getNavMeshBoundsMax();
					}
					// Reset camera and fog to match the mesh bounds.
					if (bmin && bmax)
					{
						camr = sqrtf(rcSqr(bmax[0]-bmin[0]) +
									 rcSqr(bmax[1]-bmin[1]) +
									 rcSqr(bmax[2]-bmin[2])) / 2;
						cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
						cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
						cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
						camr *= 3;
					}
					cameraEulers[0] = 45;
					cameraEulers[1] = -45;
					glFogf(GL_FOG_START, camr * 0.1f);
					glFogf(GL_FOG_END, camr * 1.25f);
				}
			}
			
			imguiEndScrollArea();
		}

		// Test cases
		if (showTestCases)
		{
			static int testScroll = 0;
			if (imguiBeginScrollArea("Choose Test To Run", width-10-250-10-200, height-10-450, 200, 450, &testScroll))
				mouseOverMenu = true;

			vector<string>::const_iterator fileIter = files.begin();
			vector<string>::const_iterator filesEnd = files.end();
			vector<string>::const_iterator testToLoad = filesEnd;
			for (; fileIter != filesEnd; ++fileIter)
			{
				if (imguiItem(fileIter->c_str()))
				{
					testToLoad = fileIter;
				}
			}
			
			if (testToLoad != filesEnd)
			{
				string path = testCasesFolder + "/" + *testToLoad;
				test = new TestCase;
				if (test)
				{
					// Load the test.
					if (!test->load(path))
					{
						delete test;
						test = 0;
					}

					// Create sample
					Sample* newSample = 0;
					for (int i = 0; i < g_nsamples; ++i)
					{
						if (g_samples[i].name == test->getSampleName())
						{
							newSample = g_samples[i].create();
							if (newSample)
								sampleName = g_samples[i].name;
						}
					}

					delete sample;
					sample = newSample;

					if (sample)
					{
						sample->setContext(&ctx);
						showSample = false;
					}

					// Load geom.
					meshName = test->getGeomFileName();
					
					
					path = meshesFolder + "/" + meshName;
					
					freeAligned<InputGeom>(geom);
					geom = allocAligned<InputGeom>(16);
					if (!geom || !geom->loadFromDir(&ctx, path.c_str(), 0.f, 0.f, false))
					{
						freeAligned<InputGeom>(geom);
						geom = 0;
						delete sample;
						sample = 0;
						showLog = true;
						logScroll = 0;
						ctx.log(RC_LOG_PROGRESS, "Geom load for mesh: %s", meshName.c_str());
					}
					if (sample && geom)
					{
						sample->handleMeshChanged(geom);
					}

					// This will ensure that tile & poly bits are updated in tiled sample.
					if (sample)
						sample->handleSettings();

					ctx.resetLog();
					if (sample && !sample->handleBuild())
					{
						ctx.log(RC_LOG_PROGRESS, "Build started for mesh: %s", meshName.c_str());
					}
					
					if (geom || sample)
					{
						const float* bmin = 0;
						const float* bmax = 0;
						if (geom)
						{
							bmin = geom->getNavMeshBoundsMin();
							bmax = geom->getNavMeshBoundsMax();
						}
						// Reset camera and fog to match the mesh bounds.
						if (bmin && bmax)
						{
							camr = sqrtf(rcSqr(bmax[0] - bmin[0]) +
										 rcSqr(bmax[1] - bmin[1]) +
										 rcSqr(bmax[2] - bmin[2])) / 2;
							cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
							cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
							cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
							camr *= 3;
						}
						cameraEulers[0] = 45;
						cameraEulers[1] = -45;
						glFogf(GL_FOG_START, camr * 0.2f);
						glFogf(GL_FOG_END, camr * 1.25f);
					}
					
					// Do the tests.
					if (sample)
						test->doTests(sample->getNavMesh(), sample->getNavMeshQuery());
				}
			}				
				
			imguiEndScrollArea();
		}

		// Log
		if (showLog && showMenu)
		{
			renderLog(logScroll, ctx, duplicateToConsole);
		}
		
		// Left column tools menu
		if (!showTestCases && showTools && showMenu)
		{
			if (imguiBeginScrollArea("Tools", 10, 10, 250, height - 20, &toolsScroll))
				mouseOverMenu = true;

			if (sample) {
				sample->handleTools();
				asyncBuilding = sample->isAsyncBuilding();
			}
			if (asyncBuilding)
			{
				imguiEndScrollArea();
				finalizeRendering(window);
				continue;
			}
			imguiEndScrollArea();
		}
		
		// Marker
		if (
			markerPositionSet &&
			gluProject(
				(GLdouble)markerPosition[0],
				(GLdouble)markerPosition[1],
				(GLdouble)markerPosition[2],
				modelviewMatrix,
				projectionMatrix,
				viewport, &x, &y, &z
			)
		) {
			// Draw marker circle
			glLineWidth(5.0f);
			glColor4ub(240,220,0,196);
			glBegin(GL_LINE_LOOP);
			const float r = 25.0f;
			for (int i = 0; i < 20; ++i)
			{
				const float a = (float)i / 20.0f * RC_PI*2;
				const float fx = (float)x + cosf(a)*r;
				const float fy = (float)y + sinf(a)*r;
				glVertex2f(fx,fy);
			}
			glEnd();
			glLineWidth(1.0f);
        }

		finalizeRendering(window);
	}
	
	imguiRenderGLDestroy();
	
	SDL_Quit();
	
	delete sample;
	freeAligned<InputGeom>(geom);
	
	return 0;
}

int main(int argc, char** argv)
{
	try {
		return mainInternal(argc, argv);
	}
	catch (const std::exception& e) {
#ifdef WIN32
		MessageBoxA(NULL, e.what(), "Exception error", MB_OK);
#else
		std::printf("Exception error: %s\n", e.what());
#endif
	}
	return 1;
}
