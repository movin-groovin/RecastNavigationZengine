--
-- premake5 file to build RecastDemo
-- http://premake.github.io/
--

local action = _ACTION or ""
local todir = "Build/" .. action

solution "recastnavigation"
	configurations { 
		"Debug",
		"Release"
	}

	location (todir)

	floatingpoint "Fast"
	symbols "On"
	exceptionhandling "Off"
	rtti "Off"
	flags { "FatalCompileWarnings" }
	defines { "ZENGINE_NAVMESH", "DT_POLYREF64" }

	-- debug configs
	configuration "Debug*"
		defines { "DEBUG" } 
		targetdir ( todir .. "/lib/Debug" )
 
 	-- release configs
	configuration "Release*"
		defines { "NDEBUG" }
		optimize "On"
		targetdir ( todir .. "/lib/Release" )

	configuration "not windows"
		warnings "Extra"

	-- windows specific
	configuration "windows"
		platforms { "Win32", "Win64" }
		defines { "WIN32", "_WINDOWS", "_CRT_SECURE_NO_WARNINGS", "_HAS_EXCEPTIONS=0" }
		-- warnings "Extra" uses /W4 which is too aggressive for us, so use W3 instead.
		-- Disable:
		-- * C4351: new behavior for array initialization
		buildoptions { "/W3", "/wd4351" }

	filter "platforms:Win32"
		architecture "x32"

	filter "platforms:Win64"
		architecture "x64"

project "Common"
	language "C++"
	kind "StaticLib"
	includedirs { 
		"../Common/Include"
	}
	files { 
		"../Common/Include/*.h", 
		"../Common/Source/*.cpp" 
	}

project "DebugUtils"
	language "C++"
	kind "StaticLib"
	includedirs { 
		"../DebugUtils/Include",
		"../Detour/Include",
		"../DetourTileCache/Include",
		"../Recast/Include",
		"../Common/Include"
	}
	files { 
		"../DebugUtils/Include/*.h",
		"../DebugUtils/Source/*.cpp"
	}
	links { 
		"Common"
	}

project "Detour"
	language "C++"
	kind "StaticLib"
	includedirs { 
		"../Detour/Include",
		"../Common/Include"
	}
	files { 
		"../Detour/Include/*.h", 
		"../Detour/Source/*.cpp" 
	}
	links { 
		"Common"
	}
	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"-Wno-class-memaccess"
		}

project "DetourCrowd"
	language "C++"
	kind "StaticLib"
	includedirs {
		"../DetourCrowd/Include",
		"../Detour/Include",
		"../Recast/Include"
	}
	files {
		"../DetourCrowd/Include/*.h",
		"../DetourCrowd/Source/*.cpp"
	}

project "DetourTileCache"
	language "C++"
	kind "StaticLib"
	includedirs {
		"../DetourTileCache/Include",
		"../Detour/Include",
		"../Recast/Include"
	}
	files {
		"../DetourTileCache/Include/*.h",
		"../DetourTileCache/Source/*.cpp"
	}

project "Recast"
	language "C++"
	kind "StaticLib"
	includedirs { 
		"../Recast/Include",
		"../Common/Include"
	}
	files { 
		"../Recast/Include/*.h",
		"../Recast/Source/*.cpp"
	}
	links { 
		"Common"
	}

project "NavMeshEditor"
	language "C++"
	-- Catch requires RTTI and exceptions
	exceptionhandling "On"
	rtti "On"
	defines { "USAGE_SSE_1_0", "CPP_EXCEPTIONS_ON" }
	kind "WindowedApp"
	includedirs { 
		"../NavMeshEditor/Include",
		"../NavMeshEditor/Contrib",
		"../NavMeshEditor/Contrib/fastlz",
		"../DebugUtils/Include",
		"../Detour/Include",
		"../DetourCrowd/Include",
		"../DetourTileCache/Include",
		"../Recast/Include",
		"../Common/Include"
	}
	files	{ 
		"../NavMeshEditor/Include/*.h",
		"../NavMeshEditor/Source/*.cpp",
		"../NavMeshEditor/Contrib/fastlz/*.h",
		"../NavMeshEditor/Contrib/fastlz/*.c"
	}

	-- project dependencies
	links { 
		"DebugUtils",
		"Detour",
		"DetourCrowd",
		"DetourTileCache",
		"Recast",
		"Common"
	}

	-- distribute executable in NavMeshEditor/Bin directory
	targetdir "Bin"

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags sdl2`",
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`",
			"-Wno-ignored-qualifiers",
			"-Wno-class-memaccess"

		}
		linkoptions { 
			"`pkg-config --libs sdl2`",
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}

	-- windows library cflags and libs
	configuration { "windows" }
		includedirs { "../NavMeshEditor/Contrib/SDL/include" }
		libdirs { "../NavMeshEditor/Contrib/SDL/lib/%{cfg.architecture:gsub('x86_64', 'x64')}" }
		debugdir "../NavMeshEditor/Bin/"
		buildoptions { "/EHsc" }
		linkoptions { "/STACK:4200000" }
		links { 
			"glu32",
			"opengl32",
			"SDL2",
			"SDL2main",
		}
		postbuildcommands {
			-- Copy the SDL2 dll to the Bin folder.
			'{COPY} "%{path.getabsolute("Contrib/SDL/lib/" .. cfg.architecture:gsub("x86_64", "x64") .. "/SDL2.dll")}" "%{cfg.targetdir}"'
		}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp" -- xcode4 failes to run the project if using WindowedApp
		includedirs { "/Library/Frameworks/SDL2.framework/Headers" }
		links { 
			"OpenGL.framework", 
			"SDL2.framework",
			"Cocoa.framework",
		}

project "Tests"
	language "C++"
	kind "ConsoleApp"

	-- Catch requires RTTI and exceptions
	exceptionhandling "On"
	rtti "On"

	includedirs { 
		"../DebugUtils/Include",
		"../Detour/Include",
		"../DetourCrowd/Include",
		"../DetourTileCache/Include",
		"../Recast/Include",
		"../Common/Include",
		"../Recast/Source",
		"../Tests/Recast",
		"../Tests",
	}
	files	{ 
		"../Tests/*.h",
		"../Tests/*.hpp",
		"../Tests/*.cpp",
		"../Tests/Recast/*.h",
		"../Tests/Recast/*.cpp",
		"../Tests/Detour/*.h",
		"../Tests/Detour/*.cpp",
	}

	-- project dependencies
	links { 
		"DebugUtils",
		"Detour",
		"DetourCrowd",
		"DetourTileCache",
		"Recast",
		"Common"
	}

	-- distribute executable in RecastDemo/Bin directory
	targetdir "Bin"

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"`pkg-config --cflags sdl2`",
			"`pkg-config --cflags gl`",
			"`pkg-config --cflags glu`",
			"-Wno-parentheses" -- Disable parentheses warning for the Tests target, as Catch's macros generate this everywhere.
		}
		linkoptions { 
			"`pkg-config --libs sdl2`",
			"`pkg-config --libs gl`",
			"`pkg-config --libs glu`" 
		}

	-- windows library cflags and libs
	configuration { "windows" }
		includedirs { "../RecastDemo/Contrib/SDL/include" }
		libdirs { "../RecastDemo/Contrib/SDL/lib/%{cfg.architecture:gsub('x86_64', 'x64')}" }
		debugdir "../RecastDemo/Bin/"
		links { 
			"glu32",
			"opengl32",
			"SDL2",
			"SDL2main",
		}

	-- mac includes and libs
	configuration { "macosx" }
		kind "ConsoleApp"
		includedirs { "/Library/Frameworks/SDL2.framework/Headers" }
		links { 
			"OpenGL.framework", 
			"SDL2.framework",
			"Cocoa.framework",
		}
