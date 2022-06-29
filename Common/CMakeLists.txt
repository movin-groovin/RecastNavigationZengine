file(GLOB SOURCES Source/*.cpp)
add_library(Common ${SOURCES})

add_library(RecastNavigation::Common ALIAS Common)
set_target_properties(Common PROPERTIES DEBUG_POSTFIX -d)

set(Detour_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

target_include_directories(Common PUBLIC
    "$<BUILD_INTERFACE:${Detour_INCLUDE_DIR}>"
)

set_target_properties(Common PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${LIB_VERSION}
        COMPILE_PDB_OUTPUT_DIRECTORY .
        COMPILE_PDB_NAME "Common-d"
        )

install(TARGETS Common
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT library
        )

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation)
if(MSVC)
    install(FILES "$<TARGET_FILE_DIR:Common>/Common-d.pdb" CONFIGURATIONS "Debug" DESTINATION "lib")
endif()
