SET (CTRL_SYS "Mercury")
SET (ModelPath ${PROJECT_SOURCE_DIR}/Simulator/SimulationModel/${CTRL_SYS}_Model/)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

if (APPLE)
    include_directories("/opt/X11/include/GL/")
endif (APPLE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
    find_package(GLUT REQUIRED)
    cmake_policy(SET CMP0003 NEW)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)

###########  CAD FILE UNIT  ############
SET(CAD_RATIO 1.0)
if(CTRL_SYS STREQUAL "NAO")
    SET (CAD_RATIO 0.1)
elseif(CTRL_SYS STREQUAL "Valkyrie")
    SET (CAD_RATIO 1.0)
elseif(CTRL_SYS STREQUAL "DracoP1")
    SET (CAD_RATIO 0.001)
elseif(CTRL_SYS STREQUAL "DracoP1Rot")
    SET (CAD_RATIO 0.001)
    elseif(CTRL_SYS STREQUAL "Mercury")
    SET(CAD_RATIO 0.001)
endif()

CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/Simulator/srSimulator/srconfig.h.cmake ${PROJECT_SOURCE_DIR}/Simulator/srSimulator/srConfiguration.h)


include_directories ("${PROJECT_SOURCE_DIR}/Simulator/srSimulator")
add_subdirectory(Renderer)
add_subdirectory(srDyn)
add_subdirectory(srg)
add_subdirectory(common)
add_subdirectory(LieGroup)
#add_subdirectory(srTerrain)
add_subdirectory(libpng)
add_subdirectory(srSysGenerator)
add_subdirectory(Systems/${CTRL_SYS})
