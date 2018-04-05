FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.cpp)
add_executable (simulation_mercury ${header} ${source} )

target_link_libraries (simulation_mercury
  common
  LieGroup
  srDyn
  SR_SysGenerator
  srg
  Renderer
  util
  ${OPENGL_LIBRARIES}
  SJurdf
  ${CTRL_SYS}_Controller
 SR_Terrain
 SJutils
 SJparam_handler
)

if (APPLE)
  target_link_libraries(simulation_mercury "-framework GLUT")
endif (APPLE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(simulation_mercury ${GLUT_glut_LIBRARIES})
  include (CheckIncludeFileCXX)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)

