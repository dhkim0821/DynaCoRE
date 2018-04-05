FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.cpp)
add_executable (simulation_valkyrie ${header} ${source} )

target_link_libraries (simulation_valkyrie
  common
  LieGroup
  srDyn
  SR_SysGenerator
  srg
  Renderer
  util
  ${OPENGL_LIBRARIES}
  SJurdf
  Valkyrie_Controller
  SR_Terrain
)

if (APPLE)
  target_link_libraries(simulation_valkyrie "-framework GLUT")
endif (APPLE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(simulation_valkyrie ${GLUT_glut_LIBRARIES})
  include (CheckIncludeFileCXX)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)
