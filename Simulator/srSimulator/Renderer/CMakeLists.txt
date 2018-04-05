set (headers
Model3DS.h
SimpleRenderer.h
SimpleViewer.h
Texture.h
)
set (sources
Model3DS.cpp
SimpleRenderer.cpp
SimpleViewer.cpp
Texture.cpp
)


add_library(Renderer ${sources} ${headers})
target_link_libraries(Renderer common)
target_link_libraries(Renderer LieGroup)
target_link_libraries(Renderer srDyn)
target_link_libraries(Renderer srg)
target_link_libraries(Renderer png)
target_link_libraries(Renderer ${OPENGL_LIBRARIES})
target_link_libraries(Renderer ${GLUT_glut_LIBRARY})
