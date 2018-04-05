FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)
add_library(dynacore_filters SHARED ${sources} ${headers})
