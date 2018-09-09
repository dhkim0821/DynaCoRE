FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB_RECURSE headerspp *.hpp)
add_library(DracoBip_model SHARED ${sources} ${headerspp})

target_link_libraries (DracoBip_model dynacore_utils dynacore_rbdl)
