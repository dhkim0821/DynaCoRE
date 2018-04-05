FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE cppheaders *.hpp)
FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB_RECURSE csources *.c)



add_library (SJnlopt SHARED ${headers} ${sources} ${cppheaders} ${csources} nlopt.f)
