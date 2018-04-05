#find_package(DART 7.0.0 REQUIRED COMPONENTS io-urdf gui gui-osg CONFIG)
find_package(DART 6.4 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)
include_directories(${DART_INCLUDE_DIRS})
add_subdirectory (DART_Systems)

#include_directories(${DART_INCLUDE_DIRS})
#add_executable(interface_test_main main_interface_test.cpp)
#target_link_libraries(interface_test_main Valkyrie_Controller
    #dynacore_utils
    #${DART_LIBRARIES})

