FILE(GLOB_RECURSE c_sources src/*.c)
FILE(GLOB_RECURSE cpp_sources src/*.cpp)
add_library (SJsnopt SHARED ${c_sources} ${cpp_sources})
target_include_directories(SJsnopt PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
   message("************ LINUX SNOPT **************")
   #find_library(snopt7_c++ NAMES libsnopt7_cpp.so.0 libsnopt7_cpp.so.0.0.0 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/ REQUIRED)
   #find_library(snopt7_int NAMES libsnopt7.so.0 libsnopt7.so.0.0.0 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/ REQUIRED)
   find_library(snopt7_c++ NAMES libsnopt7_cpp.so PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/ REQUIRED)
   find_library(snopt7_int NAMES libsnopt7.so PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/ REQUIRED)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)

target_link_libraries (SJsnopt ${snopt7_c++})
target_link_libraries (SJsnopt ${snopt7_int})

#add_subdirectory(examples) # Uncomment if you want to compile the snopt examples

