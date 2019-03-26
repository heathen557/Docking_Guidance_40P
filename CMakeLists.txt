cmake_minimum_required(VERSION 3.9)
project(Docking_Guidance2)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-fext-numeric-literals)

#add_subdirectory(pandar_grabber)
add_subdirectory(Pandar40P)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(Docking_Guidance2 framework.cpp framework.h Docking_Guidance.cpp ProcessPointCloud.cpp ProcessPointcloud.h WalkTest.cpp WalkTest.h Cluster.cpp Cluster.h tools_function.cpp tools_function.h AircraftDetect.cpp AircraftDetect.h GlobleData.cpp GlobleData.h logger.h logger.cpp)
include_directories(Pandar40P/include Pandar40P/src/Pandar40P/include)
#target_link_libraries(Docking_Guidance2 pandar_grabber)
target_link_libraries(Docking_Guidance2 Pandar40PSDK)
target_link_libraries(Docking_Guidance2 ${PCL_LIBRARIES})
#target_link_libraries(Docking_Guidance2 -lpthread)
target_link_libraries(Docking_Guidance2 -lpthread sqlite3)