cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

add_definitions(-std=c++14)

set(CMAKE_CXX_STANDARD 14)

set(CXX_FLAGS "-Wall")

set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable(environment
        src/environment.cpp
        src/render/render.cpp
        src/processPointClouds.cpp
        src/processing.cpp
        src/kdtree.cpp)
target_link_libraries(environment ${PCL_LIBRARIES})

# add_subdirectory(src/quiz/ransac)
# add_subdirectory(src/quiz/cluster)
