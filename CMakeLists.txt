cmake_minimum_required(VERSION 3.3)
project(INF552)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(OpenCV_STATIC OFF)
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

set(SOURCE_FILES
        Ransac.cpp Panorama.cpp)

add_executable(INF552 ${SOURCE_FILES} Panorama.cpp)

TARGET_LINK_LIBRARIES(INF552 ${OpenCV_LIBS} )
