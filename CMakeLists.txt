cmake_minimum_required(VERSION 2.8.9)
project(VideoTags)
find_package (Threads)
set(LDFLAGS "${LDFLAGS} -lv4l1 -lv4l2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(AprilTags . /opt/local/include)
file(GLOB SOURCES 	"Source/src/*.cpp" 
			"Source/Include/*.hpp"
			"../deps/adt/source/include/ADT_Base.hpp" 
			"../deps/sensorInterfaces/source/include/Sensor.hpp"
			"../deps/adt/source/include/Length.hpp"
			"../deps/adt/source/src/Length.cpp"
			"../deps/adt/source/include/Coordinate.hpp"
			"../deps/adt/source/src/Coordinate.cpp"
			"../deps/AprilTags/TagDetection.h"
			"../deps/AprilTags/TagDetection.cc"
			)

include_directories(
 	../deps/sensorInterfaces/source/include
	../deps/adt/source/include
	../deps/AprilTags
	../deps/*
	source/Include)

add_executable(VideoTags "Source/src/VideoTag.cpp" "Source/Include/VideoTag.hpp" ${SOURCES})
target_link_libraries(VideoTags ${OpenCV_LIBS})

