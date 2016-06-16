cmake_minimum_required(VERSION 2.8.9)
project(VideoTags)
find_package (Threads)
set(LDFLAGS "${LDFLAGS} -lv4l1 -lv4l2 -lpthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lv4l1 -lv4l2 -lpthread")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(AprilTags . /opt/local/include)
file(GLOB SOURCES 	"source/src/*.cpp" 
			"source/include/*.hpp"
			"../adt/source/include/ADT_Base.hpp" 
			"../sensorInterfaces/source/include/Sensor.hpp"
			"../sensorInterfaces/source/include/CoordinateAttitude.hpp"
			"../adt/source/include/Length.hpp"
			"../adt/source/src/Length.cpp"
			"../adt/source/include/Coordinate.hpp"
			"../adt/source/src/Coordinate.cpp"
			"../adt/source/include/*.hpp"
			"../adt/source/src/*.cpp"
			"../deps/AprilTags/*.h"
			"../deps/AprilTags/*.cc"
			"../sharedobjects/source/include/*.hpp"
			)

include_directories(
 	../sensorInterfaces/source/include
	../sharedobjects/source/include/
	../adt/source/include
	../deps/AprilTags
	source/include)

add_executable(VideoTags "source/src/VideoTag.cpp" "source/include/VideoTag.hpp" ${SOURCES})
target_link_libraries(VideoTags ${OpenCV_LIBS} -lv4l1 -lv4l2 -lpthread)

