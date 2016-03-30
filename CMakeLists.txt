# CMake entry point
cmake_minimum_required (VERSION 2.8)
project (Densify2D)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
	/usr/local/include/eigen3
	/Users/alexhagiopol/gtsam
	.
)
set(OpenCV_DIR "/home/alex/opencv-2.4.11/build")
set(OpenCV_FOUND 1) #fixes issue in CLion that prevents building
find_package(GTSAM REQUIRED) #CRAZY ISSUE adding this along with the other find_package statements breaks GLEW and GLFW linkage....
include_directories(${GTSAM_INCLUDE_DIRS}) #see above
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Densify2D
		main.cpp
		utils.h utils.cpp SFMPoint.h imageProcessing.h imageProcessing.cpp)

add_executable(tests
		tests.cpp
		utils.h utils.cpp SFMPoint.h imageProcessing.h imageProcessing.cpp )

target_link_libraries(Densify2D
		${ALL_LIBS}
		gtsam
		${OpenCV_LIBS}
)

target_link_libraries(tests
		${ALL_LIBS}
		gtsam
		${OpenCV_LIBS}
		)
