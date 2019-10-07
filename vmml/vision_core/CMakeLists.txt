cmake_minimum_required(VERSION 3.1.0)
project(vision_core)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set (CMAKE_MODULE_PATH 
		${CMAKE_MODULE_PATH} 
		"${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
set (CMAKE_INCLUDE_CURRENT_DIR		ON)
#set (CMAKE_AUTOMOC					ON)
#set (CMAKE_AUTOUIC					ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	tf
	cv_bridge

	# For RandomAccessBag
	rosbag_viewer

	velodyne_pointcloud
)

find_package (Boost REQUIRED COMPONENTS serialization python graph filesystem)
find_package (OpenCV 3 REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (PCL REQUIRED)
find_package (OpenMP REQUIRED)
find_package (PythonLibs REQUIRED)
find_package (G2O REQUIRED)

if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_subdirectory (DBoW2)


catkin_package(
	LIBRARIES vmml
	CATKIN_DEPENDS roscpp libg2o rosbag_viewer
)

include_directories(
	include
	DBoW2
	${CMAKE_CURRENT_SOURCE_DIR}
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${G2O_INCLUDE_DIR}	
)

add_library(
	vmml STATIC
		src/Pose.cpp
		src/utilities.cpp
		src/CameraPinholeParams.cpp
		src/BaseFrame.cpp
		src/MapPoint.cpp
		src/Trajectory.cpp
)
