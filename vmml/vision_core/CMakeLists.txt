cmake_minimum_required(VERSION 3.1.0)
project(vision_core)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -frtti -fopenmp)

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

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES vmml
	CATKIN_DEPENDS roscpp rosbag_viewer
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

set(REQUIRED_LIBS
	DBoW2
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
	${G2O_LIBRARIES}
	${catkin_LIBRARIES}
	Boost::serialization
	Boost::system
)


# DBoW2 is now handled directly
add_library(DBoW2
	DBoW2/BowVector.cpp
	DBoW2/FORB.cpp      
	DBoW2/FeatureVector.cpp
	DBoW2/ScoringObject.cpp
	DBoW2/DUtils/Random.cpp
	DBoW2/DUtils/Timestamp.cpp
)

target_link_libraries(DBoW2 ${OpenCV_LIBS})

install(
	TARGETS DBoW2
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Main Library
add_library(
	vmml 
		src/Pose.cpp
		src/utilities.cpp
		src/CameraPinholeParams.cpp
		src/KeyFrame.cpp
		src/BaseFrame.cpp
		src/MapPoint.cpp
		src/Trajectory.cpp
		src/VisionMap.cpp
		src/LidarScanBag.cpp
		src/MapBuilder.cpp
#		src/Initializer.cpp
		src/Triangulation.cpp
		src/Matcher.cpp
		src/Triangulation.cpp
		src/VisualOdometry.cpp
		src/ImageBag.cpp
		src/Optimizer.cpp
		src/LoopClosure.cpp
		src/MapBuilderLidar.cpp
		src/LocalLidarMapper.cpp
		src/ImageDatabaseBuilder.cpp
)

target_link_libraries(
	vmml
		${REQUIRED_LIBS}
)

install(
	TARGETS vmml
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Testing; do not install
add_executable(
	image
		test/image.cpp
)

target_link_libraries(
	image
		vmml
		${REQUIRED_LIBS}
)

add_executable(
	mapload_test
		test/mapload_test.cpp
)

target_link_libraries(
	mapload_test
		vmml
		${REQUIRED_LIBS}
)


# Generic Vocabulary handling
add_custom_target (orb_vocabulary 
	ALL 
		bunzip2 -c ${CMAKE_CURRENT_SOURCE_DIR}/samples/ORBvoc.txt.bz2 > ORBvoc.txt
	DEPENDS samples/ORBvoc.txt.bz2
	COMMENT "Extracting vocabulary... "
)
install(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/ORBvoc.txt
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



