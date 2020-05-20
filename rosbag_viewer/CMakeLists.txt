cmake_minimum_required(VERSION 2.8)
project(rosbag_viewer)

add_compile_options(-std=c++11)

set (CMAKE_INCLUDE_CURRENT_DIR		ON)
set (CMAKE_AUTOMOC					ON)
set (CMAKE_AUTOUIC					ON)


find_package(catkin REQUIRED COMPONENTS 
	cv_bridge
	pcl_ros
	rosbag_storage)

#find_package(Qt5 REQUIRED Core Widgets)
find_package (Qt5Core CONFIG REQUIRED)
find_package (Qt5Widgets CONFIG REQUIRED)
find_package(PCL REQUIRED)
find_package (OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  	random_access_bag
  CATKIN_DEPENDS 
	roscpp 
	rosbag_storage
)


include_directories(
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	common
	include
)

#link_directories(
#	${catkin_LIBRARY_DIRS})


add_library(
	random_access_bag
		random_access_bag/RandomAccessBag.cpp
)

target_link_libraries(
	random_access_bag
		${catkin_LIBRARIES}
)

add_library(
	image_dataset_viewer
		bag_viewer/GenericDatasetViewer.cpp
)

target_link_libraries(
	image_dataset_viewer
		Qt5::Widgets
		${OpenCV_LIBS}
)


add_executable(bag_viewer
	bag_viewer/main.cpp
	bag_viewer/BagViewer.cpp
	bag_viewer/ClickableLabel.cpp
 	bag_viewer/ratio_layouted_frame.cpp
 	bag_viewer/ratio_layouted_frame.h
	bag_viewer/BagViewer.ui
	bag_viewer/ClickableLabel.h
)

target_link_libraries(
	bag_viewer
		random_access_bag
		${catkin_LIBRARIES}
		Qt5::Widgets
		${OpenCV_LIBS}
)

target_include_directories(bag_viewer
	PUBLIC bag_viewer
)

install(
	FILES include/RandomAccessBag.h 
	DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(
	TARGETS random_access_bag
	LIBRARY
		DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
	TARGETS bag_viewer
	RUNTIME
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#if (CATKIN_ENABLE_TESTING)
#	find_package(rostest REQUIRED)
#	add_rostest_gtest(bag_test 
#		test/test_random_access_bag.launch 
#		test/test_random_access_bag.cpp)
#	target_link_libraries(bag_test 
#		random_access_bag 
#		${catkin_LIBRARIES})
#endif ()

