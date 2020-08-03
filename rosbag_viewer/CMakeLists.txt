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

# XXX: Make Qt optional.
# if it's not found,
# do not compile bag viewer and its library

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
	bag_viewer
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
	generic_imageset_viewer
		bag_viewer/GenericImagesetViewer.cpp
		bag_viewer/ratio_layouted_frame.cpp
		bag_viewer/GenericImagesetViewer.ui
		bag_viewer/ClickableLabel.cpp
		# Don't know why must include these header in build command
		include/GenericImagesetViewer.h
		include/ratio_layouted_frame.h
		include/ClickableLabel.h
)

target_link_libraries(
	generic_imageset_viewer
		Qt5::Widgets
		${OpenCV_LIBS}
)

add_executable(
	bagviewer2
		bag_viewer/bagviewer2.cpp
)

target_link_libraries(
	bagviewer2
		generic_imageset_viewer
		random_access_bag
		${catkin_LIBRARIES}
)

install(
	FILES include/RandomAccessBag.h 
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
	TARGETS random_access_bag
	LIBRARY
		DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
	TARGETS generic_imageset_viewer
	LIBRARY
		DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
	FILES include/GenericImagesetViewer.h include/ClickableLabel.h include/ratio_layouted_frame.h
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
	TARGETS bagviewer2
	RUNTIME
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

INSTALL(
	FILES bag_viewer/GenericImagesetViewer.ui
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
	FILES python/RandomAccessBag.py
	DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
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

