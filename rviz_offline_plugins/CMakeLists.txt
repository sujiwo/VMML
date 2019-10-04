cmake_minimum_required(VERSION 2.8)
project(rviz_offline_plugins)

add_compile_options(-std=c++11)

set (CMAKE_INCLUDE_CURRENT_DIR		ON)
set (CMAKE_AUTOMOC					ON)
set (CMAKE_AUTOUIC					ON)


find_package(catkin REQUIRED COMPONENTS 
	rviz
	pcl_ros
)

find_package (Qt5Core CONFIG REQUIRED)
find_package (Qt5Widgets CONFIG REQUIRED)
find_package(PCL REQUIRED)


catkin_package(
  LIBRARIES 
  	rviz_offline_display
  CATKIN_DEPENDS 
	roscpp 
	rviz 
)


include_directories(
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	common
	include
)

link_directories(
	${catkin_LIBRARY_DIRS})


add_library(
	rviz_offline_display
		src/PCDFileDisplay.cpp
		src/CSVTrajectory.cpp
		src/FileProperty.cpp
)

target_link_libraries(
	rviz_offline_display
		Qt5::Core
		Qt5::Widgets
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
)

install(
	TARGETS rviz_offline_display
	LIBRARY
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}	
)