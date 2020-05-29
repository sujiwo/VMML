cmake_minimum_required(VERSION 3.0.2)
project(oxford_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rosbag_viewer vision_core)
find_package(Boost REQUIRED COMPONENTS system filesystem)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES oxford_test
	CATKIN_DEPENDS vision_core
#  DEPENDS system_lib
)

include_directories(
# include
	${catkin_INCLUDE_DIRS}
)

#XXX: Move to add_library

add_executable(oxford_open
	OxfordDataset.cpp
	oxford_open.cpp
)

target_link_libraries(
	oxford_open
		${catkin_LIBRARIES}
		Boost::system
)

install(
	TARGETS oxford_open
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/model
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	FILES_MATCHING PATTERN *
)
