cmake_minimum_required(VERSION 2.8.3)
project(vision_mapper)

# XXX: Find better way to find libg2o by sharing script
set (CMAKE_MODULE_PATH 
		${CMAKE_MODULE_PATH} 
		"${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
set (CMAKE_INCLUDE_CURRENT_DIR		ON)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
	tf
	vision_core
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package (G2O REQUIRED)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(
	simple_bag_mapper
		nodes/simple_bag_mapper.cpp
)

target_link_libraries(
	simple_bag_mapper
		${catkin_LIBRARIES}
)