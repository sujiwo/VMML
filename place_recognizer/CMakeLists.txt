cmake_minimum_required(VERSION 3.0)
project(place_recognizer)

add_compile_options(-std=c++11 -frtti -fopenmp)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
)

find_package (Boost REQUIRED COMPONENTS system serialization filesystem program_options)
find_package (OpenCV 3 REQUIRED)
find_package (OpenMP REQUIRED)

if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


add_service_files(
	FILES
		place_recognizer.srv
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES place_recognizer
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
	include
	${CMAKE_CURRENT_SOURCE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_library(
	place_recognizer
		lib/IncrementalBoW.cpp
)