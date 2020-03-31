cmake_minimum_required(VERSION 2.8.3)
project(libopenvslam)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-fPIC)

set (CMAKE_MODULE_PATH 
		${CMAKE_MODULE_PATH} 
		"${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED)

find_package (G2O REQUIRED)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS 
	core features2d calib3d highgui)
message(STATUS "Use OpenCV ${OpenCV_VERSION}")

include(CMakePackageConfigHelpers)
set(LIBRARY_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(ARCHIVE_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(INCLUDES_DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/3rd/json/include ${PROJECT_SOURCE_DIR}/3rd/spdlog/include
  LIBRARIES openvslam
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS OpenCV G2O Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	DBoW2
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/3rd/json/include
	${PROJECT_SOURCE_DIR}/3rd/popl/include
	${PROJECT_SOURCE_DIR}/3rd/spdlog/include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${G2O_INCLUDE_DIR}
)


# built-in DBoW
add_library(DBoW2 STATIC
	DBoW2/BowVector.cpp
	DBoW2/FORB.cpp      
	DBoW2/FeatureVector.cpp
	DBoW2/ScoringObject.cpp
	DBoW2/DUtils/Random.cpp
	DBoW2/DUtils/Timestamp.cpp
)


add_subdirectory(openvslam)
