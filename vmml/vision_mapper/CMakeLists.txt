cmake_minimum_required(VERSION 2.8.3)
project(vision_mapper)

# XXX: Find better way to find libg2o by sharing script
set (CMAKE_MODULE_PATH 
		${CMAKE_MODULE_PATH} 
		"${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
set (CMAKE_INCLUDE_CURRENT_DIR		ON)

add_compile_options(-std=c++11 -frtti -fopenmp)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
	tf
	vision_core
	cv_bridge
	sensor_msgs
	cv_bridge
	image_transport
	pcl_conversions
	obindex2
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
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
	CATKIN_DEPENDS vision_core
)

include_directories(
	include
	${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${G2O_INCLUDE_DIR}	
	${catkin_INCLUDE_DIRS}
	${vision_core_SOURCE_DIR}
)

add_executable(
	simple_bag_mapper
		nodes/simple_bag_mapper.cpp
		src/RVizConnector.cpp
)

target_link_libraries(
	simple_bag_mapper
		${catkin_LIBRARIES}
		${vision_core_LIBRARIES}
		${OpenCV_LIBS}
)

install(
	TARGETS simple_bag_mapper
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(
	imagelidar_bag_mapper
		nodes/imagelidar_bag_mapper.cpp
		src/RVizConnector.cpp
)

target_link_libraries(
	imagelidar_bag_mapper
		${catkin_LIBRARIES}
		${vision_core_LIBRARIES}
		${OpenCV_LIBS}
)

install(
	TARGETS imagelidar_bag_mapper
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(
	test_vo
		nodes/test_vo.cpp
)

target_link_libraries(
	test_vo
		${catkin_LIBRARIES}
		${vision_core_LIBRARIES}
		${OpenCV_LIBS}
)

install(
	TARGETS test_vo
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(
	vocabulary_creator
		src/RVizConnector.cpp
		nodes/vocabulary_creator.cpp
)

target_link_libraries(
	vocabulary_creator
		${catkin_LIBRARIES}
		${vision_core_LIBRARIES}
		${OpenCV_LIBS}
)

install(
	TARGETS vocabulary_creator
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(
	index_creator
		nodes/index_creator.cpp
)

target_link_libraries(
	index_creator
		${catkin_LIBRARIES}
		${vision_core_LIBRARIES}
		${OpenCV_LIBS}
)

install(
	TARGETS index_creator
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(
	imagedb_mapper
		nodes/imagedb_mapper.cpp
		src/RVizConnector.cpp
)

target_link_libraries(
	imagedb_mapper
		${catkin_LIBRARIES}
		${vision_core_LIBRARIES}
		${OpenCV_LIBS}
)

install(
	TARGETS imagedb_mapper
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Default Velodyne HDL-64E-S2 calibration file
install(
	FILES ${CMAKE_CURRENT_SOURCE_DIR}/params/meidai-64e-S2.yaml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Default mask
install(
	FILES ${CMAKE_CURRENT_SOURCE_DIR}/params/meidai_mask.png
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
