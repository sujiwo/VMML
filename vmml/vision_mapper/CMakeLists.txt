project(vision_mapper)

# XXX: Find better way to find libg2o by sharing script
set (CMAKE_MODULE_PATH 
		${CMAKE_MODULE_PATH} 
		"${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
set (CMAKE_INCLUDE_CURRENT_DIR		ON)

add_compile_options(-std=c++11 -frtti -fopenmp)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rosbag_storage
	std_msgs
	tf
	vision_core
	cv_bridge
	sensor_msgs
	cv_bridge
	image_transport
	pcl_conversions
	message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package (Boost REQUIRED COMPONENTS serialization python graph filesystem program_options)
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

if(SEGNET_DIR)
	message("SegNet is set")
	include("${SEGNET_DIR}/share/Caffe/CaffeConfig.cmake")
	message("SegNet include: " "${Caffe_INCLUDE_DIRS}")
	set(SEGNET_FOUND true)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_CUDNN -DUSE_LEVELDB -DUSE_LMDB -DUSE_OPENCV -DSEGNET_FOUND=1")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L${SEGNET_DIR}/lib")
endif()


add_service_files(
FILES
	place_recognizer.srv
)

generate_messages(
	DEPENDENCIES
		std_msgs
		sensor_msgs
)

catkin_package(
	CATKIN_DEPENDS vision_core
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${G2O_INCLUDE_DIR}	
	${vision_core_SOURCE_DIR}
	${Caffe_INCLUDE_DIRS}
)

set(common_class 
	src/RVizConnector.cpp
	src/ProgramOptions.cpp
)
if (SEGNET_FOUND)
	list(APPEND common_class src/Segmentation.cpp)
endif()

set(common_libs
	${catkin_LIBRARIES}
	${vision_core_LIBRARIES}
	${OpenCV_LIBS}
	Boost::program_options
)
if (SEGNET_FOUND)
	list(APPEND common_libs ${Caffe_LIBRARIES})
endif()


add_library(vmml_mapper STATIC
	${common_class}
)

add_executable(
	simple_bag_mapper
		nodes/simple_bag_mapper.cpp
)

target_link_libraries(
	simple_bag_mapper
		vmml_mapper
		${common_libs}
)

install(
	TARGETS simple_bag_mapper
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(
	imagelidar_bag_mapper
		nodes/imagelidar_bag_mapper.cpp
)

target_link_libraries(
	imagelidar_bag_mapper
		vmml_mapper
		${common_libs}
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


#add_executable(
#	vocabulary_creator
#		src/RVizConnector.cpp
#		nodes/vocabulary_creator.cpp
#)
#
#target_link_libraries(
#	vocabulary_creator
#		${catkin_LIBRARIES}
#		${vision_core_LIBRARIES}
#		${OpenCV_LIBS}
#)
#
#install(
#	TARGETS vocabulary_creator
#	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


add_executable(
	index_creator
		nodes/index_creator.cpp
)

target_link_libraries(
	index_creator
		vmml_mapper
		${common_libs}
)

install(
	TARGETS index_creator
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(
	test_imagedb
		nodes/test_imagedb.cpp
)

target_link_libraries(
	test_imagedb
		${catkin_LIBRARIES}
		${vision_core_LIBRARIES}
		${OpenCV_LIBS}
)

install(
	TARGETS test_imagedb
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(
	imagedb_mapper
		nodes/imagedb_mapper.cpp
)

target_link_libraries(
	imagedb_mapper
		vmml_mapper
		${common_libs}
)

install(
	TARGETS imagedb_mapper
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Default samples
install(
	DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/params
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	PATTERN "*"
)


add_executable(
	test_program_options
		nodes/test_program_options.cpp
)

target_link_libraries(
	test_program_options
		vmml_mapper
		${common_libs}
)

install(
	TARGETS test_program_options
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(
	place_recognizer_srv
		nodes/place_recognizer_srv.cpp
)

target_link_libraries(
	place_recognizer_srv
		vmml_mapper
		${common_libs}
)

install(
	TARGETS place_recognizer_srv
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(
	image_trajectory
		nodes/image_trajectory.cpp
)

target_link_libraries(
	image_trajectory
		${common_libs}
)

install(
	TARGETS image_trajectory
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(
	test_rgb_filters
		nodes/test_rgb_filters.cpp
)

target_link_libraries(
	test_rgb_filters
		vmml_mapper
		${common_libs}
)

add_executable(
	testbag
		nodes/testbag.cpp
)

target_link_libraries(
	testbag
		vmml_mapper
		${common_libs}
)

