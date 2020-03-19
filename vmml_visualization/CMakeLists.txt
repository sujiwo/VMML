cmake_minimum_required(VERSION 2.8.3)
project(vmml_visualization)

add_compile_options(-std=c++11)

find_package (OpenCV 3 REQUIRED)
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
	vision_core
	vision_mapper
)

if(SEGNET_DIR)
	message("SegNet is set")
	include("${SEGNET_DIR}/share/Caffe/CaffeConfig.cmake")
	message("SegNet include: " "${Caffe_INCLUDE_DIRS}")
	set(SEGNET_FOUND true)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_CUDNN -DUSE_LEVELDB -DUSE_LMDB -DUSE_OPENCV -DSEGNET_FOUND=1")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L${SEGNET_DIR}/lib")
endif()

catkin_package(
#  INCLUDE_DIRS include
	CATKIN_DEPENDS vision_core vision_mapper
#  DEPENDS system_lib
)

include_directories(
# include
	${vision_core_SOURCE_DIR}
	${vision_core_SOURCE_DIR}/include
	${Caffe_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

set(common_libs
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
)
if (SEGNET_FOUND)
	list(APPEND common_libs ${Caffe_LIBRARIES})
endif()

###
add_executable(
	imagelidar_projection
		nodes/imagelidar_projection.cpp
)
target_link_libraries(
	imagelidar_projection
		${common_libs}
)
install(
	TARGETS imagelidar_projection
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###
add_executable(
	test_rgb_filters
		nodes/test_rgb_filters.cpp
)
target_link_libraries(
	test_rgb_filters
		${common_libs}
)
install(
	TARGETS test_rgb_filters
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###
add_executable(
	testbag
		nodes/testbag.cpp
)
target_link_libraries(
	testbag
		${common_libs}
)
install(
	TARGETS testbag
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

