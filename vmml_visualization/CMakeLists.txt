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
find_package(libviso2)
find_package(libopenvslam)

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
		nodes/sift/sift.cpp
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

###
if (libviso2_FOUND)
	message("Viso2-based visual odometry will be built")
	add_executable(
		viso_run
			nodes/viso_run.cpp
	)
	target_include_directories(viso_run PUBLIC ${libviso2_INCLUDE_DIRS})
	target_link_libraries(
		viso_run
			${common_libs}
			${libviso2_LIBRARIES}
	)
	install(
		TARGETS viso_run
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
else()
	message("Viso2-based visual odometry will NOT be built")
endif()

###
if (libopenvslam_FOUND)
	message("OpenVSlam demo will be built")
	find_package(yaml-cpp REQUIRED)
	add_executable(
		vosl_demo
			nodes/vosl_demo.cpp
	)
	target_include_directories(vosl_demo PUBLIC ${libopenvslam_INCLUDE_DIRS})
	target_link_libraries(
		vosl_demo
			${common_libs}
			${libopenvslam_LIBRARIES}
	)
	install(
		TARGETS vosl_demo
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
	
	# Place Recognizer Service Node
	add_executable(
		vosl_place_recognizer_srv
			nodes/vosl_place_recognizer_srv.cpp
	)
	target_include_directories(vosl_place_recognizer_srv PUBLIC ${libopenvslam_INCLUDE_DIRS})
	target_link_libraries(
		vosl_place_recognizer_srv
			${common_libs}
			${libopenvslam_LIBRARIES}
	)
	install(
		TARGETS vosl_place_recognizer_srv
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
	
else()
	message("OpenVSlam demo will NOT be built")
endif()


###
#add_executable(
#	openvideo
#		nodes/openvideo.cpp
#		nodes/VideoImageBag.cpp
#)
#target_link_libraries(
#	openvideo
#		${common_libs}
#)
