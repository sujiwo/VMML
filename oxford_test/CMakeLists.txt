cmake_minimum_required(VERSION 3.0.2)
project(oxford_test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rosbag_viewer vision_core vision_mapper)
find_package(Boost REQUIRED COMPONENTS system filesystem)

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
#  LIBRARIES oxford_test
	CATKIN_DEPENDS vision_core vision_mapper
#  DEPENDS system_lib
)

include_directories(
# include
	${Caffe_INCLUDE_DIRS}
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

add_executable(viewer
	viewer.cpp
	OxfordDataset.cpp
)

target_link_libraries(
	viewer
		generic_imageset_viewer
		${catkin_LIBRARIES}
		Boost::system
)

add_executable(oxf_index_mapper
	index_mapper.cpp
	OxfordDataset.cpp
)

target_link_libraries(
	oxf_index_mapper
		${catkin_LIBRARIES}
		${Caffe_LIBRARIES}
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

install(
	TARGETS viewer
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)