cmake_minimum_required(VERSION 3.0.2)
project(im_enhance)

add_compile_options(-std=c++11)

find_package (OpenCV REQUIRED)
find_package (PythonLibs REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (pybind11 REQUIRED)
find_package (OpenMP REQUIRED)
find_package (Boost REQUIRED COMPONENTS filesystem)

find_package (catkin COMPONENTS
	roscpp
	sensor_msgs
	cv_bridge
	image_transport
)

if (OPENMP_FOUND)
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
	LIBRARIES im_enhance
	DEPENDS OpenCV
)

include_directories(
	include
	${OpenCV_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_library(im_enhance
	src/im_enhance.cpp
	src/retinex.cpp
	src/expfusion.cpp
)

target_link_libraries(
	im_enhance
		${OpenCV_LIBS}
)

install(
	TARGETS im_enhance
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
)

add_executable(
	test_ime
		main.cpp
)

target_link_libraries(test_ime
	im_enhance
	Boost::system
	Boost::filesystem
)

pybind11_add_module(
	ime_py MODULE
		python/mod.cpp
)

install(
	FILES src/ying.py
	DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
)

if (catkin_FOUND)
	add_executable(ime_ros
		src/ime_ros.cpp)
	target_link_libraries(ime_ros
		im_enhance
		${catkin_LIBRARIES})
endif()


