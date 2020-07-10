cmake_minimum_required(VERSION 3.0.2)
project(im_enhance)

add_compile_options(-std=c++11)

find_package (OpenCV REQUIRED)
find_package (PythonLibs REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (pybind11 REQUIRED)

catkin_package(
	LIBRARIES im_enhance
	DEPENDS OpenCV
)

include_directories(
	include
	${OpenCV_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

add_library(im_enhance
	src/im_enhance.cpp
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
)

pybind11_add_module(
	ime_py MODULE
		python/mod.cpp
)