cmake_minimum_required(VERSION 3.0.2)
project(im_enhance)

add_compile_options(-std=c++11)

find_package (OpenCV 3 REQUIRED)
find_package (PythonLibs REQUIRED)
find_package (pybind11 REQUIRED)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

add_library(im_enhance
	im_enhance.cpp
)

target_link_libraries(
	im_enhance
		${OpenCV_LIBS}
)