cmake_minimum_required(VERSION 2.8)
project(DBoW2)

add_compile_options(-std=c++11 -frtti -fopenmp)

find_package (OpenCV REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
)

add_library(DBoW2
	BowVector.cpp
	FORB.cpp      
	FeatureVector.cpp
	ScoringObject.cpp
	DUtils/Random.cpp
	DUtils/Timestamp.cpp
)

target_link_libraries(DBoW2 ${OpenCV_LIBS})
