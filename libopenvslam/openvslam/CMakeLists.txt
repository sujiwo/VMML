# ----- Find dependencies -----

# json
set(json_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/json/include)

# ----- Create OpenVSLAM library -----

add_library(openvslam
            SHARED
            ${CMAKE_CURRENT_SOURCE_DIR}/config.h
            ${CMAKE_CURRENT_SOURCE_DIR}/type.h
            ${CMAKE_CURRENT_SOURCE_DIR}/system.h
            ${CMAKE_CURRENT_SOURCE_DIR}/tracking_module.h
            ${CMAKE_CURRENT_SOURCE_DIR}/mapping_module.h
            ${CMAKE_CURRENT_SOURCE_DIR}/global_optimization_module.h
            ${CMAKE_CURRENT_SOURCE_DIR}/config.cc
            ${CMAKE_CURRENT_SOURCE_DIR}/system.cc
            ${CMAKE_CURRENT_SOURCE_DIR}/tracking_module.cc
            ${CMAKE_CURRENT_SOURCE_DIR}/mapping_module.cc
            ${CMAKE_CURRENT_SOURCE_DIR}/global_optimization_module.cc)
            
# Set output directory of the library
set_target_properties(openvslam PROPERTIES
                      OUTPUT_NAME openvslam
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# ----- Compile configuration -----

# OpenMP
set(USE_OPENMP OFF CACHE BOOL "Use OpenMP")
find_package(OpenMP)
if(OpenMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	target_compile_definitions(openvslam PUBLIC USE_OPENMP)
	message(STATUS "OpenMP: ENABLED")
else()
	message(STATUS "OpenMP: DISABLED")
endif()


# ----- Configure OpenVSLAM library -----
target_compile_definitions(openvslam PUBLIC -DUSE_DBOW2)

# Include directories
#target_include_directories(openvslam
#                           PUBLIC
#                           ${G2O_INCLUDE_DIR}
#                           $<BUILD_INTERFACE:${json_INCLUDE_DIR}>
#                           $<BUILD_INTERFACE:${spdlog_INCLUDE_DIR}>
#                           $<BUILD_INTERFACE:${BoW_INCLUDE_DIR}>
#                           ${PROJECT_SOURCE_DIR}
#                           $<INSTALL_INTERFACE:include/openvslam/3rd/json/include>
#                           $<INSTALL_INTERFACE:include/openvslam/3rd/spdlog/include>
#                           $<INSTALL_INTERFACE:${BoW_INCLUDE_DIR}>
#                           $<INSTALL_INTERFACE:include/>)

# Link to required libraries
target_link_libraries(openvslam
                      PUBLIC
                      DBoW2
                      Threads::Threads
                      Eigen3::Eigen
                      yaml-cpp
                      opencv_core
                      opencv_features2d
                      opencv_calib3d
                      ${G2O_LIBRARIES}
                      PRIVATE
                      ${LAPACK_LIBRARIES}
)

# ----- Install configuration -----

set(OPENVSLAM_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/openvslam)

# Install OpenVSLAM library
install(TARGETS openvslam
        EXPORT ${OPENVSLAM_TARGETS_EXPORT_NAME}
        RUNTIME DESTINATION ${RUNTIME_DESTINATION}
        LIBRARY DESTINATION ${LIBRARY_DESTINATION}
        ARCHIVE DESTINATION ${ARCHIVE_DESTINATION})

# Install OpenVSLAM headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR})

# Install 3rd party headers
install(FILES ${json_INCLUDE_DIR}/nlohmann/json.hpp ${json_INCLUDE_DIR}/nlohmann/json_fwd.hpp
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/3rd/json/include/nlohmann)

# ----- Append subdirectory -----

add_subdirectory(camera)
add_subdirectory(data)
add_subdirectory(feature)
add_subdirectory(initialize)
add_subdirectory(io)
add_subdirectory(match)
add_subdirectory(module)
add_subdirectory(optimize)
add_subdirectory(publish)
add_subdirectory(solve)
add_subdirectory(util)
