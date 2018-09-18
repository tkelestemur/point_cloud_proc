cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_proc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	message_generation
	geometry_msgs
  	sensor_msgs
	std_msgs
  	pcl_msgs
  	pcl_ros
  	roscpp
  	roslib
  	rospy
  	tf
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY msg
	FILES
    Triangle.msg
  	Mesh.msg
	Object.msg
	Objects.msg
  	Plane.msg
	Planes.msg
)


## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY srv
	FILES
  	SinglePlaneSegmentation.srv
	MultiPlaneSegmentation.srv
	TabletopExtraction.srv
	TabletopClustering.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES point_cloud_proc
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs pcl_ros roscpp rospy sensor_msgs tf
# DEPENDS PCL
)

###########
## Build ##
###########
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#     set(CMAKE_CXX_FLAGS "-std=c++11")
# elseif(COMPILER_SUPPORTS_CXX0X)
#     set(CMAKE_CXX_FLAGS "-std=c++0x")
# else()
#     message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
# endif()
# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
include_directories(
	include
	# SYSTEM
 	${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} src/point_cloud_proc.cpp)
target_link_libraries(point_cloud_proc ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(point_cloud_proc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} point_cloud_proc_generate_messages_cpp)

add_executable(test_single_plane tests/test_single_plane.cpp)
target_link_libraries(test_single_plane point_cloud_proc ${catkin_LIBRARIES})

add_executable(test_multi_plane tests/test_multi_plane.cpp)
target_link_libraries(test_multi_plane point_cloud_proc ${catkin_LIBRARIES})

add_executable(test_tabletop_cluster tests/test_tabletop_cluster.cpp)
target_link_libraries(test_tabletop_cluster point_cloud_proc ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide


#############
## Install ##
#############
#install(TARGETS ${PROJECT_NAME}
#	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)
#
#install(DIRECTORY include/${PROJECT_NAME}/
#	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})