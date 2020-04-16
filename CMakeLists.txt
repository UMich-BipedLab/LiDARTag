cmake_minimum_required (VERSION 2.8.3)
project (lidartag)

# CMAKE TWEAKS
#========================================================================
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")
set(MATLAB_INCLUDE_DIRS /usr/local/MATLAB/R2019b/extern/include/)
set(MATLAB_LIBRARIES_DIRS /usr/local/MATLAB/R2019b/bin/glnxa64)
set(MATLAB_LIBRARIES ${MATLAB_LIBRARIES_DIRS}/libmat.so ${MATLAB_LIBRARIES_DIRS}/libmx.so)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    tf
    sensor_msgs
    visualization_msgs
    message_filters
    std_msgs
    velodyne_pointcloud
    roslib
    lidartag_msgs
)
# CHECK THE DEPENDENCIES
# PCL
find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED
             COMPONENTS filesystem system signals regex date_time program_options thread
)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  message_filters 
	roscpp 
	sensor_msgs
  std_msgs 
	tf
  DEPENDS
	Eigen3
)

include_directories(
	include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# COMPILE THE SOURCE
#========================================================================
add_executable(lidartag_main src/main.cc src/lidartag.cc src/lidartag_decode.cc src/apriltag_utils.cc src/utils.cc src/tag49h14.cc src/tag16h5.cc src/lidartag_rviz.cc src/lidartag_cluster.cc src/lidartag_prune.cc)

add_dependencies(lidartag_main ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidartag_main 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
)
