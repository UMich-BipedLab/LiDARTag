cmake_minimum_required (VERSION 3.1)
project (lidartag)

# CMAKE TWEAKS
#========================================================================
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")
# set(MATLAB_INCLUDE_DIRS /usr/local/MATLAB/R2019b/extern/include/)
# set(MATLAB_LIBRARIES_DIRS /usr/local/MATLAB/R2019b/bin/glnxa64)
# set(MATLAB_LIBRARIES ${MATLAB_LIBRARIES_DIRS}/libmat.so ${MATLAB_LIBRARIES_DIRS}/libmx.so)
# list(APPEND CMAKE_PREFIX_PATH "/home/brucebot/softwares/oneTBB")
# list(APPEND CMAKE_PREFIX_PATH "/usr/local/include")
# list(APPEND CMAKE_PREFIX_PATH "/usr/local/lib/cmake")
# list(APPEND CMAKE_PREFIX_PATH "/home/brucebot/softwares/tbb_outsourced/build2")
# set(CMAKE_MODULE_PATH "/home/brucebot/softwares/TBB/build/;/usr/local/lib/;/usr/local/lib/cmake/;${CMAKE_MODULE_PATH}")
LIST(APPEND CMAKE_MODULE_PATH "/home/brucebot/workspace/catkin/src/LiDARTag/cmake/")
message(STATUS "=============================================CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

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
find_package(NLopt REQUIRED)
# PCL
find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED
             COMPONENTS filesystem system signals regex date_time program_options thread
)
# find_package(Eigen3)
#include(cmake)
# include(${CMAKE_CURRENT_SOURCE_DIR}/../A/CMakeLists.txt)
# find_package(Eigen3 HINTS /home/brucebot/softwares/eigen_git/cmake/)
# set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
# message(STATUS "=============================================Eigen path: ${EIGEN3_INCLUDE_DIR}")
# set(EIGEN3_INCLUDE_DIR "/usr/local/include/eigen3/")

message(STATUS "===============================NLOPT lib: ${NLOPT_LIBRARIES}")
find_package(TBB REQUIRED)
if(TBB_FOUND)
    message(STATUS "=============================================TBB FOUND")
    message(STATUS "===============================TBB include_dirs: ${TBB_INCLUDE_DIRS}")
    message(STATUS "===============================TBB includes: ${TBB_INCLUDES}")
    message(STATUS "===============================TBB libs: ${TBB_LIBS}")
    message(STATUS "===============================TBB libraries: ${TBB_LIBRARIES}")
    message(STATUS "===============================TBB libraries: ${TBB_LIBRARIES}")
    message(STATUS "===============================TBB libs: ${TBB_LIBS}")
    message(STATUS "===============================TBB defs: ${TBB_DEFINITIONS}")
else()
    message(STATUS "=============================================TBB NOT FOUND")
endif() 


find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  # message(STATUS "=============================================Eigen path: ${EIGEN3_INCLUDE_DIR}")
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  # message(STATUS "=============================================Eigen path: ${EIGEN3_INCLUDE_DIR}")
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
  ${TBB_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# COMPILE THE SOURCE
#========================================================================
add_executable(lidartag_main src/main.cc src/lidartag_pose.cc src/lidartag.cc src/lidartag_decode.cc src/apriltag_utils.cc src/utils.cc src/tag49h14.cc src/tag16h5.cc src/lidartag_rviz.cc src/lidartag_cluster.cc src/lidartag_prune.cc)

add_dependencies(lidartag_main ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidartag_main 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    ${NLOPT_LIBRARIES}
    ${TBB_LIBRARIES}
)
