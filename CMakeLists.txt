cmake_minimum_required (VERSION 3.5)
project (lidartag)

# CMAKE TWEAKS
#========================================================================
set(CMAKE_CXX_STANDARD 17)
#set( CMAKE_VERBOSE_MAKEFILE on )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
message(STATUS "=============================================CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

find_package(ament_cmake_auto REQUIRED)

# CHECK THE DEPENDENCIES
find_package(NLopt REQUIRED)

find_package(Boost REQUIRED COMPONENTS atomic filesystem system regex date_time program_options thread)

message(STATUS "=============================================Boost")
message(STATUS "===============================Boost include_dirs: ${Boost_INCLUDE_DIRS}")
message(STATUS "===============================Boost libraries: ${Boost_LIBRARIES}")

# NLOPT
message(STATUS "===============================NLOPT lib: ${NLOPT_LIBRARIES}")

# TBB
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


message(STATUS "Trying to include Eigen library")
set(EIGEN3_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/internal_eigen3/eigen)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3 3.4.0 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "================================== EIGEN (internal): " ${EIGEN3_VERSION})

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

ament_auto_find_build_dependencies() # This makes the find packages from the xml redundant


ament_export_include_directories(
#include_directories(
	include 
	${catkin_INCLUDE_DIRS} 
	#${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
)

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS}) # TODO KL: we may still need this

# COMPILE THE SOURCE
#========================================================================
ament_auto_add_executable(lidartag_main 
    src/main.cc 
    src/lidartag_pose.cc 
    src/lidartag.cc 
    src/lidartag_decode.cc 
    src/apriltag_utils.cc 
    src/utils.cc 
    src/tag49h14.cc 
    src/tag16h5.cc 
    src/lidartag_rviz.cc 
    src/lidartag_cluster.cc 
    src/lidartag_prune.cc)

target_link_libraries(lidartag_main 
    #${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${NLOPT_LIBRARIES}
    ${TBB_LIBRARIES}
    ${Boost_LIBRARIES}
    Boost::thread # Thos should be redundant from the last line (${Boost_LIBRARIES}) but if not explicitly added, CMake will not link against it...
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


#ament_package() # This seems to be at the end
ament_auto_package(
  INSTALL_TO_SHARE
    launch
    lib
    outputs
)
