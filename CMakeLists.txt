cmake_minimum_required(VERSION 3.5)
project(lidartag)

# CMAKE TWEAKS
#========================================================================
set(CMAKE_CXX_STANDARD 17)
#set( CMAKE_VERBOSE_MAKEFILE on )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -DEIGEN_NO_DEBUG -Wl,--no-as-needed")
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
    message(STATUS "===============================TBB defs: ${TBB_DEFINITIONS}")
else()
    message(STATUS "=============================================TBB NOT FOUND")
endif()

### Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()


ament_export_include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
)


# COMPILE THE SOURCE
#========================================================================
ament_auto_add_executable(lidartag_main
    src/main.cpp
    src/lidartag_pose.cpp
    src/lidartag.cpp
    src/lidartag_decode.cpp
    src/apriltag_utils.cpp
    src/utils.cpp
    src/tag49h14.cpp
    src/tag16h5.cpp
    src/lidartag_rviz.cpp
    src/lidartag_cluster.cpp
    src/lidartag_prune.cpp
    src/rectangle_estimator.cpp
    src/decoding/naive_hamming_decoding.cpp)

target_link_libraries(lidartag_main
    ${PCL_LIBRARIES}
    ${NLOPT_LIBRARIES}
    ${TBB_LIBRARIES}
    ${Boost_LIBRARIES}
    Boost::thread
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  #find_package(OpenCV REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  )

  # Unit tests
  set(TEST_SOURCES
    #test/test_rectangle_estimation.cpp
    #src/rectangle_estimator.cpp
    test/test_hamming_decoding.cpp
    src/decoding/naive_hamming_decoding.cpp
  )

  ament_add_gtest(test_decoding ${TEST_SOURCES})

  target_link_libraries(test_decoding
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

endif()


ament_auto_package(
  INSTALL_TO_SHARE
    launch
    lib
    outputs
    rviz
)
