cmake_minimum_required(VERSION 2.8.3)
project(Initialize)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge
)


find_package(OpenCV REQUIRED)

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

add_executable(Initialize
    src/main.cpp
    src/estimator.cpp
    src/feature_manager.cpp
    src/pose_local_parameterization.cpp
    src/projection_factor.cpp
    src/utility.cpp
    src/solve_5pts.cpp
    src/initial_aligment.cpp
    src/initial_sfm.cpp
    src/initial_ex_rotation.cpp
    src/marginalization_factor.cpp
    src/projection_factor.cpp
    src/parameters.cpp
    src/Initialize.cpp
   
    )


target_link_libraries(Initialize ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}  ${Sophus_LIBRARIES}) 


