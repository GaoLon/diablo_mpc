cmake_minimum_required(VERSION 2.8.3)
project(mpc)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  visualization_msgs
  diablo_sdk
)
find_package(Eigen3 REQUIRED)
# find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs diablo_sdk
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
)

add_executable (mpc src/mpc.cpp src/cubic_spline_planner.cpp)
target_link_libraries(mpc
    # PRIVATE osqp::osqp
    OsqpEigen::OsqpEigen
    ${catkin_LIBRARIES}
    )
