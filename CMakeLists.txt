cmake_minimum_required(VERSION 3.1)
project(convert_rosbag)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -march=native -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  rosbag
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

add_executable(parse_bag src/parse_topic.cpp)
target_link_libraries(parse_bag ${catkin_LIBRARIES} ${PCL_LIBRARIES})