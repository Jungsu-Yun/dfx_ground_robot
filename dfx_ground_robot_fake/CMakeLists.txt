cmake_minimum_required(VERSION 3.0.2)
project(dfx_ground_robot_fake)

find_package(catkin REQUIRED COMPONENTS
  dfx_ground_robot_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dfx_ground_robot_msgs geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/dfx_ground_robot_fake_node.cpp src/DfxGroundRobotFake.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})