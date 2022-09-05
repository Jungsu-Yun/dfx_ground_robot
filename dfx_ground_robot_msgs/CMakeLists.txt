cmake_minimum_required(VERSION 3.0.2)
project(dfx_ground_robot_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    SensorState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)