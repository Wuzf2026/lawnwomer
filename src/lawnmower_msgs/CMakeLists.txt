cmake_minimum_required(VERSION 3.10)
project(lawnmower_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(FILES PlatformState.msg)
add_service_files(FILES DeviceCheck.srv)

    catkin_package(CATKIN_DEPENDS message_runtime std_msgs)