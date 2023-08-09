cmake_minimum_required(VERSION 3.0.2)
project(cart_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  controller_manager
  hardware_interface
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cart_interface_node src/cart_interface.cpp)
add_dependencies(cart_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cart_interface_node ${catkin_LIBRARIES})