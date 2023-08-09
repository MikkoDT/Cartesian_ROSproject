cmake_minimum_required(VERSION 3.0.2)
project(cart_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(

)


include_directories(

  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
  scripts/simple_publisher.py
  scripts/simple_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)