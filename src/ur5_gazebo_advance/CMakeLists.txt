cmake_minimum_required(VERSION 3.0.2)
project(ur5_gazebo_advance)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/ur5_admittance_fixed_point_controller.py
  scripts/ur5_periodic_wrench_publisher.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
