cmake_minimum_required(VERSION 3.0.2)
project(ur5_gazebo_advance)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  trajectory_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS gazebo_msgs geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf2_ros trajectory_msgs visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/ur5_active_rehab_admittance_controller.py
  scripts/ur5_admittance_fixed_point_controller.py
  scripts/ur5_circular_reference_generator.py
  scripts/ur5_command_wrench_error_monitor.py
  scripts/ur5_delayed_tare_trigger.py
  scripts/ur5_periodic_wrench_publisher.py
  scripts/ur5_rehab_stability_line_controller.py
  scripts/ur5_rehab_tracking_error_monitor.py
  scripts/ur5_plot_active_rehab_bag.py
  scripts/ur5_pulse_wrench_publisher.py
  scripts/ur5_force_marker_publisher.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)
