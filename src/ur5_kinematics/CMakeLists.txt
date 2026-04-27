cmake_minimum_required(VERSION 3.0.2)
project(ur5_kinematics)

add_compile_options(-std=c++11)

catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  tf2_geometry_msgs
  kdl_parser
  nav_msgs
  message_generation # 新增，用来编译服务
)

find_path(KDL_INCLUDE_DIR
  NAMES kdl/chain.hpp
  PATHS /usr/include /usr/local/include
)

find_library(KDL_LIBRARY
  NAMES orocos-kdl
  PATHS /lib /usr/lib /usr/local/lib
)

if(NOT KDL_INCLUDE_DIR OR NOT KDL_LIBRARY)
  message(FATAL_ERROR "System Orocos KDL not found. Please install liborocos-kdl-dev.")
endif()

# 新增：添加服务文件
add_service_files(
  FILES
  IkService.srv
)

# 新增：生成服务消息
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime # 新增
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${KDL_INCLUDE_DIR}
)

# 原来的节点
add_executable(ur5_fk_node src/ur5_fk_node.cpp)
target_link_libraries(ur5_fk_node ${catkin_LIBRARIES} ${KDL_LIBRARY})

add_executable(ur5_ik_node src/ur5_ik_node.cpp)
target_link_libraries(ur5_ik_node ${catkin_LIBRARIES} ${KDL_LIBRARY})
add_dependencies(ur5_ik_node ${PROJECT_NAME}_gencpp) # 新增，依赖服务生成

# 新的轨迹节点
add_executable(rehab_trajectory_node src/rehab_trajectory_node.cpp)
target_link_libraries(rehab_trajectory_node ${catkin_LIBRARIES})
add_dependencies(rehab_trajectory_node ${PROJECT_NAME}_gencpp) # 新增，依赖服务生成