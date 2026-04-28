# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ur5_kinematics: 0 messages, 1 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ur5_kinematics_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jiaid/ur5_rehab_ws/src/ur5_kinematics/srv/IkService.srv" NAME_WE)
add_custom_target(_ur5_kinematics_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur5_kinematics" "/home/jiaid/ur5_rehab_ws/src/ur5_kinematics/srv/IkService.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(ur5_kinematics
  "/home/jiaid/ur5_rehab_ws/src/ur5_kinematics/srv/IkService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur5_kinematics
)

### Generating Module File
_generate_module_cpp(ur5_kinematics
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur5_kinematics
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ur5_kinematics_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ur5_kinematics_generate_messages ur5_kinematics_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jiaid/ur5_rehab_ws/src/ur5_kinematics/srv/IkService.srv" NAME_WE)
add_dependencies(ur5_kinematics_generate_messages_cpp _ur5_kinematics_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur5_kinematics_gencpp)
add_dependencies(ur5_kinematics_gencpp ur5_kinematics_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur5_kinematics_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(ur5_kinematics
  "/home/jiaid/ur5_rehab_ws/src/ur5_kinematics/srv/IkService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur5_kinematics
)

### Generating Module File
_generate_module_eus(ur5_kinematics
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur5_kinematics
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ur5_kinematics_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ur5_kinematics_generate_messages ur5_kinematics_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jiaid/ur5_rehab_ws/src/ur5_kinematics/srv/IkService.srv" NAME_WE)
add_dependencies(ur5_kinematics_generate_messages_eus _ur5_kinematics_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur5_kinematics_geneus)
add_dependencies(ur5_kinematics_geneus ur5_kinematics_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur5_kinematics_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(ur5_kinematics
  "/home/jiaid/ur5_rehab_ws/src/ur5_kinematics/srv/IkService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur5_kinematics
)

### Generating Module File
_generate_module_lisp(ur5_kinematics
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur5_kinematics
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ur5_kinematics_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ur5_kinematics_generate_messages ur5_kinematics_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jiaid/ur5_rehab_ws/src/ur5_kinematics/srv/IkService.srv" NAME_WE)
add_dependencies(ur5_kinematics_generate_messages_lisp _ur5_kinematics_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur5_kinematics_genlisp)
add_dependencies(ur5_kinematics_genlisp ur5_kinematics_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur5_kinematics_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(ur5_kinematics
  "/home/jiaid/ur5_rehab_ws/src/ur5_kinematics/srv/IkService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur5_kinematics
)

### Generating Module File
_generate_module_nodejs(ur5_kinematics
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur5_kinematics
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ur5_kinematics_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ur5_kinematics_generate_messages ur5_kinematics_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jiaid/ur5_rehab_ws/src/ur5_kinematics/srv/IkService.srv" NAME_WE)
add_dependencies(ur5_kinematics_generate_messages_nodejs _ur5_kinematics_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur5_kinematics_gennodejs)
add_dependencies(ur5_kinematics_gennodejs ur5_kinematics_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur5_kinematics_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(ur5_kinematics
  "/home/jiaid/ur5_rehab_ws/src/ur5_kinematics/srv/IkService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_kinematics
)

### Generating Module File
_generate_module_py(ur5_kinematics
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_kinematics
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ur5_kinematics_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ur5_kinematics_generate_messages ur5_kinematics_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jiaid/ur5_rehab_ws/src/ur5_kinematics/srv/IkService.srv" NAME_WE)
add_dependencies(ur5_kinematics_generate_messages_py _ur5_kinematics_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur5_kinematics_genpy)
add_dependencies(ur5_kinematics_genpy ur5_kinematics_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur5_kinematics_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur5_kinematics)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur5_kinematics
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(ur5_kinematics_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(ur5_kinematics_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ur5_kinematics_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur5_kinematics)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur5_kinematics
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(ur5_kinematics_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(ur5_kinematics_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ur5_kinematics_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur5_kinematics)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur5_kinematics
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(ur5_kinematics_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(ur5_kinematics_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ur5_kinematics_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur5_kinematics)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur5_kinematics
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(ur5_kinematics_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(ur5_kinematics_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ur5_kinematics_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_kinematics)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_kinematics\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_kinematics
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_kinematics")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur5_kinematics
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(ur5_kinematics_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(ur5_kinematics_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ur5_kinematics_generate_messages_py std_msgs_generate_messages_py)
endif()
