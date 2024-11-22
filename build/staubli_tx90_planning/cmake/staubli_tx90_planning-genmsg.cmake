# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "staubli_tx90_planning: 1 messages, 0 services")

set(MSG_I_FLAGS "-Istaubli_tx90_planning:/home/wfh/catkin_ws/src/staubli_tx90_planning/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(staubli_tx90_planning_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wfh/catkin_ws/src/staubli_tx90_planning/msg/Coordinate_force.msg" NAME_WE)
add_custom_target(_staubli_tx90_planning_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "staubli_tx90_planning" "/home/wfh/catkin_ws/src/staubli_tx90_planning/msg/Coordinate_force.msg" "std_msgs/Header:geometry_msgs/Point:geometry_msgs/Wrench:geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(staubli_tx90_planning
  "/home/wfh/catkin_ws/src/staubli_tx90_planning/msg/Coordinate_force.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/staubli_tx90_planning
)

### Generating Services

### Generating Module File
_generate_module_cpp(staubli_tx90_planning
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/staubli_tx90_planning
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(staubli_tx90_planning_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(staubli_tx90_planning_generate_messages staubli_tx90_planning_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wfh/catkin_ws/src/staubli_tx90_planning/msg/Coordinate_force.msg" NAME_WE)
add_dependencies(staubli_tx90_planning_generate_messages_cpp _staubli_tx90_planning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(staubli_tx90_planning_gencpp)
add_dependencies(staubli_tx90_planning_gencpp staubli_tx90_planning_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS staubli_tx90_planning_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(staubli_tx90_planning
  "/home/wfh/catkin_ws/src/staubli_tx90_planning/msg/Coordinate_force.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/staubli_tx90_planning
)

### Generating Services

### Generating Module File
_generate_module_eus(staubli_tx90_planning
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/staubli_tx90_planning
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(staubli_tx90_planning_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(staubli_tx90_planning_generate_messages staubli_tx90_planning_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wfh/catkin_ws/src/staubli_tx90_planning/msg/Coordinate_force.msg" NAME_WE)
add_dependencies(staubli_tx90_planning_generate_messages_eus _staubli_tx90_planning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(staubli_tx90_planning_geneus)
add_dependencies(staubli_tx90_planning_geneus staubli_tx90_planning_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS staubli_tx90_planning_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(staubli_tx90_planning
  "/home/wfh/catkin_ws/src/staubli_tx90_planning/msg/Coordinate_force.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/staubli_tx90_planning
)

### Generating Services

### Generating Module File
_generate_module_lisp(staubli_tx90_planning
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/staubli_tx90_planning
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(staubli_tx90_planning_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(staubli_tx90_planning_generate_messages staubli_tx90_planning_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wfh/catkin_ws/src/staubli_tx90_planning/msg/Coordinate_force.msg" NAME_WE)
add_dependencies(staubli_tx90_planning_generate_messages_lisp _staubli_tx90_planning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(staubli_tx90_planning_genlisp)
add_dependencies(staubli_tx90_planning_genlisp staubli_tx90_planning_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS staubli_tx90_planning_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(staubli_tx90_planning
  "/home/wfh/catkin_ws/src/staubli_tx90_planning/msg/Coordinate_force.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/staubli_tx90_planning
)

### Generating Services

### Generating Module File
_generate_module_nodejs(staubli_tx90_planning
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/staubli_tx90_planning
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(staubli_tx90_planning_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(staubli_tx90_planning_generate_messages staubli_tx90_planning_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wfh/catkin_ws/src/staubli_tx90_planning/msg/Coordinate_force.msg" NAME_WE)
add_dependencies(staubli_tx90_planning_generate_messages_nodejs _staubli_tx90_planning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(staubli_tx90_planning_gennodejs)
add_dependencies(staubli_tx90_planning_gennodejs staubli_tx90_planning_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS staubli_tx90_planning_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(staubli_tx90_planning
  "/home/wfh/catkin_ws/src/staubli_tx90_planning/msg/Coordinate_force.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/staubli_tx90_planning
)

### Generating Services

### Generating Module File
_generate_module_py(staubli_tx90_planning
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/staubli_tx90_planning
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(staubli_tx90_planning_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(staubli_tx90_planning_generate_messages staubli_tx90_planning_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wfh/catkin_ws/src/staubli_tx90_planning/msg/Coordinate_force.msg" NAME_WE)
add_dependencies(staubli_tx90_planning_generate_messages_py _staubli_tx90_planning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(staubli_tx90_planning_genpy)
add_dependencies(staubli_tx90_planning_genpy staubli_tx90_planning_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS staubli_tx90_planning_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/staubli_tx90_planning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/staubli_tx90_planning
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(staubli_tx90_planning_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/staubli_tx90_planning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/staubli_tx90_planning
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(staubli_tx90_planning_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/staubli_tx90_planning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/staubli_tx90_planning
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(staubli_tx90_planning_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/staubli_tx90_planning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/staubli_tx90_planning
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(staubli_tx90_planning_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/staubli_tx90_planning)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/staubli_tx90_planning\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/staubli_tx90_planning
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(staubli_tx90_planning_generate_messages_py geometry_msgs_generate_messages_py)
endif()
