# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "optoforce_ros: 7 messages, 0 services")

set(MSG_I_FLAGS "-Ioptoforce_ros:/home/wfh/catkin_ws/devel/share/optoforce_ros/msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(optoforce_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceAction.msg" NAME_WE)
add_custom_target(_optoforce_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optoforce_ros" "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceAction.msg" "optoforce_ros/OptoForceActionGoal:geometry_msgs/Vector3:std_msgs/Header:actionlib_msgs/GoalStatus:optoforce_ros/OptoForceFeedback:optoforce_ros/OptoForceActionResult:optoforce_ros/OptoForceGoal:geometry_msgs/Wrench:actionlib_msgs/GoalID:geometry_msgs/WrenchStamped:optoforce_ros/OptoForceResult:optoforce_ros/OptoForceActionFeedback"
)

get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg" NAME_WE)
add_custom_target(_optoforce_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optoforce_ros" "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg" "optoforce_ros/OptoForceGoal:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg" NAME_WE)
add_custom_target(_optoforce_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optoforce_ros" "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg" "actionlib_msgs/GoalStatus:std_msgs/Header:optoforce_ros/OptoForceResult:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg" NAME_WE)
add_custom_target(_optoforce_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optoforce_ros" "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg" "geometry_msgs/Vector3:actionlib_msgs/GoalStatus:std_msgs/Header:optoforce_ros/OptoForceFeedback:geometry_msgs/Wrench:actionlib_msgs/GoalID:geometry_msgs/WrenchStamped"
)

get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg" NAME_WE)
add_custom_target(_optoforce_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optoforce_ros" "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg" ""
)

get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg" NAME_WE)
add_custom_target(_optoforce_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optoforce_ros" "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg" ""
)

get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg" NAME_WE)
add_custom_target(_optoforce_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "optoforce_ros" "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg" "geometry_msgs/WrenchStamped:geometry_msgs/Wrench:std_msgs/Header:geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceAction.msg"
  "${MSG_I_FLAGS}"
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optoforce_ros
)
_generate_msg_cpp(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optoforce_ros
)
_generate_msg_cpp(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optoforce_ros
)
_generate_msg_cpp(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optoforce_ros
)
_generate_msg_cpp(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optoforce_ros
)
_generate_msg_cpp(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optoforce_ros
)
_generate_msg_cpp(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optoforce_ros
)

### Generating Services

### Generating Module File
_generate_module_cpp(optoforce_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optoforce_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(optoforce_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(optoforce_ros_generate_messages optoforce_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceAction.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_cpp _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_cpp _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_cpp _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_cpp _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_cpp _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_cpp _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_cpp _optoforce_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(optoforce_ros_gencpp)
add_dependencies(optoforce_ros_gencpp optoforce_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS optoforce_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceAction.msg"
  "${MSG_I_FLAGS}"
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optoforce_ros
)
_generate_msg_eus(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optoforce_ros
)
_generate_msg_eus(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optoforce_ros
)
_generate_msg_eus(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optoforce_ros
)
_generate_msg_eus(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optoforce_ros
)
_generate_msg_eus(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optoforce_ros
)
_generate_msg_eus(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optoforce_ros
)

### Generating Services

### Generating Module File
_generate_module_eus(optoforce_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optoforce_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(optoforce_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(optoforce_ros_generate_messages optoforce_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceAction.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_eus _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_eus _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_eus _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_eus _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_eus _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_eus _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_eus _optoforce_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(optoforce_ros_geneus)
add_dependencies(optoforce_ros_geneus optoforce_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS optoforce_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceAction.msg"
  "${MSG_I_FLAGS}"
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optoforce_ros
)
_generate_msg_lisp(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optoforce_ros
)
_generate_msg_lisp(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optoforce_ros
)
_generate_msg_lisp(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optoforce_ros
)
_generate_msg_lisp(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optoforce_ros
)
_generate_msg_lisp(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optoforce_ros
)
_generate_msg_lisp(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optoforce_ros
)

### Generating Services

### Generating Module File
_generate_module_lisp(optoforce_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optoforce_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(optoforce_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(optoforce_ros_generate_messages optoforce_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceAction.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_lisp _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_lisp _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_lisp _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_lisp _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_lisp _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_lisp _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_lisp _optoforce_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(optoforce_ros_genlisp)
add_dependencies(optoforce_ros_genlisp optoforce_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS optoforce_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceAction.msg"
  "${MSG_I_FLAGS}"
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optoforce_ros
)
_generate_msg_nodejs(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optoforce_ros
)
_generate_msg_nodejs(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optoforce_ros
)
_generate_msg_nodejs(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optoforce_ros
)
_generate_msg_nodejs(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optoforce_ros
)
_generate_msg_nodejs(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optoforce_ros
)
_generate_msg_nodejs(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optoforce_ros
)

### Generating Services

### Generating Module File
_generate_module_nodejs(optoforce_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optoforce_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(optoforce_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(optoforce_ros_generate_messages optoforce_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceAction.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_nodejs _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_nodejs _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_nodejs _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_nodejs _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_nodejs _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_nodejs _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_nodejs _optoforce_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(optoforce_ros_gennodejs)
add_dependencies(optoforce_ros_gennodejs optoforce_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS optoforce_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceAction.msg"
  "${MSG_I_FLAGS}"
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optoforce_ros
)
_generate_msg_py(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optoforce_ros
)
_generate_msg_py(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optoforce_ros
)
_generate_msg_py(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optoforce_ros
)
_generate_msg_py(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optoforce_ros
)
_generate_msg_py(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optoforce_ros
)
_generate_msg_py(optoforce_ros
  "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optoforce_ros
)

### Generating Services

### Generating Module File
_generate_module_py(optoforce_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optoforce_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(optoforce_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(optoforce_ros_generate_messages optoforce_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceAction.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_py _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionGoal.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_py _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionResult.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_py _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceActionFeedback.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_py _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceGoal.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_py _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceResult.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_py _optoforce_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wfh/catkin_ws/devel/share/optoforce_ros/msg/OptoForceFeedback.msg" NAME_WE)
add_dependencies(optoforce_ros_generate_messages_py _optoforce_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(optoforce_ros_genpy)
add_dependencies(optoforce_ros_genpy optoforce_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS optoforce_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optoforce_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/optoforce_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(optoforce_ros_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(optoforce_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(optoforce_ros_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optoforce_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/optoforce_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(optoforce_ros_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(optoforce_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(optoforce_ros_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optoforce_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/optoforce_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(optoforce_ros_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(optoforce_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(optoforce_ros_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optoforce_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/optoforce_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(optoforce_ros_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(optoforce_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(optoforce_ros_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optoforce_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optoforce_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/optoforce_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(optoforce_ros_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(optoforce_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(optoforce_ros_generate_messages_py geometry_msgs_generate_messages_py)
endif()
