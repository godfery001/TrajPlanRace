# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cav_msgs: 9 messages, 0 services")

set(MSG_I_FLAGS "-Icav_msgs:/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cav_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Control.msg" NAME_WE)
add_custom_target(_cav_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cav_msgs" "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Control.msg" ""
)

get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/VehicleState.msg" NAME_WE)
add_custom_target(_cav_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cav_msgs" "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/VehicleState.msg" ""
)

get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Gpybm.msg" NAME_WE)
add_custom_target(_cav_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cav_msgs" "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Gpybm.msg" ""
)

get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/PlanedPath.msg" NAME_WE)
add_custom_target(_cav_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cav_msgs" "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/PlanedPath.msg" "cav_msgs/Point"
)

get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg" NAME_WE)
add_custom_target(_cav_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cav_msgs" "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg" ""
)

get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg" NAME_WE)
add_custom_target(_cav_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cav_msgs" "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg" ""
)

get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Goal.msg" NAME_WE)
add_custom_target(_cav_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cav_msgs" "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Goal.msg" "cav_msgs/RefPoint"
)

get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg" NAME_WE)
add_custom_target(_cav_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cav_msgs" "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg" ""
)

get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/ObstacleVec.msg" NAME_WE)
add_custom_target(_cav_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cav_msgs" "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/ObstacleVec.msg" "cav_msgs/Obstacle"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cav_msgs
)
_generate_msg_cpp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cav_msgs
)
_generate_msg_cpp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Gpybm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cav_msgs
)
_generate_msg_cpp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/PlanedPath.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cav_msgs
)
_generate_msg_cpp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cav_msgs
)
_generate_msg_cpp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cav_msgs
)
_generate_msg_cpp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cav_msgs
)
_generate_msg_cpp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cav_msgs
)
_generate_msg_cpp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/ObstacleVec.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cav_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(cav_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cav_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cav_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cav_msgs_generate_messages cav_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Control.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_cpp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/VehicleState.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_cpp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Gpybm.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_cpp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/PlanedPath.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_cpp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_cpp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_cpp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Goal.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_cpp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_cpp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/ObstacleVec.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_cpp _cav_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cav_msgs_gencpp)
add_dependencies(cav_msgs_gencpp cav_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cav_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cav_msgs
)
_generate_msg_eus(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cav_msgs
)
_generate_msg_eus(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Gpybm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cav_msgs
)
_generate_msg_eus(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/PlanedPath.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cav_msgs
)
_generate_msg_eus(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cav_msgs
)
_generate_msg_eus(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cav_msgs
)
_generate_msg_eus(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cav_msgs
)
_generate_msg_eus(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cav_msgs
)
_generate_msg_eus(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/ObstacleVec.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cav_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(cav_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cav_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(cav_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(cav_msgs_generate_messages cav_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Control.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_eus _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/VehicleState.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_eus _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Gpybm.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_eus _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/PlanedPath.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_eus _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_eus _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_eus _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Goal.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_eus _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_eus _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/ObstacleVec.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_eus _cav_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cav_msgs_geneus)
add_dependencies(cav_msgs_geneus cav_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cav_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cav_msgs
)
_generate_msg_lisp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cav_msgs
)
_generate_msg_lisp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Gpybm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cav_msgs
)
_generate_msg_lisp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/PlanedPath.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cav_msgs
)
_generate_msg_lisp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cav_msgs
)
_generate_msg_lisp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cav_msgs
)
_generate_msg_lisp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cav_msgs
)
_generate_msg_lisp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cav_msgs
)
_generate_msg_lisp(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/ObstacleVec.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cav_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(cav_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cav_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cav_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cav_msgs_generate_messages cav_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Control.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_lisp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/VehicleState.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_lisp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Gpybm.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_lisp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/PlanedPath.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_lisp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_lisp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_lisp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Goal.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_lisp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_lisp _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/ObstacleVec.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_lisp _cav_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cav_msgs_genlisp)
add_dependencies(cav_msgs_genlisp cav_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cav_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cav_msgs
)
_generate_msg_nodejs(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cav_msgs
)
_generate_msg_nodejs(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Gpybm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cav_msgs
)
_generate_msg_nodejs(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/PlanedPath.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cav_msgs
)
_generate_msg_nodejs(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cav_msgs
)
_generate_msg_nodejs(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cav_msgs
)
_generate_msg_nodejs(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cav_msgs
)
_generate_msg_nodejs(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cav_msgs
)
_generate_msg_nodejs(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/ObstacleVec.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cav_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(cav_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cav_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(cav_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(cav_msgs_generate_messages cav_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Control.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_nodejs _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/VehicleState.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_nodejs _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Gpybm.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_nodejs _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/PlanedPath.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_nodejs _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_nodejs _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_nodejs _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Goal.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_nodejs _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_nodejs _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/ObstacleVec.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_nodejs _cav_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cav_msgs_gennodejs)
add_dependencies(cav_msgs_gennodejs cav_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cav_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cav_msgs
)
_generate_msg_py(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/VehicleState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cav_msgs
)
_generate_msg_py(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Gpybm.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cav_msgs
)
_generate_msg_py(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/PlanedPath.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cav_msgs
)
_generate_msg_py(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cav_msgs
)
_generate_msg_py(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cav_msgs
)
_generate_msg_py(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Goal.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cav_msgs
)
_generate_msg_py(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cav_msgs
)
_generate_msg_py(cav_msgs
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/ObstacleVec.msg"
  "${MSG_I_FLAGS}"
  "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cav_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(cav_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cav_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cav_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cav_msgs_generate_messages cav_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Control.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_py _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/VehicleState.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_py _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Gpybm.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_py _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/PlanedPath.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_py _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_py _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_py _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Goal.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_py _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_py _cav_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/ObstacleVec.msg" NAME_WE)
add_dependencies(cav_msgs_generate_messages_py _cav_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cav_msgs_genpy)
add_dependencies(cav_msgs_genpy cav_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cav_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cav_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cav_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(cav_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cav_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cav_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(cav_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cav_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cav_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(cav_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cav_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cav_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(cav_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cav_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cav_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cav_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(cav_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
