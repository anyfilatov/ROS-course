# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "RobotsBattle: 1 messages, 0 services")

set(MSG_I_FLAGS "-IRobotsBattle:/home/maksim/ClionProjects/RobotsBattle/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(RobotsBattle_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/maksim/ClionProjects/RobotsBattle/msg/TargetCoordinates.msg" NAME_WE)
add_custom_target(_RobotsBattle_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "RobotsBattle" "/home/maksim/ClionProjects/RobotsBattle/msg/TargetCoordinates.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(RobotsBattle
  "/home/maksim/ClionProjects/RobotsBattle/msg/TargetCoordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RobotsBattle
)

### Generating Services

### Generating Module File
_generate_module_cpp(RobotsBattle
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RobotsBattle
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(RobotsBattle_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(RobotsBattle_generate_messages RobotsBattle_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/maksim/ClionProjects/RobotsBattle/msg/TargetCoordinates.msg" NAME_WE)
add_dependencies(RobotsBattle_generate_messages_cpp _RobotsBattle_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RobotsBattle_gencpp)
add_dependencies(RobotsBattle_gencpp RobotsBattle_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RobotsBattle_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(RobotsBattle
  "/home/maksim/ClionProjects/RobotsBattle/msg/TargetCoordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RobotsBattle
)

### Generating Services

### Generating Module File
_generate_module_eus(RobotsBattle
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RobotsBattle
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(RobotsBattle_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(RobotsBattle_generate_messages RobotsBattle_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/maksim/ClionProjects/RobotsBattle/msg/TargetCoordinates.msg" NAME_WE)
add_dependencies(RobotsBattle_generate_messages_eus _RobotsBattle_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RobotsBattle_geneus)
add_dependencies(RobotsBattle_geneus RobotsBattle_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RobotsBattle_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(RobotsBattle
  "/home/maksim/ClionProjects/RobotsBattle/msg/TargetCoordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RobotsBattle
)

### Generating Services

### Generating Module File
_generate_module_lisp(RobotsBattle
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RobotsBattle
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(RobotsBattle_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(RobotsBattle_generate_messages RobotsBattle_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/maksim/ClionProjects/RobotsBattle/msg/TargetCoordinates.msg" NAME_WE)
add_dependencies(RobotsBattle_generate_messages_lisp _RobotsBattle_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RobotsBattle_genlisp)
add_dependencies(RobotsBattle_genlisp RobotsBattle_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RobotsBattle_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(RobotsBattle
  "/home/maksim/ClionProjects/RobotsBattle/msg/TargetCoordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RobotsBattle
)

### Generating Services

### Generating Module File
_generate_module_nodejs(RobotsBattle
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RobotsBattle
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(RobotsBattle_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(RobotsBattle_generate_messages RobotsBattle_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/maksim/ClionProjects/RobotsBattle/msg/TargetCoordinates.msg" NAME_WE)
add_dependencies(RobotsBattle_generate_messages_nodejs _RobotsBattle_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RobotsBattle_gennodejs)
add_dependencies(RobotsBattle_gennodejs RobotsBattle_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RobotsBattle_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(RobotsBattle
  "/home/maksim/ClionProjects/RobotsBattle/msg/TargetCoordinates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RobotsBattle
)

### Generating Services

### Generating Module File
_generate_module_py(RobotsBattle
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RobotsBattle
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(RobotsBattle_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(RobotsBattle_generate_messages RobotsBattle_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/maksim/ClionProjects/RobotsBattle/msg/TargetCoordinates.msg" NAME_WE)
add_dependencies(RobotsBattle_generate_messages_py _RobotsBattle_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RobotsBattle_genpy)
add_dependencies(RobotsBattle_genpy RobotsBattle_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RobotsBattle_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RobotsBattle)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RobotsBattle
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(RobotsBattle_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RobotsBattle)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RobotsBattle
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(RobotsBattle_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RobotsBattle)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RobotsBattle
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(RobotsBattle_generate_messages_lisp std_msgs_generate_messages_lisp)

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RobotsBattle)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RobotsBattle
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
add_dependencies(RobotsBattle_generate_messages_nodejs std_msgs_generate_messages_nodejs)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RobotsBattle)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RobotsBattle\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RobotsBattle
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(RobotsBattle_generate_messages_py std_msgs_generate_messages_py)
