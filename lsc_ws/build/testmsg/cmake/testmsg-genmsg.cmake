# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "testmsg: 1 messages, 0 services")

set(MSG_I_FLAGS "-Itestmsg:/home/tong/lsc_ws/src/testmsg/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(testmsg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/tong/lsc_ws/src/testmsg/msg/Testmag.msg" NAME_WE)
add_custom_target(_testmsg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "testmsg" "/home/tong/lsc_ws/src/testmsg/msg/Testmag.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(testmsg
  "/home/tong/lsc_ws/src/testmsg/msg/Testmag.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/testmsg
)

### Generating Services

### Generating Module File
_generate_module_cpp(testmsg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/testmsg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(testmsg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(testmsg_generate_messages testmsg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tong/lsc_ws/src/testmsg/msg/Testmag.msg" NAME_WE)
add_dependencies(testmsg_generate_messages_cpp _testmsg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testmsg_gencpp)
add_dependencies(testmsg_gencpp testmsg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testmsg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(testmsg
  "/home/tong/lsc_ws/src/testmsg/msg/Testmag.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/testmsg
)

### Generating Services

### Generating Module File
_generate_module_eus(testmsg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/testmsg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(testmsg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(testmsg_generate_messages testmsg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tong/lsc_ws/src/testmsg/msg/Testmag.msg" NAME_WE)
add_dependencies(testmsg_generate_messages_eus _testmsg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testmsg_geneus)
add_dependencies(testmsg_geneus testmsg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testmsg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(testmsg
  "/home/tong/lsc_ws/src/testmsg/msg/Testmag.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/testmsg
)

### Generating Services

### Generating Module File
_generate_module_lisp(testmsg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/testmsg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(testmsg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(testmsg_generate_messages testmsg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tong/lsc_ws/src/testmsg/msg/Testmag.msg" NAME_WE)
add_dependencies(testmsg_generate_messages_lisp _testmsg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testmsg_genlisp)
add_dependencies(testmsg_genlisp testmsg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testmsg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(testmsg
  "/home/tong/lsc_ws/src/testmsg/msg/Testmag.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/testmsg
)

### Generating Services

### Generating Module File
_generate_module_nodejs(testmsg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/testmsg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(testmsg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(testmsg_generate_messages testmsg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tong/lsc_ws/src/testmsg/msg/Testmag.msg" NAME_WE)
add_dependencies(testmsg_generate_messages_nodejs _testmsg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testmsg_gennodejs)
add_dependencies(testmsg_gennodejs testmsg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testmsg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(testmsg
  "/home/tong/lsc_ws/src/testmsg/msg/Testmag.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testmsg
)

### Generating Services

### Generating Module File
_generate_module_py(testmsg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testmsg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(testmsg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(testmsg_generate_messages testmsg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tong/lsc_ws/src/testmsg/msg/Testmag.msg" NAME_WE)
add_dependencies(testmsg_generate_messages_py _testmsg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testmsg_genpy)
add_dependencies(testmsg_genpy testmsg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testmsg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/testmsg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/testmsg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/testmsg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/testmsg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/testmsg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/testmsg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/testmsg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/testmsg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testmsg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testmsg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testmsg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
