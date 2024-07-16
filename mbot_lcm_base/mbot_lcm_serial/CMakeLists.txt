cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
project(mbot_lcm_serial C)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Threads REQUIRED)
find_package(lcm REQUIRED)

include(${LCM_USE_FILE})

# Shim binary.
add_executable(lcm_serial_server src/lcm_serial_server_main.c
  src/comms_common.c
  src/listener.c
  src/protocol.c
  src/topic_data.c
)
target_link_libraries(lcm_serial_server
  ${CMAKE_THREAD_LIBS_INIT}
  lcm
  mbot_lcm_msgs
)
target_include_directories(lcm_serial_server PRIVATE
  include
)

# This is needed to find the shared libraries correctly on RPi OS.
set_target_properties(lcm_serial_server PROPERTIES
    INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib
    BUILD_WITH_INSTALL_RPATH 1
)

# INSTALL needed binaries.
install(
  TARGETS lcm_serial_server
  DESTINATION bin
)
