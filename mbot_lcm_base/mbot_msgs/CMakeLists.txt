cmake_minimum_required(VERSION 3.3 FATAL_ERROR)
project(mbot_lcm_msgs)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(lcm REQUIRED)
include(${LCM_USE_FILE})

find_package(Python3)
find_package(Java)

if(JAVA_FOUND)
  include(UseJava)
endif()

if(Python3_FOUND)
  set(python_args PYTHON_SOURCES python_sources)
endif()
if(JAVA_FOUND)
  set(java_args JAVA_SOURCES java_sources)
endif()

set(LCM_FILES
      lcmtypes/twist2D_t.lcm
      lcmtypes/timestamp_t.lcm
      lcmtypes/path2D_t.lcm
      lcmtypes/pose2D_t.lcm
      lcmtypes/mbot_slam_reset_t.lcm
      lcmtypes/particles_t.lcm
      lcmtypes/mbot_encoders_t.lcm
      lcmtypes/occupancy_grid_t.lcm
      lcmtypes/point3D_t.lcm
      lcmtypes/joy_t.lcm
      lcmtypes/mbot_imu_t.lcm
      lcmtypes/mbot_motor_pwm_t.lcm
      lcmtypes/pose3D_t.lcm
      lcmtypes/mbot_motor_vel_t.lcm
      lcmtypes/twist3D_t.lcm
      lcmtypes/lidar_t.lcm
      lcmtypes/mbot_message_received_t.lcm
      lcmtypes/particle_t.lcm
      lcmtypes/slam_status_t.lcm
      lcmtypes/exploration_status_t.lcm
      lcmtypes/planner_request_t.lcm
      lcmtypes/mbot_apriltag_t.lcm
      lcmtypes/mbot_apriltag_array_t.lcm
)

lcm_wrap_types(
  C_EXPORT mbot_lcm_msgs
  C_SOURCES c_sources
  C_HEADERS c_headers
  CPP_HEADERS cpp_headers
  ${python_args}
  ${java_args}
  ${LCM_FILES}
)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/mbot_lcm_msgs_serial.h
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/lcm_serial_gen.py ${CMAKE_CURRENT_SOURCE_DIR} ${LCM_FILES}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/lcm_serial_gen.py ${CMAKE_CURRENT_SOURCE_DIR}/lcmtypes/*.lcm
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating ${CMAKE_CURRENT_BINARY_DIR}/mbot_lcm_msgs_serial.c/.h"
)

message(STATUS "Generating serial lcmtypes")

#list(APPEND c_sources ${CMAKE_CURRENT_BINARY_DIR}/mbot_lcm_msgs_serial.c)
list(APPEND c_headers ${CMAKE_CURRENT_BINARY_DIR}/mbot_lcm_msgs_serial.h)

lcm_add_library(mbot_lcm_msgs C ${c_sources} ${c_headers})
#add_dependencies(mbot_lcm_msgs serial_lcmtypes)
include(GenerateExportHeader)
generate_export_header(mbot_lcm_msgs)
target_include_directories(mbot_lcm_msgs INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
target_link_libraries(mbot_lcm_msgs INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

lcm_add_library(mbot_lcm_msgs-cpp CPP ${cpp_headers})
target_include_directories(mbot_lcm_msgs-cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)

if(JAVA_FOUND)
  add_jar(mbot_lcm_msgs-jar
    OUTPUT_NAME mbot_lcm_msgs
    INCLUDE_JARS lcm-java
    SOURCES ${java_sources}
  )
endif()

# INSTALL C and C++ headers.

lcm_install_headers(DESTINATION include
  ${CMAKE_CURRENT_BINARY_DIR}/mbot_lcm_msgs_export.h
  ${c_headers}
  ${cpp_headers}
)

install(TARGETS mbot_lcm_msgs mbot_lcm_msgs-cpp
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib${LIB_SUFFIX}
  ARCHIVE DESTINATION lib${LIB_SUFFIX}
  INCLUDES DESTINATION include
)

# Install Python and Java.

if(Python3_FOUND)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "if True:
      import sysconfig as sc
      print(sc.get_path('stdlib'))"
    OUTPUT_VARIABLE PY_INSTALL_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  message("INSTALLING PYTHON TO ${PY_INSTALL_DIR}")
  lcm_install_python(DESTINATION ${PY_INSTALL_DIR} ${python_sources})
endif()

if(JAVA_FOUND)
  install_jar(mbot_lcm_msgs-jar share/java)
endif()

# Configure this as a CMake package

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
# generate the config file that is includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "lib/cmake"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

export(PACKAGE ${PROJECT_NAME})
