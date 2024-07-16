cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
project(mbot_timesync)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Threads REQUIRED)
find_package(lcm REQUIRED)

include(${LCM_USE_FILE})

add_executable(mbot_timesync
  src/timesync.cpp
  src/timestamp.c
)
target_link_libraries(mbot_timesync
  lcm
  mbot_lcm_msgs-cpp
)
target_include_directories(mbot_timesync PRIVATE
  include
)
