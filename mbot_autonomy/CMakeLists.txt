cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(mbot_autonomy)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(Threads REQUIRED)
find_package(lcm REQUIRED)
find_package(GTK2 2.6 REQUIRED gtk)
find_package(mbot_lcm_msgs)

include(${LCM_USE_FILE})

# Common utils.
add_library(common_utils STATIC
  src/utils/geometric/pose_trace.cpp
  src/utils/getopt.c
  src/utils/timestamp.c
  src/utils/zarray.c
  src/utils/zhash.c
)
target_link_libraries(common_utils
  ${GTK2_LIBRARIES}
)
target_include_directories(common_utils PUBLIC
  ${GTK2_INCLUDE_DIRS}
  include
)

# MOTION CONTROLLER
add_executable(mbot_motion_controller src/mbot/diff_motion_controller.cpp)
# add_executable(mbot_motion_controller src/mbot/omni_motion_controller.cpp)

target_link_libraries(mbot_motion_controller
  common_utils
  lcm
)
target_include_directories(mbot_motion_controller PRIVATE
  include
)

# SLAM
add_executable(mbot_slam src/slam/slam_main.cpp
  src/slam/action_model.cpp
  src/slam/mapping.cpp
  src/slam/moving_laser_scan.cpp
  src/slam/occupancy_grid.cpp
  src/slam/particle_filter.cpp
  src/slam/sensor_model.cpp
  src/slam/slam.cpp
)
target_link_libraries(mbot_slam
  ${CMAKE_THREAD_LIBS_INIT}
  common_utils
  lcm
)
target_include_directories(mbot_slam PRIVATE
  include
)

# EXPLORATION
add_executable(exploration src/planning/exploration_main.cpp
                           src/planning/exploration.cpp
                           src/planning/motion_planner.cpp
                           src/planning/frontiers.cpp
                           src/slam/occupancy_grid.cpp
                           src/planning/obstacle_distance_grid.cpp
                           src/planning/astar.cpp
)
target_link_libraries(exploration
  mbot_lcm_msgs-cpp
  common_utils
  lcm
  ${CMAKE_THREAD_LIBS_INIT}
)
target_include_directories(exploration PRIVATE
  include
  /usr/local/include
)

# PLANNING SERVER
add_executable(motion_planning_server src/planning/motion_planner_server_main.cpp
                                      src/planning/motion_planner.cpp
                                      src/planning/motion_planner_server.cpp
                                      src/slam/occupancy_grid.cpp
                                      src/planning/obstacle_distance_grid.cpp
                                      src/planning/astar.cpp
)
target_link_libraries(motion_planning_server
  mbot_lcm_msgs-cpp
  common_utils
  lcm
  ${CMAKE_THREAD_LIBS_INIT}
)
target_include_directories(motion_planning_server PRIVATE
  include
)

# TODO: Remove from this project. Moved to LCM base repo.
# TIMESYNC
# add_executable(timesync src/mbot/timesync.cpp
# )
# target_link_libraries(timesync
#   common_utils
#   lcm
# )
# target_include_directories(timesync PRIVATE
#   include
# )

# This is needed to find the shared libraries correctly on RPi OS.
set_target_properties(mbot_slam mbot_motion_controller PROPERTIES
    INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib
    BUILD_WITH_INSTALL_RPATH 1
)

# INSTALL needed binaries.
install(
  TARGETS mbot_slam mbot_motion_controller
  DESTINATION bin
)
