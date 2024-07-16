#ifndef PLANNING_MOTION_PLANNER_SERVER_HPP
#define PLANNING_MOTION_PLANNER_SERVER_HPP

#include <lcm/lcm-cpp.hpp>
#include <mbot_lcm_msgs/occupancy_grid_t.hpp>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/planner_request_t.hpp>
#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <planning/motion_planner.hpp>
#include <slam/occupancy_grid.hpp>
#include <mutex>


/**
* MotionPlannerServer is a simple ActionServer(ROS-esque) that will serve as a way from the MBot webapp to interact with the MBot Motion Planner.
* The sole purpose of MotionPlannerServer is to receive a request to plan from the MBot webapp (when the user clicks a goal location), use
* the MBot motion planner to generate a plan, and finally publish the planned path.
*/
class MotionPlannerServer
{
public:

    /**
    * Constructor for MotionPlanner.
    *
    * \param    params          Parameters for controlling the behavior of the planner (optional)
    */
    explicit MotionPlannerServer(lcm::LCM& lcmComm, const MotionPlanner& planner = MotionPlanner());


    void handleRequest(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::planner_request_t* request);
    void handleSlamPose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* pose);
    void handleMap(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::occupancy_grid_t* map);
    void run(void);

private:
    std::mutex lock_;

    MotionPlanner planner_;
    lcm::LCM& lcm_;
    mbot_lcm_msgs::pose2D_t slamPose_;

    OccupancyGrid latest_map_;
};

#endif // PLANNING_MOTION_PLANNER_SERVER_HPP
