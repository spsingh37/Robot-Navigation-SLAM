#include <planning/motion_planner_server.hpp>
#include <mbot/mbot_channels.h>
#include <slam/slam_channels.h>
#include <unistd.h>
#include <stdio.h>

MotionPlannerServer::MotionPlannerServer(lcm::LCM& lcmComm, const MotionPlanner& mp)
: planner_(mp)
, lcm_(lcmComm)
, latest_map_(OccupancyGrid())
{
    slamPose_.x = slamPose_.y = slamPose_.theta = 0.0;
    // Sub to requests from webapp
    lcm_.subscribe(PATH_REQUEST_CHANNEL, &MotionPlannerServer::handleRequest, this);
    // Sub to map
    lcm_.subscribe(SLAM_MAP_CHANNEL, &MotionPlannerServer::handleMap, this);
    // Sub to current pose
    lcm_.subscribe(SLAM_POSE_CHANNEL, &MotionPlannerServer::handleSlamPose, this);
}

void MotionPlannerServer::handleRequest(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::planner_request_t* request)
{
    // Handle the incoming request from the webapp here
    // Step 1: Define pose2D_t type for start pose
    // Step 2: Define pose2D_t type for goal pose
    // Step 3: Define SearchParams (optional)
    // Step 4: Use latest_map_ to set planner_ map (planner_.setMap(latest_map_))
    // Step 4: Call planner_.planPath(start, goal, searchParams)
    // Step 5: Validate and publish the path
    std::lock_guard<std::mutex> autoLock(lock_);

    mbot_lcm_msgs::pose2D_t start;
    start.x = slamPose_.x;
    start.y = slamPose_.y;
    start.theta = slamPose_.theta;

    mbot_lcm_msgs::pose2D_t goal;
    goal.theta = request->goal.theta;
    auto goalCoord = grid_position_to_global_position(Point<double>(request->goal.y, request->goal.x), latest_map_);
    goal.x = goalCoord.x;
    goal.y = goalCoord.y;

    mbot_lcm_msgs::path2D_t path;
    if(request->require_plan){
        planner_.setMap(latest_map_);
        path = planner_.planPath(start, goal);
    }else{
        path.path_length = 2;
        path.path.push_back(start);
        path.path.push_back(goal);
    }

    lcm_.publish(CONTROLLER_PATH_CHANNEL, &path);

}

void MotionPlannerServer::handleSlamPose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* pose) {
    std::lock_guard<std::mutex> autoLock(lock_);

    slamPose_ = *pose;
}

void MotionPlannerServer::handleMap(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::occupancy_grid_t* map){
    std::lock_guard<std::mutex> autoLock(lock_);
    latest_map_.fromLCM(*map);
}

void MotionPlannerServer::run(void){
    while(true)
    {
        usleep(1000);
    }
}
