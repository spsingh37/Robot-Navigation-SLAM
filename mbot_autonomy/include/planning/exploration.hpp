#ifndef PLANNING_EXPLORATION_HPP
#define PLANNING_EXPLORATION_HPP

#include <utils/lcm_config.h>
#include <planning/planning_channels.h>
#include <planning/motion_planner.hpp>
#include <planning/frontiers.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/exploration_status_t.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <mbot_lcm_msgs/mbot_message_received_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <mutex>
#include <set>
#include <iostream>


/**
* Exploration runs a simple state machine to explore -- and possibly escape from -- an environment. The state machine
* for Exploration goes through the following steps:
*
*   - INITIALIZING:
*       start in this state and immediately change states
*           go to EXPLORING_MAP
*   - EXPLORING_MAP;
*       select frontiers and drive to them until all frontiers have been explored
*           go to RETURNING_HOME if the map is explored
*           go to FAILED_EXPLORATION if all frontiers cannot be reached for some reason
*   - RETURNING_HOME:
*       drive back to the home pose -- the first pose received by the exploration module
*           go to COMPLETED_EXPLORATION after reaching the hoome pose if escape mode isn't used
*           go to FAILED_EXPLORATION if a path to the home pose can't be found
*   - COMPLETED_EXPLORATION:
*       stop the robot, exit with SUCCESS
*   - FAILED_EXPLORATION:
*       stop the robot, exit with FAILURE
*
*/
class Exploration
{
public:

    /**
    * Constructor for Exploration
    *
    * The target file contains two lines, each of which is a space-delimited description of a pose. The key pose is
    * first, then the treasure pose
    *
    *   keyPose.x keyPose.y keyPose.theta
    *   treasurePose.x treasurePose.y treasurePose.theta
    *
    * TODO: Add any parameters here that you need to configure your Exploration implementation. The existing parameters
    *   are required.
    *
    * \param    shouldAttemptEscape     Flag indicating if the robot should try to escape the map, or just explore it
    * \param    targetFile              Name of the file holding the key and treasure target information (only matters if shouldAttemptEscape is true)
    * \param    lcmInstance             Instance of LCM to use for communication
    */
    Exploration(lcm::LCM* lcmInstance);

    /**
    * exploreEnvironment explores the robot's environment. The exploration routine assumes that the environment
    * is enclosed.
    *
    * exploreEnvironment doesn't return until the exploration is completed.
    *
    * \return   True if the exploration was successful. False if the exploration failed.
    */
    bool exploreEnvironment(void);

    // Data handlers for LCM messages
    void handleMap(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::occupancy_grid_t* map);
    void handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* pose);
    void handleConfirmation(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::mbot_message_received_t* confirm);

private:
    // Current state and data associated with the update -- use these variables for your exploration computations
    int8_t state_;                      // Current state of the high-level exploration state machine, as defined in exploration_status_t
    bool  shouldAttemptEscape_;         // Flag indicating if the escaping_map state should be entered after returning_home completes
    mbot_lcm_msgs::pose2D_t curr_pose_;  // Robot pose to use for computing new paths
    mbot_lcm_msgs::pose2D_t goal_pose_curr_;
    bool haveCurrentGoal_ = false;
    OccupancyGrid currentMap_;          // Map to use for finding frontiers and planning paths to them
    MotionPlanner planner_;             // Planner to use for finding collision-free paths to select frontiers

    mbot_lcm_msgs::pose2D_t homePose_;  // Pose of the robot when it is home, i.e. the initial pose before exploration begins

    mbot_lcm_msgs::path2D_t currentPath_;  // Current path being followed to a frontier or other target, like the home or key poses
    std::vector<frontier_t> frontiers_; // Current frontiers in the map

    // Data coming in from other modules -- used by LCM thread
    mbot_lcm_msgs::pose2D_t incomingPose_;  // Temporary storage for the most recently received pose until needed by explore thread
    OccupancyGrid incomingMap_;         // Temporary storage for the most recently received map until needed by explore thread

    bool haveNewPose_;                  // Flag indicating if a new pose has been received since the last call to copyDataForUpdate
    bool haveNewMap_;                   // Flag indicating if a new map has been received since the last call to copyDataForUpdate
    bool haveHomePose_;                 // Flag indicating if the home pose has been set

    lcm::LCM* lcmInstance_;             // Instance of LCM to use for sending out information
    std::mutex dataLock_;               // Lock to keep the LCM and explore threads properly synchronized

    /////////// TODO: Add any state variables you might need here //////////////

    mbot_lcm_msgs::pose2D_t   currentTarget_;  // Current target robot is driving to
    OccupancyGrid exploredMap_;     // Map found after completing the RETURNING_HOME state

    size_t prev_frontier_size;
    bool pathReceived_;
    int64_t most_recent_path_time;
//    int8_t path_redundancy_count;

    // Whether or not it has received at least one frontier to explore
    bool initialized_;

    /////////////////////////// End student code ///////////////////////////////


    bool isReadyToUpdate(void);
    void runExploration(void);
    void copyDataForUpdate(void);

    void   executeStateMachine(void);
    int8_t executeInitializing(void);
    int8_t executeExploringMap(bool initialize);
    int8_t executeReturningHome(bool initialize);
    int8_t executeCompleted(bool initialize);
    int8_t executeFailed(bool initialize);

    /////////// TODO: Add any additional methods you might need here //////////////

};

#endif // PLANNING_EXPLORATION_HPP
