#ifndef SLAM_OCCUPANCY_GRID_SLAM_HPP
#define SLAM_OCCUPANCY_GRID_SLAM_HPP

#include <deque>
#include <mutex>

#include <lcm/lcm-cpp.hpp>
#include <mbot_lcm_msgs/lidar_t.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>

#include <utils/geometric/pose_trace.hpp>
#include <utils/lcm_config.h>
#include <slam/mapping.hpp>
#include <slam/occupancy_grid.hpp>
#include <slam/particle_filter.hpp>

/**
* OccupancyGridSLAM runs on a thread and handles mapping.
*
* LCM messages are assumed to be arriving asynchronously from the runSLAM thread. Synchronization
* between the two threads is handled internally.
*/
class OccupancyGridSLAM
{
public:

    /**
    * Constructor for OccupancyGridSLAM.
    *
    * \param    numParticles      Number of particles to use in the filter
    * \param    hitOddsIncrease   Amount to increase odds when laser hits a cell
    * \param    missOddsDecrease  Amount to decrease odds when laser passes through a cell
    * \param    lcmComm           LCM instance for establishing subscriptions
    * \param    waitForOptitrack  Don't start performing SLAM until a message establishing the reference frame arrives from the Optitrack
    * \param    mappingOnlyMode   Flag indicating if poses are going to be arriving from elsewhere, so just update the mapping (optional, default = false, don't run mapping-only mode)
    * \param    actionOnlyMode    Flag indicating if we will run the sensor model when updating the particle filter
    * \param    mapFile           Name of the map to load for localization-only mode, or to save to for mapping mode (optional, default = "")
    * \param    randomInitialPos  Flag indicating whether the initial particles position will be set randomly
    * \pre mappingOnly or localizationOnly are mutually exclusive. They can both be false for full SLAM mode.
    */
    OccupancyGridSLAM(int numParticles,
                      int8_t hitOddsIncrease,
                      int8_t missOddsDecrease,
                      lcm::LCM& lcmComm,
                      bool waitForOptitrack,
                      bool mappingOnlyMode = false,
                      bool localizationOnlyMode = false,
                      bool actionOnlyMode = false,
                      const std::string mapFile = std::string("current.map"),
                      bool randomInitialPos = false,
                      mbot_lcm_msgs::pose2D_t initialPose = {0, 0, 0, 0});

    ~OccupancyGridSLAM();

    /**
    * runSLAM enters an infinite loop where SLAM will keep running as long as data is arriving.
    * It will sit and block forever if no data is incoming.
    *
    * This method should be launched on its own thread.
    */
    void runSLAM(void);
    void stopSLAM(void);


    // Handlers for LCM messages
    void handleLaser(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::lidar_t* scan);
    void handleOdometry(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* odometry);
    void handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* pose);
    void handleOptitrack(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* pose);

    mbot_lcm_msgs::pose2D_t getCurrentPose() const { return currentPose_; };

    enum Mode
    {
        INVALID=-1,
        mapping_only=0,
        action_only=1,
        localization_only=2,
        full_slam=3,
        idle=99,
    };

private:

    // State variables for controlling progress of the algorithm
    Mode mode_;     // which mode is currently being used?
    bool haveInitializedPoses_;
    bool waitingForOptitrack_;
    bool haveMap_;
    bool running_;
    int  numIgnoredScans_;
    int iters_;
    std::string mapFile_;

    // Data from LCM
    std::deque<mbot_lcm_msgs::lidar_t> incomingScans_;
    PoseTrace groundTruthPoses_;
    PoseTrace odometryPoses_;

    // Data being used for current SLAM iteration
    mbot_lcm_msgs::lidar_t currentScan_;
    mbot_lcm_msgs::pose2D_t currentOdometry_;

    mbot_lcm_msgs::pose2D_t initialPose_;
    mbot_lcm_msgs::pose2D_t previousPose_;
    mbot_lcm_msgs::pose2D_t currentPose_;
    mbot_lcm_msgs::occupancy_grid_t map_obj;

    ParticleFilter filter_;
    OccupancyGrid map_;
    Mapping mapper_;

    lcm::LCM& lcm_;
    std::vector<lcm::Subscription*> lcm_subscriptions_;
    int mapUpdateCount_;  // count so we only send the map occasionally, as it takes lots of bandwidth

    std::mutex dataMutex_;
    std::mutex stopMutex_;

    bool isReadyToUpdate(void);
    void runSLAMIteration(void);
    void copyDataForSLAMUpdate(void);
    void initializePosesIfNeeded(void);
    void updateLocalization(void);
    void updateMap(void);
    bool updateOdometry(const mbot_lcm_msgs::pose2D_t& odomPose, const mbot_lcm_msgs::pose2D_t& slamPose);


    // TODO: your own variables
    bool randomInitialPos_;
    float odomResetThreshDist_, odomResetThreshAng_;
};

#endif // SLAM_OCCUPANCY_GRID_SLAM_HPP
