#include <thread>
#include <memory>
#include <fstream>
#include <csignal>

#include <lcm/lcm-cpp.hpp>

#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/slam_status_t.hpp>
#include <mbot_lcm_msgs/mbot_slam_reset_t.hpp>

#include <utils/getopt.h>
#include <utils/time_util.h>
#include <mbot/mbot_channels.h>
#include <slam/slam_channels.h>
#include <slam/slam.hpp>

#define LOG_HEADER "[SLAM HANDLER] "

using UniqueSlamPtr = std::unique_ptr<OccupancyGridSLAM>;

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

using SlamMode = OccupancyGridSLAM::Mode;
class SystemResetHandler
{
public:
    bool reset_requested = false;

    SystemResetHandler(int numParticles,
                       int hitOdds,
                       int missOdds,
                       lcm::LCM& lcmConnection,
                       bool useOptitrack,
                       SlamMode mode,
                       std::string& mapFile,
                       bool randomInitialPos)
        : numParticles_(numParticles)
        , hitOdds_(hitOdds)
        , missOdds_(missOdds)
        , useOptitrack_(useOptitrack)
        , mode_(mode)
        , mapFile_(mapFile)
        , randomInitialPos_(randomInitialPos)
        , retainPose_(false)
    {
        lcmConnection.subscribe(MBOT_SYSTEM_RESET_CHANNEL, &SystemResetHandler::handle_system_reset, this);
    }

    void handle_system_reset(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                             const mbot_lcm_msgs::mbot_slam_reset_t* reset_msg)
    {
        mode_ = static_cast<SlamMode>(reset_msg->slam_mode);
        retainPose_ = reset_msg->retain_pose;

        if (mode_ == SlamMode::full_slam) randomInitialPos_ = false;
        else if (mode_ == SlamMode::localization_only && !retainPose_) randomInitialPos_ = true;

        // Check if file exists.
        std::ifstream f(reset_msg->slam_map_location.c_str());
        if (f.good()) mapFile_ = reset_msg->slam_map_location;
        else
        {
            std::cout << LOG_HEADER << "WARNING: Provided map does not exist: " << reset_msg->slam_map_location;
            std::cout << ". Using this map instead: " << mapFile_ << std::endl;
        }
        reset_requested = true;

        std::cout << LOG_HEADER << "INFO: Slam reset requested with mode " << mode_ << ". ";
        std::cout << "Saving to file: " << mapFile_ << std::endl;
    }

    UniqueSlamPtr get_reset_slam_ptr(lcm::LCM& lcmConnection, const mbot_lcm_msgs::pose2D_t& pose = {0, 0, 0, 0})
    {
        bool mappingOnly, localizationOnly, actionOnly;
        if (mode_ == SlamMode::idle) return nullptr;
        else if (mode_ == SlamMode::full_slam) mappingOnly = localizationOnly = actionOnly = false;
        else if (mode_ == SlamMode::mapping_only)
        {
            localizationOnly = actionOnly = false;
            mappingOnly = true;
        }
        else if (mode_ == SlamMode::localization_only)
        {
            mappingOnly = actionOnly = false;
            localizationOnly = true;
        }
        else if (mode_ == SlamMode::action_only)
        {
            mappingOnly = false;
            localizationOnly = true;
            actionOnly = true;
        }
        else
        {
            std::cout << LOG_HEADER << "ERROR: Unknown mode: " << mode_ << std::endl;
            return nullptr;
        }

        if (retainPose_)
        {
            std::cout << LOG_HEADER << "Resetting SLAM. Retaining pose." << std::endl;
            return std::make_unique<OccupancyGridSLAM>(
                numParticles_, hitOdds_, missOdds_, lcmConnection, useOptitrack_,
                mappingOnly, localizationOnly, actionOnly, mapFile_, false, pose
            );
        }

        std::cout << LOG_HEADER << "Resetting SLAM." << std::endl;

        return std::make_unique<OccupancyGridSLAM>(
            numParticles_, hitOdds_, missOdds_, lcmConnection, useOptitrack_,
            mappingOnly, localizationOnly, actionOnly, mapFile_, randomInitialPos_
        );
    }

    void reset_complete()
    {
        reset_requested = false;
    }

    bool idle()
    {
        return mode_ == SlamMode::idle;
    }

    std::string getMapFile()
    {
        return mapFile_;
    }

    int getMode()
    {
        int mode;
        switch (mode_) {
            case SlamMode::mapping_only:
                mode = 0;
                break;
            case SlamMode::action_only:
                mode = 1;
                break;
            case SlamMode::localization_only:
                mode = 2;
                break;
            case SlamMode::full_slam:
                mode = 3;
                break;
            case SlamMode::idle:
                mode = 99;
                break;
            default:
                mode = -1;
        }

        return mode;
    }

private:
    int numParticles_;
    int hitOdds_;
    int missOdds_;
    bool useOptitrack_;
    SlamMode mode_;
    std::string mapFile_;
    bool randomInitialPos_;
    bool retainPose_;
};

int main(int argc, char** argv)
{
    const char* kNumParticlesArg = "num-particles";
    const char* kHitOddsArg = "hit-odds";
    const char* kMissOddsArg = "miss-odds";
    const char* kUseOptitrackArg = "use-optitrack";
    const char* kMappingOnlyArg = "mapping-only";
    const char* kActionOnlyArg = "action-only";
    const char* kLocalizationOnlyArg = "localization-only";
    const char* kRandomParticleInitialization = "random-initial-pos";
    const char* kListeningMode = "listen-for-mode";
    const char* kMapFile = "map";

    // Handle Options
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show this help");
    getopt_add_int(gopt, '\0', kNumParticlesArg, "300", "Number of particles to use in the particle filter");
    getopt_add_int(gopt, '\0', kHitOddsArg, "3", "Amount to increase log-odds when a cell is hit by a laser ray");
    getopt_add_int(gopt, '\0', kMissOddsArg, "2", "Amount to decrease log-odds when a cell is passed through by a laser ray");
    getopt_add_bool(gopt, '\0', kUseOptitrackArg, 0, "Flag indicating if the map reference frame should be set to the Optitrack reference frame.");
    getopt_add_bool(gopt, '\0', kMappingOnlyArg, 0, "Flag indicating if mapping-only mode should be run");
    getopt_add_bool(gopt, '\0', kActionOnlyArg, 0, "Flag indicating if action-only mode should be run");
    getopt_add_bool(gopt, '\0', kLocalizationOnlyArg, 0, "Localization only mode should be run.");
    getopt_add_bool(gopt, '\0', kListeningMode, 0, "Given this flag, the system will listen for an lcm mode message.");
    getopt_add_string(gopt, '\0', kMapFile, "current.map", "Map to load if localization only, output map file if mapping mode.");

    getopt_add_bool(gopt, '\0', kRandomParticleInitialization, 0, "Initial particles should be randomly distributed along the map.");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options]", argv[0]);
        getopt_do_usage(gopt);
        return 1;
    }

    int numParticles = getopt_get_int(gopt, kNumParticlesArg);
    int hitOdds = getopt_get_int(gopt, kHitOddsArg);
    int missOdds = getopt_get_int(gopt, kMissOddsArg);
    bool useOptitrack = getopt_get_bool(gopt, kUseOptitrackArg);
    bool mappingOnly = getopt_get_bool(gopt, kMappingOnlyArg);
    bool actionOnly = getopt_get_bool(gopt, kActionOnlyArg);
    bool localizationOnly = getopt_get_bool(gopt, kLocalizationOnlyArg);
    bool randomInitialPos = getopt_get_bool(gopt, kRandomParticleInitialization);
    bool listeningMode = getopt_get_bool(gopt, kListeningMode);
    std::string mapFile = getopt_get_string(gopt, kMapFile);

    // Get the mode from the arguments.
    SlamMode mode = SlamMode::full_slam;
    if (listeningMode) mode = SlamMode::idle;
    else if (actionOnly) mode = SlamMode::action_only;
    else if (mappingOnly) mode = SlamMode::mapping_only;
    else if (localizationOnly) mode = SlamMode::localization_only;

    ctrl_c_pressed = false;
    signal(SIGINT, ctrlc);
    signal(SIGTERM, ctrlc);
    lcm::LCM lcmConnection(MULTICAST_URL);
    if(!lcmConnection.good()){
        return 1;
    }
    SystemResetHandler systemResetHandler(numParticles, hitOdds, missOdds, lcmConnection, useOptitrack,
                                          mode, mapFile, randomInitialPos);

    UniqueSlamPtr slam = systemResetHandler.get_reset_slam_ptr(lcmConnection);
    systemResetHandler.reset_complete();
    std::shared_ptr<std::thread> slamThreadPtr;

    if (!systemResetHandler.idle())
    {
        std::cout << LOG_HEADER << "Starting SLAM..." << std::endl;
        slamThreadPtr = std::unique_ptr<std::thread >(new std::thread([&slam]() { slam->runSLAM(); }));
    }
    else
    {
        std::cout << LOG_HEADER << "SLAM started in IDLE mode, waiting for reset..." << std::endl;
    }

    while(true)
    {
        lcmConnection.handleTimeout(1000);

        if (ctrl_c_pressed) break;

        if (systemResetHandler.reset_requested)
        {
            mbot_lcm_msgs::pose2D_t pose;
            if (slam != nullptr)
            {
                pose = slam->getCurrentPose();
                std::cout << LOG_HEADER << "Stopping old SLAM instance." << std::endl;
                slam->stopSLAM();
                slamThreadPtr->join();
                slam.reset();
                std::cout << LOG_HEADER << "Old SLAM instance stopped." << std::endl;
            }

            slam = systemResetHandler.get_reset_slam_ptr(lcmConnection, pose);
            if (!systemResetHandler.idle())
            {
                slamThreadPtr = std::unique_ptr<std::thread>(new std::thread([&slam]() { slam->runSLAM(); }));
                std::cout << LOG_HEADER << "Starting new SLAM instance." << std::endl;
            }
            else
            {
                std::cout << LOG_HEADER << "SLAM is in idle mode." << std::endl;
            }
            systemResetHandler.reset_complete();
        }

        // Publish the map status.
        mbot_lcm_msgs::slam_status_t status;
        status.utime = utime_now();
        status.slam_mode = systemResetHandler.getMode();
        status.map_path = systemResetHandler.getMapFile();
        lcmConnection.publish(SLAM_STATUS_CHANNEL, &status);
    }

    if (!systemResetHandler.idle())
    {
        std::cout << LOG_HEADER << "Cleaning up..." << std::endl;

        // Publish status once more.
        mbot_lcm_msgs::slam_status_t status;
        status.utime = utime_now();
        status.slam_mode = SlamMode::INVALID;
        lcmConnection.publish(SLAM_STATUS_CHANNEL, &status);

        // Stop SLAM.
        slam->stopSLAM();
        slamThreadPtr->join();
        slam.reset();
    }

    std::cout << LOG_HEADER << "Slam stopped." << std::endl;

    return 0;
}
