#ifndef SLAM_ACTION_MODEL_HPP
#define SLAM_ACTION_MODEL_HPP

#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <random>

/**
* ActionModel implements the sampling-based odometry action model for estimating the motion of the robot between
* time t and t'.
*
* An action model is used to propagate a sample from the prior distribution, x, into
* the proposal distribution, x', based on the supplied motion estimate of the robot
* in the time interval [t, t'].
*
* To use the ActionModel, a two methods exist:
*
*   - bool updateAction(const pose2D_t& odometry);
*   - particle_t applyAction(const particle_t& sample);
*
* updateAction() provides the most recent odometry data so the action model can update the distributions from
* which it will sample.
*
* applyAction() applies the action to the provided sample and returns a new sample that can be part of the proposal
* distribution for the particle filter.
*/
class ActionModel
{
public:

    /**
    * Constructor for ActionModel.
    */
    ActionModel(void);

    /**
    * updateAction sets up the motion model for the current update for the localization.
    * After initialization, calls to applyAction() will be made, so all distributions based on sensor data
    * should be created here.
    *
    * \param    odometry            Current odometry data from the robot
    * \return   The pose transform distribution representing the uncertainty of the robot's motion.
    */
    bool updateAction(const mbot_lcm_msgs::pose2D_t& odometry);

    /**
    * applyAction applies the motion to the provided sample and returns a new sample that
    * can be part of the proposal distribution for the particle filter.
    *
    * \param    sample          Sample to be moved
    * \return   New sample based on distribution from the motion model at the current update.
    */
    mbot_lcm_msgs::particle_t applyAction(const mbot_lcm_msgs::particle_t& sample);

    void resetPrevious(const mbot_lcm_msgs::pose2D_t& odometry);

private:

    ////////// TODO: Add private member variables needed for you implementation ///////////////////
    const float k1_;
    const float k2_;
    double min_dist_;
    double min_Theta;

    mbot_lcm_msgs::pose2D_t previousPose_;
    double deltax;
    double deltay;
    double deltatheta;
    uint64_t utime_;

    bool init_;

    std::mt19937 num_gen;

    double Rot_1;
    double Rot_2;
    double Theta;

    double rot1Std_;
    double rot2Std_;
    double thetaStd_;

    // float xStd_;
    // float yStd_;
    // float thetaStd_;
};

#endif // SLAM_ACTION_MODEL_HPP