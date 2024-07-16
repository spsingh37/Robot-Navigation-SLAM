#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.005f)
, k2_(0.025f)
, min_dist_(0.0025)
, min_Theta(0.02)
, init_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rnd_var;

    num_gen = std::mt19937(rnd_var());
    

}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose2D_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose2D_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

    if(init_==false)
    {
        resetPrevious(odometry);

        init_ = true; 
    }
    deltax = odometry.x-previousPose_.x;
    deltay = odometry.y-previousPose_.y;

    deltatheta = angle_diff(odometry.theta, previousPose_.theta);

    Rot_1 = angle_diff(std::atan2(deltay, deltax), previousPose_.theta);
    Theta = std::sqrt(deltax * deltax + deltay * deltay);

    int temp = 1;
    if (std::abs(Rot_1) > M_PI_2)
    {
        Rot_1 = angle_diff(M_PI, Rot_1);
        temp = -1;
    }

    Rot_2 = angle_diff(deltatheta, Rot_1);
    bool should_move = (deltax != 0) || (deltay != 0) || (deltatheta != 0);
    if (!should_move) {
            rot1Std_ = 0;
            thetaStd_ = 0;
            rot2Std_ = 0;
        } else {
            rot1Std_ = abs(k1_ * Rot_1);
            thetaStd_ = abs(k2_ * Theta);
            rot2Std_ = abs(k1_ * Rot_2);

        }

    Theta *= temp;
    utime_ = odometry.utime;
    previousPose_ = odometry;
    
    return should_move;    // Placeholder
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    mbot_lcm_msgs::particle_t next_part = sample;
    
    std::normal_distribution<double> Rot_1sample(0.0, rot1Std_);
    std::normal_distribution<double> trans_sample(0.0, thetaStd_);
    std::normal_distribution<double> Rot_2sample(0.0, rot2Std_);
    next_part.parent_pose = sample.pose;
    next_part.pose.utime = utime_;

    double Rot_1hat = angle_diff(Rot_1, Rot_1sample(num_gen));
    double trans_hat = Theta - trans_sample(num_gen);
    double Rot_2hat = angle_diff(Rot_2, Rot_2sample(num_gen));

    next_part.pose.x += trans_hat * std::cos(angle_sum(sample.pose.theta, Rot_1hat));
    next_part.pose.y += trans_hat * std::sin(angle_sum(sample.pose.theta, Rot_1hat));
    next_part.pose.theta = angle_sum(next_part.pose.theta, angle_sum(Rot_1hat, Rot_2hat));    

    return next_part;
}