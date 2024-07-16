#include <algorithm>
#include <iostream>
#include <cassert>
#include <signal.h>

#include <lcm/lcm-cpp.hpp>
#include <mbot_lcm_msgs/twist2D_t.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <mbot_lcm_msgs/timestamp_t.hpp>
#include <mbot_lcm_msgs/mbot_message_received_t.hpp>
#include <mbot_lcm_msgs/mbot_slam_reset_t.hpp>

#include <utils/timestamp.h>
#include <utils/geometric/angle_functions.hpp>
#include <utils/geometric/pose_trace.hpp>
#include <utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <slam/slam_channels.h>

#include "omni_maneuver_controller.h"

/////////////////////// TODO: /////////////////////////////
/**
 * Code below is a little more than a template. You will need
 * to update the maneuver controllers to function more effectively
 * and/or add different controllers.
 * You will at least want to:
 *  - Add a form of PID to control the speed at which your
 *      robot reaches its target pose.
 *  - Add a rotation element to the StratingManeuverController
 *      to maintian a avoid deviating from the intended path.
 *  - Limit (min max) the speeds that your robot is commanded
 *      to avoid commands to slow for your bots or ones too high
 */
///////////////////////////////////////////////////////////

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

static float distBetweenPoses(const mbot_lcm_msgs::pose2D_t& pose1, const mbot_lcm_msgs::pose2D_t& pose2)
{
    return sqrt(pow(pose1.x - pose2.x, 2) + pow(pose1.y - pose2.y, 2));
}

class OmniXYManeuverController : public OmniManeuverControllerBase
{
    private:
        float xy_pid[3] = {1.8, 0.0, 0.0};
        float max_vel_ = 0.6;                 // meters / s
        float reset_error_ = 0.01;            // meters
        float max_accel_ = 0.4;               // meters / s^2
        float lookahead_ = 0.15;              // meters
        float target_reached_thresh_ = 0.02;  // meters

        float xy_sum_error_ = 0.0;
        float xy_last_error_ = 0.0;
        float last_vel_ = 0;

        int64_t last_time_ = -1;

        float limitVelocity(float vel, float dt)
        {
            // Cap the velocity.
            vel = std::min(vel, max_vel_);
            vel = std::max(vel, -max_vel_);

            // Cap the acceleration.
            if (vel > last_vel_)
            {
                // Acceleration.
                vel = std::min(vel, last_vel_ + max_accel_ * dt);
            }
            else
            {
                // Deceleration.
                vel = std::max(vel, last_vel_ - max_accel_ * dt);
            }

            return vel;
        }

    public:
        OmniXYManeuverController() = default;
        virtual mbot_lcm_msgs::twist2D_t get_command(const mbot_lcm_msgs::pose2D_t& pose,
                                                                const mbot_lcm_msgs::pose2D_t& target) override
        {
            int64_t utime = utime_now();

            float dx = target.x - pose.x;
            float dy = target.y - pose.y;
            float alpha = atan2(dy,dx) - pose.theta;
            float dxy = sqrt(pow(dx, 2) + pow(dy, 2));

            // If we have at least one measurement, update the derivative and integral.
            float dt = 0.02;
            float xy_der = 0;
            if (last_time_ > 0)
            {
                dt = (utime - last_time_) / 1.e6;
                xy_sum_error_ += dxy * dt;
                xy_der = (dxy - xy_last_error_) / dt;
            }

            // If error is small, reset the integral term.
            if (dxy < reset_error_) xy_sum_error_ = 0;

            // Calculate velocity.
            float vxy = xy_pid[0] * dxy + xy_pid[1] * xy_der + xy_pid[2] * xy_sum_error_;
            vxy = limitVelocity(vxy, dt);

            float vx = vxy*cos(alpha);
            float vy = vxy*sin(alpha);

            // Update values for next loop.
            last_time_ = utime;
            xy_last_error_ = dxy;
            last_vel_ = vxy;

            return {utime, vx, vy, 0.0};
        }

        mbot_lcm_msgs::twist2D_t get_command(const mbot_lcm_msgs::pose2D_t& pose,
                                                        const std::vector<mbot_lcm_msgs::pose2D_t>& path,
                                                        const int idx)
        {
            int64_t utime = utime_now();
            if (path.size() < 1) return {utime, 0, 0, 0};

            // Calculate the lookahead index.
            int lookahead_idx = idx;
            float dist_to_end = 0;
            for (size_t i = idx; i < path.size() - 1; i++)
            {
                dist_to_end += distBetweenPoses(path[idx], path[idx + 1]);
                if (dist_to_end <= lookahead_) lookahead_idx = i + 1;
            }

            // If we are trying to reach the last pose in the path, use default controller.
            if (lookahead_idx == path.size() - 1) return get_command(pose, path.back());

            // Add the distance to the path from the current pose to the total.
            dist_to_end += distBetweenPoses(pose, path[idx]);

            // Calculate error to target at lookahead.
            auto target = path[lookahead_idx];
            float dx_map = target.x - pose.x;
            float dy_map = target.y - pose.y;
            float dist_to_target = sqrt(dx_map * dx_map + dy_map * dy_map);

            // Move the vector to the robot's coordinate frame.
            float vx = (dx_map * cos(pose.theta) + dy_map * sin(pose.theta)) / dist_to_target;
            float vy = (-dx_map * sin(pose.theta) + dy_map * cos(pose.theta)) / dist_to_target;

            // Calculate time step since last loop.
            float dt = 0.02;
            if (last_time_ > 0) dt = (utime - last_time_) / 1.e6;

            // Limit velocity and acceleration.
            float vel = max_vel_;  // limitVelocity(xy_pid[0] * dist_to_end, dt);

            // Update values for next loop.
            last_time_ = utime;
            last_vel_ = vel;

            return {utime, vel * vx, vel * vy, 0.0};
        }

        virtual bool target_reached(const mbot_lcm_msgs::pose2D_t& pose, const mbot_lcm_msgs::pose2D_t& target, bool is_end_pose)  override
        {
            return distBetweenPoses(pose, target) < target_reached_thresh_;
        }

        void reset()
        {
            xy_sum_error_ = 0.0;
            xy_last_error_ = 0.0;
            last_vel_ = 0;
            last_time_ = -1;
        }
};

class MotionController
{
public:

    /**
    * Constructor for MotionController.
    */
    MotionController(lcm::LCM * instance)
    :
        lcmInstance(instance),
        odomToGlobalFrame_{0, 0, 0, 0}
    {
        subscribeToLcm();

	    time_offset = 0;
	    timesync_initialized_ = false;
        path_idx_ = -1;
    }

    /**
    * \brief updateCommand calculates the new motor command to send to the Mbot. This method is called after each call to
    * lcm.handle. You need to check if you have sufficient data to calculate a new command, or if the previous command
    * should just be used again until for feedback becomes available.
    *
    * \return   The motor command to send to the mbot_driver.
    */
    bool updateCommand(mbot_lcm_msgs::twist2D_t& cmd)
    {
        if(targets_.empty() || odomTrace_.empty()) return false;

        cmd.utime = now();
        cmd.vx = cmd.vy = cmd.wz = 0;

        mbot_lcm_msgs::pose2D_t pose = currentPose();

        if (!omni_xy_controller.target_reached(pose, targets_.back(), false))
        {
            if (targets_.size() <= 2)
            {
                // Only one pose target, so just go to it.
                cmd = omni_xy_controller.get_command(pose, targets_.back());
            }
            else
            {
                // Control along path.
                if (updatePathIdx(pose))  // Ensure path index is valid.
                {
                    cmd = omni_xy_controller.get_command(pose, targets_, path_idx_);
                }
            }
		}
        else
        {
            std::cout << "Target reached!" << std::endl;
            targets_.clear();
            path_idx_ = -1;
            omni_xy_controller.reset();
        }

        return true;
    }

    bool timesync_initialized(){ return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::timestamp_t* timesync)
    {
	    timesync_initialized_ = true;
	    time_offset = timesync->utime-utime_now();
    }

    void handlePath(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::path2D_t* path)
    {
        targets_ = path->path;
        // std::reverse(targets_.begin(), targets_.end()); // store first at back to allow for easy pop_back()

    	std::cout << "received new path at time: " << path->utime;
        std::cout << " with length: " << path->path.size() << std::endl;

        path_idx_ = 0;

        //confirm that the path was received
        // mbot_lcm_msgs::message_received_t confirm {now(), path->utime, channel};
        // lcmInstance->publish(MESSAGE_CONFIRMATION_CHANNEL, &confirm);
    }

    void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* odometry)
    {
        mbot_lcm_msgs::pose2D_t pose {odometry->utime, odometry->x, odometry->y, odometry->theta};
        odomTrace_.addPose(pose);
    }

    void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::pose2D_t* pose)
    {
        computeOdometryOffset(*pose);
    }

    void handleSystemReset(const lcm::ReceiveBuffer* buf, const std::string& channel, const mbot_lcm_msgs::mbot_slam_reset_t* request)
    {
        mbot_lcm_msgs::twist2D_t cmd{now(), 0,0,0};
        lcmInstance->publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
        targets_.clear();
        odomToGlobalFrame_.x = 0;
        odomToGlobalFrame_.y = 0;
        odomToGlobalFrame_.theta = 0;
        odomTrace_.clear();
    }


private:

    enum State
    {
        OMNI
    };

    mbot_lcm_msgs::pose2D_t odomToGlobalFrame_;      // transform to convert odometry into the global/map coordinates for navigating in a map
    PoseTrace  odomTrace_;              // trace of odometry for maintaining the offset estimate
    std::vector<mbot_lcm_msgs::pose2D_t> targets_;

    State state_;

    int64_t time_offset;
    bool timesync_initialized_;
    int path_idx_;

    lcm::LCM * lcmInstance;

    OmniXYManeuverController omni_xy_controller;

    int64_t now()
    {
	    return utime_now() + time_offset;
    }

    bool assignNextTarget(void)
    {
        if(!targets_.empty()) { targets_.pop_back(); }
        state_ = OMNI;
        return !targets_.empty();
    }

    bool updatePathIdx(const mbot_lcm_msgs::pose2D_t& pose)
    {
        if (path_idx_ < 0 || path_idx_ >= targets_.size() - 1) return false;

        float min_dist = distBetweenPoses(pose, targets_[path_idx_]);
        for (size_t i = path_idx_ + 1; i < targets_.size(); i++)
        {
            float dist = distBetweenPoses(pose, targets_[i]);
            if (dist > min_dist) break;
            min_dist = dist;
            path_idx_ = i;
        }
        return true;
    }

    void computeOdometryOffset(const mbot_lcm_msgs::pose2D_t& globalPose)
    {
        mbot_lcm_msgs::pose2D_t odomAtTime = odomTrace_.poseAt(globalPose.utime);
        double deltaTheta = globalPose.theta - odomAtTime.theta;
        double xOdomRotated = (odomAtTime.x * std::cos(deltaTheta)) - (odomAtTime.y * std::sin(deltaTheta));
        double yOdomRotated = (odomAtTime.x * std::sin(deltaTheta)) + (odomAtTime.y * std::cos(deltaTheta));

        odomToGlobalFrame_.x = globalPose.x - xOdomRotated;
        odomToGlobalFrame_.y = globalPose.y - yOdomRotated;
        odomToGlobalFrame_.theta = deltaTheta;
    }

    mbot_lcm_msgs::pose2D_t currentPose(void)
    {
        assert(!odomTrace_.empty());

        mbot_lcm_msgs::pose2D_t odomPose = odomTrace_.back();
        mbot_lcm_msgs::pose2D_t pose;
        pose.x = (odomPose.x * std::cos(odomToGlobalFrame_.theta)) - (odomPose.y * std::sin(odomToGlobalFrame_.theta))
            + odomToGlobalFrame_.x;
        pose.y = (odomPose.x * std::sin(odomToGlobalFrame_.theta)) + (odomPose.y * std::cos(odomToGlobalFrame_.theta))
            + odomToGlobalFrame_.y;
        pose.theta = angle_sum(odomPose.theta, odomToGlobalFrame_.theta);

        return pose;
    }

    void subscribeToLcm()
    {
        lcmInstance->subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, this);
        lcmInstance->subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, this);
        lcmInstance->subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, this);
        lcmInstance->subscribe(MBOT_TIMESYNC_CHANNEL, &MotionController::handleTimesync, this);
        lcmInstance->subscribe(MBOT_SYSTEM_RESET_CHANNEL, &MotionController::handleSystemReset, this);

    }
};

int main(int argc, char** argv)
{
    lcm::LCM lcmInstance(MULTICAST_URL);
    MotionController controller(&lcmInstance);

    if(!lcmInstance.good()){
        return 1;
    }

    ctrl_c_pressed = false;
    signal(SIGINT, ctrlc);
    signal(SIGTERM, ctrlc);

    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum
    	if(controller.timesync_initialized()){
            mbot_lcm_msgs::twist2D_t cmd;
            if(controller.updateCommand(cmd)) lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
    	}

        if (ctrl_c_pressed) break;
    }

    // Stop the robot when motion controller quits.
    mbot_lcm_msgs::twist2D_t zero;
    zero.vx = 0;
    zero.vy = 0;
    zero.wz = 0;
    lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &zero);

    return 0;
}
