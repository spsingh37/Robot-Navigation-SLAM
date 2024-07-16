#ifndef OMNI_MANEUVERS_CONTROLLER_BASE_
#define OMNI_MANEUVERS_CONTROLLER_BASE_

#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/twist2D_t.hpp>

class OmniManeuverControllerBase
{
public:
    OmniManeuverControllerBase() = default;

    virtual mbot_lcm_msgs::twist2D_t get_command(const mbot_lcm_msgs::pose2D_t& pose,
                                                             const mbot_lcm_msgs::pose2D_t& target) = 0;
    virtual bool target_reached(const mbot_lcm_msgs::pose2D_t& pose,
                                const mbot_lcm_msgs::pose2D_t& target,
                                bool is_end_pose = false) = 0;
};

#endif
