#ifndef SLAM_MAPPING_HPP
#define SLAM_MAPPING_HPP

#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/lidar_t.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <cstdint>
#include <vector>
#include <tuple>

/**
* Mapping implements the occupancy grid mapping algorithm.
*/
class Mapping
{
public:

    /**
    * Constructor for Mapping.
    *
    * \param    maxLaserDistance    Maximum distance for the rays to be traced
    * \param    hitOdds             Increase in occupied odds for cells hit by a laser ray
    * \param    missOdds            Decrease in occupied odds for cells passed through by a laser ray
    */
    Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds);

    /**                                                                                     *
    * updateMap incorporates information from a new laser scan into an existing OccupancyGrid.
    *
    * \param    scan            Laser scan to use for updating the occupancy grid
    * \param    pose            Pose of the robot at the time when the last ray was measured
    * \param    map             OccupancyGrid instance to be updated
    */
    void updateMap(const mbot_lcm_msgs::lidar_t& scan, const mbot_lcm_msgs::pose2D_t& pose, OccupancyGrid& map);

private:

    const float  kMaxLaserDistance_;
    const int8_t kHitOdds_;
    const int8_t kMissOdds_;

    //////////////////// TODO: Add any private members needed for your occupancy grid mapping algorithm ///////////////
    bool initialized_;
    mbot_lcm_msgs::pose2D_t previousPose_;

    void scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map);
    void scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map);
    std::vector<Point<int>> bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map);
    std::vector<Point<int>> divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map);
    void increaseCellOdds(int x, int y, OccupancyGrid &map);
    void decreaseCellOdds(int x, int y, OccupancyGrid &map);
};

#endif // SLAM_MAPPING_HPP