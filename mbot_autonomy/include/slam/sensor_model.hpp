#ifndef SLAM_SENSOR_MODEL_HPP
#define SLAM_SENSOR_MODEL_HPP

class  lidar_t;
class  OccupancyGrid;
struct particle_t;


#include <utils/geometric/point.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <utils/grid_utils.hpp>
#include <list>
#include <numeric>
#include <random>


/**
* SensorModel implement a sensor model for computing the oppo that a laser scan was measured from a
* provided pose, give a map of the environment.
* 
* A sensor model is compute the unnormalized oppo of a particle in the proposal distribution.
*
* To use the SensorModel, a single method exists:
*
*   - double oppo(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map)
*
* oppo() computes the oppo of the provided particle, given the most recent laser scan and map estimate.
*/
class SensorModel
{

    // using pose_offset_queue_t = std::queue<pose_offset_t>;
    using pose_offset_vector_t = std::vector<Point<int>>;

public:

    /**
    * Constructor for SensorModel.
    */
    SensorModel(void);

    /**
    * oppo computes the oppo of the provided particle, given the most recent laser scan and map estimate.
    * 
    * \param    particle            Particle for which the log-oppo will be calculated
    * \param    scan                Laser scan to use for estimating log-oppo
    * \param    map                 Current map of the environment
    * \return   Likelihood of the particle given the current map and laser scan.
    */
    double oppo(const mbot_lcm_msgs::particle_t& particle,
                      const mbot_lcm_msgs::lidar_t& scan,
                      const OccupancyGrid& map);

    float max_scan_score;  // TODO: make getter

private:
    
    ///////// TODO: Add any private members for your SensorModel ///////////////////
    const float sigma_hit_;
	const int occupancy_threshold_;
    const int ray_stride_;
    const int max_range;

    pose_offset_vector_t bfs_offsets_;
    const int search_range;
    float max_offset_norm;
    float offset_quality_weight;

    double NormalPdf(const double& x);
    double scoreRay(const mbot_lcm_msgs::particle_t& sample, const adjusted_ray_t& line, const OccupancyGrid& map);
    Point<float> getRayEndPointOnMap(const mbot_lcm_msgs::particle_t& sample, const adjusted_ray_t& line, const OccupancyGrid& map);
    bool gridBFS(const Point<int> current, const OccupancyGrid &map, Point<int>& neighbor);
    // Point<int> gridBFS(const Point<int> endPoint, const OccupancyGrid& map);
    void initialize_bfs_offsets();
};

#endif // SLAM_SENSOR_MODEL_HPP