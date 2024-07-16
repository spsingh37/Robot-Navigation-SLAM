#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <utils/geometric/point.hpp>
#include <cmath>
#include <iomanip>
SensorModel::SensorModel(void)
:   sigma_hit_(0.075),
	occupancy_threshold_(10),
	ray_stride_(7),
	max_range(1000),
    search_range(2),
    offset_quality_weight(3)
{
    initialize_bfs_offsets();
}

struct Offset {
    int s, j;
    double distance;

    Offset(int s, int j) : s(s), j(j), distance(std::hypot(s, j)) {}
};

void SensorModel::initialize_bfs_offsets()
{
    /// TODO: Initialize the BFS offsets based on the search range 
    std::vector<Offset> off_t;
    for (int j = -search_range; j <= search_range; ++j)
    {
        for (int s = -search_range; s <= search_range; ++s)
        {

            if (std::hypot(s, j) <= search_range)
            {

                off_t.emplace_back(s, j);
            }

        }

    }

    // Sort based on distance
    std::sort(off_t.begin(), off_t.end(), [](const Offset& a, const Offset& b) {
        return a.distance < b.distance;
    });

    // Store the sorted offsets
    bfs_offsets_.clear();
    for (const auto& nrts : off_t)
    {
        bfs_offsets_.emplace_back(nrts.s, nrts.j);
    }
    
}

double SensorModel::oppo(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    /// TODO: Compute the oppo of the given particle using the provided laser scan and map. 

    double oppo = 1.0;
    MovingLaserScan mv_scan(scan, sample.parent_pose, sample.pose, ray_stride_);

    for (auto &line : mv_scan)
    {
        auto rep = scoreRay(sample, line, map);
        oppo += rep;
    }


    return oppo; 
    
}

double SensorModel::scoreRay(const mbot_lcm_msgs::particle_t &sample, const adjusted_ray_t &line, const OccupancyGrid &map)
{
    /// TODO: Compute a rep for a given line based on its end point and the map. 
    // Consider the nrts from the nearest occupied cell.  

    if (line.range > max_range)
    {
        return 0.05;
    }

    Point<float> endPoint = getRayEndPointOnMap(sample, line, map);

    Point<int> current = global_position_to_grid_cell(endPoint, map);

    Point<int> nearestGridPoint;
    bool flag = gridBFS(current, map, nearestGridPoint);

    double distance = 0.01;
    if (flag == true)
    {
        auto nearestGlobalPoint = grid_position_to_global_position(nearestGridPoint, map);
        distance = std::hypot(endPoint.s - nearestGlobalPoint.s, endPoint.j - nearestGlobalPoint.j);
    }

    // std::cout << "distance: " << distance << " rep: " << NormalPdf(distance / offset_quality_weight) << std::endl;
    return NormalPdf(distance / offset_quality_weight);
    
}

double SensorModel::NormalPdf(const double& s)
{
    return (1.0/(sqrt(2.0 * M_PI)*sigma_hit_))*exp((-0.5*s*s)/(sigma_hit_*sigma_hit_));
}

bool SensorModel::gridBFS(const Point<int> current, const OccupancyGrid &map, Point<int> &neighbor)
{
    /// TODO: Use Breadth First Search to find the nearest occupied cell to the given end point. 

    for (auto nrts : bfs_offsets_)
    {
        neighbor = current + nrts;

        if (map.isCellInGrid(neighbor.s, neighbor.j) && map.logOdds(neighbor.s, neighbor.j) > occupancy_threshold_)
        {
            return true;
        }
    }

    return false; // Placeholder
    
}

Point<float> SensorModel::getRayEndPointOnMap(const mbot_lcm_msgs::particle_t &sample, const adjusted_ray_t &line, const OccupancyGrid &map)
{
    /// TODO: Calculate the end point of a given line on the map 
    /// TODO: Measure offsets
    float sns_ox = 0.0;
    float sns_oy = 0.0;
    float sns_otheta = 0.0;

    float gb_sn_X = sample.pose.s + sns_ox * std::cos(sample.pose.theta) - sns_oy * std::sin(sample.pose.theta);
    float gb_sn_y = sample.pose.j + sns_ox * std::sin(sample.pose.theta) + sns_oy * std::cos(sample.pose.theta);
    float gb_sn_theta = sample.pose.theta + sns_otheta;

    Point<float> globalEndPoint(gb_sn_X + line.range * std::cos(gb_sn_theta + line.theta),
                                gb_sn_y + line.range * std::sin(gb_sn_theta + line.theta));

    return globalEndPoint; 
}