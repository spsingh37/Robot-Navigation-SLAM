#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono>
using namespace std::chrono;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose2D_t& pose,
                        OccupancyGrid& map)
{
    if (!initialized_)
        previousPose_ = pose;
    initialized_ = true;

    MovingLaserScan movingScan(scan, previousPose_, pose);

    /// TODO: Update the map's log odds using the movingScan  
    //
    // Hint: Consider both the cells the laser hit and the cells it passed through.
    for(auto& ray : movingScan)
    {
        scoreEndpoint(ray, map);
    }

    for(auto& ray : movingScan)
    {
        scoreRay(ray, map);
    }

    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cell that the laser endpoint hits  
    if(ray.range < kMaxLaserDistance_)
    {
        Point<float> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayCell;
        rayCell.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
        rayCell.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);

        if(map.isCellInGrid(rayCell.x, rayCell.y))
        {
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cells that the laser ray passes through  
    std::vector<Point<int>> cells = bresenham(ray, map);
    for(auto& cell : cells)
    {
        if(map.isCellInGrid(cell.x, cell.y))
        {
            decreaseCellOdds(cell.x, cell.y, map);
        }
    }
    
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement the Bresenham's line algorithm to find cells touched by the ray.
    Point<int> rayStart = global_position_to_grid_cell(ray.origin, map);
    Point<int> rayEnd;
    rayEnd.x = static_cast<int>(ray.range * std::cos(ray.theta) * map.cellsPerMeter() + rayStart.x);
    rayEnd.y = static_cast<int>(ray.range * std::sin(ray.theta) * map.cellsPerMeter() + rayStart.y);

    int dx = std::abs(rayEnd.x - rayStart.x);
    int dy = std::abs(rayEnd.y - rayStart.y);
    int sx = (rayStart.x < rayEnd.x) ? 1 : -1;
    int sy = (rayStart.y < rayEnd.y) ? 1 : -1;
    int err = dx - dy;
    int x = rayStart.x;
    int y = rayStart.y;
    std::vector<Point<int>> cells;
    while(x != rayEnd.x || y != rayEnd.y)
    {
        cells.push_back(Point<int>(x, y));
        // if(x == rayEnd.x && y == rayEnd.y)
        // {
        //     break;
        // }
        int e2 = 2 * err;
        if(e2 > -dy)
        {
            err -= dy;
            x += sx;
        }
        if(e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }
    return cells;    
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement an alternative approach to find cells touched by the ray. 
    return {};
    
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map) {
    /// TODO: Increase the odds of the cell at (x,y)
    if(!initialized_)
    {
        //Do nothing
    }
    else if(127 - map(x, y) > kHitOdds_)
    {
        map(x, y) += kHitOdds_;
    }
    else
    {
        map(x, y) = 127;
    
    }
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map) {
    /// TODO: Decrease the odds of the cell at (x,y)
    if(!initialized_)
    {
        //Do nothing
    }
    else if(map(x, y) + 128 > kMissOdds_)
    {
        map(x, y) -= kMissOdds_;
    }
    else
    {
        map(x, y) = -128;
    }
}