#include <planning/frontiers.hpp>
#include <planning/motion_planner.hpp>
#include <utils/grid_utils.hpp>
#include <utils/timestamp.h>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/path2D_t.hpp>
#include <queue>
#include <set>
#include <cassert>


bool is_frontier_cell(int x, int y, const OccupancyGrid& map);
frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers);
mbot_lcm_msgs::path2D_t path_to_frontier(const frontier_t& frontier,
                                              const mbot_lcm_msgs::pose2D_t& pose,
                                              const OccupancyGrid& map,
                                              const MotionPlanner& planner);
mbot_lcm_msgs::pose2D_t nearest_navigable_cell(mbot_lcm_msgs::pose2D_t pose,
                                                  Point<float> desiredPosition,
                                                  const OccupancyGrid& map,
                                                  const MotionPlanner& planner);

mbot_lcm_msgs::pose2D_t search_to_nearest_free_space(Point<double> position, const OccupancyGrid& map, const MotionPlanner& planner) {
    mbot_lcm_msgs::pose2D_t nearest_free_space = {0};
    if (planner.isValidGoal(position)) {
        nearest_free_space.x = position.x;
        nearest_free_space.y = position.y;
        if (planner.isValidGoal(nearest_free_space)) {
            return nearest_free_space;
        }
    }

    Point<int> start_cell = global_position_to_grid_cell(position, map);
    std::queue<Point<int>> cell_queue;
    std::set<Point<int>> visited_cells;
    cell_queue.push(start_cell);
    visited_cells.insert(start_cell);

    const int num_neighbors = 8;
    const int x_offsets[] = {-1, -1, -1, 1, 1, 1, 0, 0};
    const int y_offsets[] = {0, 1, -1, 0, 1, -1, 1, -1};

    while (!cell_queue.empty()) {
        Point<int> current_cell = cell_queue.front();
        cell_queue.pop();

        for (int i = 0; i < num_neighbors; ++i) {
            Point<int> neighbor(current_cell.x + x_offsets[i], current_cell.y + y_offsets[i]);
            if (visited_cells.find(neighbor) == visited_cells.end()) {
                visited_cells.insert(neighbor);
                cell_queue.push(neighbor);
                if (planner.isValidGoal(neighbor)) {
                    Point<double> global_position = grid_position_to_global_position(neighbor, map);
                    nearest_free_space.x = global_position.x;
                    nearest_free_space.y = global_position.y;
                    if (planner.isValidGoal(nearest_free_space)) {
                        printf("BFS found reachable space with queue size: %d\n", cell_queue.size());
                        return nearest_free_space;
                    }
                }
            }
        }
    }

    printf("Failed to find a reachable space near the centroid!\n");
    return nearest_free_space;
}



double path_length(const mbot_lcm_msgs::path2D_t& path) {
    double total_length = 0.0;
    for (int i = 0; i < path.path_length - 1; ++i) {
        double dx = path.path[i].x - path.path[i + 1].x;
        double dy = path.path[i].y - path.path[i + 1].y;
        total_length += std::sqrt(dx * dx + dy * dy);
    }
    return total_length;
}

std::vector<frontier_t> find_map_frontiers(const OccupancyGrid& map,
                                           const mbot_lcm_msgs::pose2D_t& robotPose,
                                           double minFrontierLength)
{
    /*
    * To find frontiers, we use a connected components search in the occupancy grid. 
    * encountered, then we grow that frontier until all connected cells are found. We then continue scanning through the
    * grid. This algorithm can also perform very fast blob detection if you change is_frontier_cell to some oEach connected components consists
    * only of cells where is_frontier_cell returns true. We scan the grid until an unvisited frontier cell isther check
    * based on pixel color or another condition amongst pixels.
    */
    std::vector<frontier_t> frontiers;
    std::set<Point<int>> visitedCells;

    Point<int> robotCell = global_position_to_grid_cell(Point<float>(robotPose.x, robotPose.y), map);
    std::queue<Point<int>> cellQueue;
    cellQueue.push(robotCell);
    visitedCells.insert(robotCell);

    // Use a 4-way connected check for expanding through free space.
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };

    // Do a simple BFS to find all connected free space cells and thus avoid unreachable frontiers
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();

        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);

            // If the cell has been visited or isn't in the map, then skip it
            if(visitedCells.find(neighbor) != visitedCells.end() || !map.isCellInGrid(neighbor.x, neighbor.y))
            {
                continue;
            }
            // If it is a frontier cell, then grow that frontier
            else if(is_frontier_cell(neighbor.x, neighbor.y, map))
            {
                frontier_t f = grow_frontier(neighbor, map, visitedCells);

                // If the frontier is large enough, then add it to the collection of map frontiers
                if(f.cells.size() * map.metersPerCell() >= minFrontierLength)
                {
                    frontiers.push_back(f);
                }
            }
            // If it is a free space cell, then keep growing the frontiers
            else if(map(neighbor.x, neighbor.y) < 0)
            {
                visitedCells.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }
    return frontiers;
}

struct CompareCentroids
{
    CompareCentroids(mbot_lcm_msgs::pose2D_t robotPose) { this->robotPose = robotPose;}
    inline bool operator() (const Point<double>& centr_1, const Point<double>& centr_2)
    {
        // Diff 1
        float diff_1_x = robotPose.x - centr_1.x;
        float diff_1_y = robotPose.y - centr_1.y;
        float diff_1 = diff_1_x * diff_1_x + diff_1_y * diff_1_y;
        // Diff 2
        float diff_2_x = robotPose.x - centr_2.x;
        float diff_2_y = robotPose.y - centr_2.y;
        float diff_2 = diff_2_x * diff_2_x + diff_2_y * diff_2_y;

        return (diff_1 < diff_2);
    }
    mbot_lcm_msgs::pose2D_t robotPose;
};

frontier_processing_t plan_path_to_frontier(const std::vector<frontier_t>& frontiers,
                                            const mbot_lcm_msgs::pose2D_t& robotPose,
                                            const OccupancyGrid& map,
                                            const MotionPlanner& planner) {
    mbot_lcm_msgs::path2D_t final_path;
    final_path.utime = utime_now();
    final_path.path_length = 1;
    final_path.path.push_back(robotPose);

    int unreachable_frontiers = 0;

    // Get the centroids of all frontiers
    std::vector<Point<double>> centroids;
    for (const auto& frontier : frontiers) {
        centroids.push_back(find_frontier_centroid(frontier));
    }

    // Sort the centroids by distance to the robot
    std::sort(centroids.begin(), centroids.end(), CompareCentroids(robotPose));
    
    bool valid_path_found = false;
    for (const auto& centroid : centroids) {
        mbot_lcm_msgs::pose2D_t reachable_space = search_to_nearest_free_space(centroid, map, planner);
        mbot_lcm_msgs::path2D_t path = planner.planPath(robotPose, reachable_space);
        double distance = path_length(path);

        if (distance < std::numeric_limits<double>::max() && path.path_length > 1) {
            final_path = path;
            valid_path_found = true;
            break;
        } else {
            unreachable_frontiers++;
        }
    }

    return frontier_processing_t(final_path, unreachable_frontiers);
}



// frontier_processing_t plan_path_to_frontier(const std::vector<frontier_t>& frontiers,
//                                             const mbot_lcm_msgs::pose2D_t& robotPose,
//                                             const OccupancyGrid& map,
//                                             const MotionPlanner& planner)
// {
//     ///////////// TODO: Implement your strategy to select the next frontier to explore here: This is Done above //////////////////
//     /*
//     * NOTES:
//     *   - If there's multiple frontiers, you'll need to decide which to drive to.
//     *   - A frontier is a collection of cells, you'll need to decide which one to attempt to drive to.
//     *   - The cells along the frontier might not be in the configuration space of the robot, so you won't necessarily
//     *       be able to drive straight to a frontier cell, but will need to drive somewhere close.
//     */

//     // First, choose the frontier to go to
//     // Initial alg: find the nearest one

//     // Returnable path
//     mbot_lcm_msgs::path2D_t path;
//     path.utime = utime_now();
//     path.path_length = 1;
//     path.path.push_back(robotPose);

//     int unreachable_frontiers = 0;

//     // Get the centroids of every frontier, and sort them in ascending order by distance to the robot
//     frontier_t chosen_frontier;
//     std::vector<Point<double>> centroids;
//     for (auto &&frontier : frontiers)
//     {
//         // Find the centroid of the frontier
//         auto centroid = find_frontier_centroid(frontier);
//         centroids.push_back(centroid);
//     }
//     // Order the centroids by their distance to the robot
//     std::sort(centroids.begin(), centroids.end(), CompareCentroids(robotPose));
//     Point<double> goal;
//     bool found_valid_cell = false;

//     int max_counter_dist = 40;
//     // Number of directions to search from the centroid cell
//     int num_directions = 36;
//     // std::vector<std::pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,-1},{1,-1},{-1,1}};
//     std::vector<std::pair<float,float>> directions;
//     for (size_t i = 0; i < num_directions; i++)
//     {
//         float curr_angle = 2.0 * M_PI / num_directions * i;
//         directions.push_back(std::pair<float,float>(std::cos(curr_angle), std::sin(curr_angle)));
//     }

//     for (auto &&c : centroids)
//     {
//         // Find the closest cell that is a valid goal
//         // Do a breadth first search 4 way
//         Point<int> centr_cell = global_position_to_grid_cell(c, map);
//         // printf("centroid (%d,%d)\n", centr_cell.x, centr_cell.y);

//         // This will mark how many cells from the initial one are we. We can stop searching if we get to a certain value
//         int counter_dist = 1;
//         // Step to accelerate the finding
//         int distance_step = 1;

//         Point<int> goal_cell;
//         auto centr_directions = directions;

//         while (!found_valid_cell && counter_dist <= max_counter_dist)
//         {
//             for (auto &&dir : centr_directions)
//             {
//                 // printf("dir: (%d,%d)\n", dir.first, dir.second);
//                 // Way to block certain directions
//                 if (dir.first == 0.0 && dir.second == 0.0)
//                     continue;

//                 cell_t new_cell(
//                     centr_cell.x + dir.first * counter_dist,
//                     centr_cell.y + dir.second * counter_dist
//                 );
//                 // printf("new cell: (%d,%d)\n", new_cell.x, new_cell.y);
//                 // Check if cell is a wall. Block that direction
//                 if (map.isCellInGrid(new_cell.x, new_cell.y) && map.logOdds(new_cell.x, new_cell.y) > 0)
//                 {
//                     dir = std::pair<float,float>(0.0,0.0);
//                     continue;
//                 }

//                 // Check if it is a valid goal
//                 if (planner.isValidGoal(new_cell))
//                 {
//                     goal_cell = new_cell;
//                     found_valid_cell = true;
//                     goal = grid_position_to_global_position(goal_cell, map);
//                     mbot_lcm_msgs::pose2D_t goal_pose;
//                     goal_pose.x = goal.x;
//                     goal_pose.y = goal.y;
//                     goal_pose.theta = 0.0;
//                     goal_pose.utime = utime_now();

//                     // printf("goal (%f,%f)\n", goal.x, goal.y);
//                     // Plan the path to that chosen centroid
//                     path = planner.planPath(robotPose, goal_pose);
//                     // Only end the processing if it found a valid path
//                     if (path.path_length > 1)
//                         break;
//                     else
//                         found_valid_cell = false;
//                 }
//             }

//             counter_dist++;
//         }
//         if (found_valid_cell)
//         {
//             break;
//         }
//         if (!found_valid_cell)
//         {
//             // printf("was unreachable\n");
//             unreachable_frontiers++;
//         }
//     }

//     return frontier_processing_t(path, unreachable_frontiers);
// }


bool is_frontier_cell(int x, int y, const OccupancyGrid& map)
{
    // A cell is a frontier if it has log-odds 0 and a neighbor has log-odds < 0

    // A cell must be in the grid and must have log-odds 0 to even be considered as a frontier
    if(!map.isCellInGrid(x, y) || (map(x, y) != 0))
    {
        return false;
    }

    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };

    for(int n = 0; n < kNumNeighbors; ++n)
    {
        // If any of the neighbors are free, then it's a frontier
        // Note that logOdds returns 0 for out-of-map cells, so no explicit check is needed.
        if(map.logOdds(x + xDeltas[n], y + yDeltas[n]) < 0)
        {
            return true;
        }
    }

    return false;
}


frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers)
{
    // Every cell in cellQueue is assumed to be in visitedFrontiers as well
    std::queue<Point<int>> cellQueue;
    cellQueue.push(cell);
    visitedFrontiers.insert(cell);

    // Use an 8-way connected search for growing a frontier
    const int kNumNeighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = {  0,  1, -1, 0, 1,-1, 1,-1 };

    frontier_t frontier;

    // Do a simple BFS to find all connected frontier cells to the starting cell
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();

        // The frontier stores the global coordinate of the cells, so convert it first
        frontier.cells.push_back(grid_position_to_global_position(nextCell, map));

        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            if((visitedFrontiers.find(neighbor) == visitedFrontiers.end())
                && (is_frontier_cell(neighbor.x, neighbor.y, map)))
            {
                visitedFrontiers.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }

    return frontier;
}

Point<double> find_frontier_centroid(const frontier_t& frontier)
{
    // Using the mid point of the frontier
    Point<double> mid_point;
    int index = (int)(frontier.cells.size() / 2.0);
    // printf("index: %d, size: %d\n", index, frontier.cells.size());
    mid_point = frontier.cells[index];
    printf("Mid point of frontier: (%f,%f)\n", mid_point.x, mid_point.y);

    return mid_point;
    // // use the geometric median instead
    // float sum_x = 0;
    // float sum_y = 0;
    // for (auto &&cell : frontier.cells)
    // {
    //     sum_x += cell.x;
    //     sum_y += cell.y;
    // }
    // Point<double> centroid(
    //         sum_x / frontier.cells.size(),
    //         sum_y / frontier.cells.size()
    //         );
    // return centroid;
}
