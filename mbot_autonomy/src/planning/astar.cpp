#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                             mbot_lcm_msgs::pose2D_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    bool found_path = false;
    
    // Create the open and closed lists
    std::vector<Node*> openList;
    std::vector<Node*> closedList;

    // Create the start and goal nodes
    Node* startNode = new Node(startCell.x, startCell.y);
    Node* goalNode = new Node(goalCell.x, goalCell.y);

    // Add the start node to the open list
    openList.push_back(startNode);

    // A* search algorithm
    while (!openList.empty()) {
        // Get the node with the lowest f_cost from the open list
        Node* currentNode = openList.front();
        for (auto node : openList) {
            if (node->f_cost < currentNode->f_cost) {
                currentNode = node;
            }
        }

        // Move the current node from the open list to the closed list
        openList.erase(std::remove(openList.begin(), openList.end(), currentNode), openList.end());
        closedList.push_back(currentNode);

        // Check if the goal node is reached
        if (*currentNode == *goalNode) {
            found_path = true;
            break;
        }

        // Expand the current node
        std::vector<Node*> neighbors = expand_node(currentNode, distances, params);
        for (auto neighbor : neighbors) {
            // Skip if neighbor is in the closed list
            if (is_in_list(neighbor, closedList)) continue;

            // Calculate tentative g_cost
            double tentative_g_cost = currentNode->g_cost + g_cost(currentNode, neighbor, distances, params);

            // If neighbor is not in the open list or tentative g_cost is lower
            if (!is_in_list(neighbor, openList) || tentative_g_cost < neighbor->g_cost) {
                neighbor->parent = currentNode;
                neighbor->g_cost = tentative_g_cost;
                neighbor->h_cost = h_cost(neighbor, goalNode, distances);
                neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;

                // Add neighbor to the open list if it's not already in it
                if (!is_in_list(neighbor, openList)) {
                    openList.push_back(neighbor);
                }
            }
        }
    }

    // Generate path if found
    mbot_lcm_msgs::path2D_t path;
    path.utime = start.utime;
    if (found_path) {
        auto nodePath = extract_node_path(goalNode, startNode);
        path.path = extract_pose_path(nodePath, distances);
        // Remove last pose, and add the goal pose
        path.path.pop_back();
        path.path.push_back(goal);
    } else {
        printf("[A*] Didn't find a path\n");
    }
    path.path_length = path.path.size();
    return path;
}

double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    double h_cost = 0.0;
    // Manhattan distance heuristic
    h_cost = abs(goal->cell.x - from->cell.x) + abs(goal->cell.y - from->cell.y);
    return h_cost;
}

double g_cost(Node* from, Node* to, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // Assuming a uniform cost between adjacent cells
    double g_cost = 0.0;
    ////////////////// TODO: Implement your goal cost, use obstacle distances: This is Done //////////////////////////
    // Manhattan distance for movement cost
    g_cost = abs(to->cell.x - from->cell.x) + abs(to->cell.y - from->cell.y);
    double obs_cost = 0.0;

    double goal_ob_dis = distances(goal->cell.x,goal->cell.y);
    
    if (goal_ob_dis < params.maxDistanceWithCost && goal_ob_dis > params.minDistanceToObstacle)
    {
        obs_cost = std::pow(params.maxDistanceWithCost - goal_ob_dis,params.distanceCostExponent);
    } 

    double from_ob_dis = distances(from->cell.x,from->cell.y);
    if (from_ob_dis < params.maxDistanceWithCost && from_ob_dis > params.minDistanceToObstacle)
    {
        obs_cost += std::pow(params.maxDistanceWithCost - from_ob_dis, params.distanceCostExponent);
    }
    g_cost = g_cost + obs_cost;


    
    
    return g_cost;

}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    std::vector<Node*> children;
    ////////////////// TODO: Implement your expand node algorithm: This is Done //////////////////////////
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};
    for (int n = 0; n < 8; ++n)
    {
        Node adjacentCell(node->cell.x + xDeltas[n], node->cell.y + yDeltas[n]);
        if (distances.isCellInGrid(adjacentCell.cell.x, adjacentCell.cell.y))
        {
            auto Distance = distances(adjacentCell.cell.x,adjacentCell.cell.y);
            if (Distance > params.minDistanceToObstacle)
            {
                Node *newNode = new Node(adjacentCell.cell.x,adjacentCell.cell.y);
                children.push_back(newNode);
            }
            
        }
    }

    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    std::vector<Node*> path;
    // Traverse nodes and add parent nodes to the vector
    Node* currentNode = goal_node;
    while (currentNode != nullptr) {
        path.push_back(currentNode);
        currentNode = currentNode->parent;
    }
    // Reverse path
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    std::vector<mbot_lcm_msgs::pose2D_t> path;
    // Convert nodes to poses
    for (auto node : nodes) {
        mbot_lcm_msgs::pose2D_t pose;
        pose.x = distances.origin_x + node->x * distances.resolution;
        pose.y = distances.origin_y + node->y * distances.resolution;
        // Assuming constant heading for simplicity
        pose.theta = 0.0;
        path.push_back(pose);
    }
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto&& item : list) {
        if (*node == *item) return true;
    }
    return false;
}

