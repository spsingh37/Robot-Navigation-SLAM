#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>

ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}

void ObstacleDistanceGrid::initializeDistances(const OccupancyGrid& map)
{
    int width = map.widthInCells();
    int height = map.heightInCells();
    cell_t cell;

    for (cell.y = 0; cell.y < height; ++cell.y)
    {
        for (cell.x = 0; cell.x < width; ++cell.x)
        {
            if (is_cell_free(cell, map))
            {
                distance(cell.x, cell.y) = -1.0; // This is Done: Mark as uninitialized
            }
            else if (is_cell_occupied(cell, map))
            {
                distance(cell.x, cell.y) = 0.0; // This is Done: Mark as obstacle
            }
            else {
                distance(cell.x, cell.y) = 0.0; // This is Done: Mark as free
            }
        }
    }
}

void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);

    initializeDistances(map); // This is Done: Initialize distances

    std::priority_queue<DistanceNode> searchQueue;
    enqueue_obstacle_cells(map, *this, searchQueue);

    while (!searchQueue.empty())
    {
        auto nextNode = searchQueue.top();
        searchQueue.pop();

        float& node_distance = cells_[cellIndex(nextNode.cell.x, nextNode.cell.y)];

        if (node_distance == -1) // This is Done: Check if not yet processed
        {
            node_distance = nextNode.distance * metersPerCell(); // This is Done: Update distance
            expand_node(nextNode, *this, searchQueue); // This is Done: Enqueue neighboring cells
        }
    }
}

void expand_node(const DistanceNode& node, ObstacleDistanceGrid& grid, std::priority_queue<DistanceNode>& search_queue)
{
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};

    for (int n = 0; n < 8; ++n)
    {
        cell_t adjacentCell(node.cell.x + xDeltas[n], node.cell.y + yDeltas[n]);
        if (grid.isCellInGrid(adjacentCell.x, adjacentCell.y))
        {
            if (grid(adjacentCell.x, adjacentCell.y) < 0) // This is Done: Check if not yet visited
            {
                float distance = node.distance + (n < 4 ? 1.0 : 1.414); // This is Done: Update distance based on movement type
                DistanceNode adjacentNode(adjacentCell, distance);
                grid(adjacentCell.x, adjacentCell.y) = adjacentNode.distance * grid.metersPerCell(); // This is Done: Set distance in grid
                search_queue.push(adjacentNode); // This is Done: Enqueue neighboring cell for further expansion
            }
        }
    }
}

bool is_cell_free(cell_t cell, const OccupancyGrid& map)
{
    return map.logOdds(cell.x, cell.y) < 0;
}

bool is_cell_occupied(cell_t cell, const OccupancyGrid& map)
{
    return map.logOdds(cell.x, cell.y) >= 0;
}
