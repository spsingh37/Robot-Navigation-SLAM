import heapq
import math
import numpy as np

ASTAR = 3
GREEDY = 2
DIJKST = 1
NONE = 0
class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def is_member(self, cell_x, cell_y):
        for node in self.elements:
            if (cell_x == node.x) & (cell_y == node.y):
                return True
        return False

    def put(self, item, priority1, priority2):
        heapq.heappush(self.elements, (priority1, priority2, item))

    def get(self):
        return heapq.heappop(self.elements)[2]

class NodeList:
    def __init__(self):
        self.nodes = []

    def put(self, node):
        self.nodes.append(node)

    def get(self, cell_x, cell_y):
        for node in self.nodes:
            if (cell_x == node.x) & (cell_y == node.y):
                return node

    def is_member(self, cell_x, cell_y):
        for node in self.nodes:
            if (cell_x == node.x) & (cell_y == node.y):
                return True
        return False

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = 0
        self.g_cost = 0
        self.h_cost = 0

    def f_cost(self):
        return self.g_cost + self.h_cost

    def is_in_list(self, node_list):
        return node_list.is_member(self.x, self.y)

    def is_in_map(self, map):
        map_x = map.shape[0]
        map_y = map.shape[1]
        return ((self.x < map_x)&(self.y < map_y)&(self.x >= 0)&(self.y >= 0))

    def is_obstacle(self, map):
        return (map[self.x][self.y] == 1)
    
    def is_free(self, map):
        return (map[self.x][self.y] == 0)
    
    def __str__(self):
        return "(%s, %s [%s, %s])" % (self.g_cost, self.h_cost,
                                      self.x, self.y)
    def __eq__(self, node):
        return (self.x == node.x) and (self.y == node.y)

    def __ne__(self, node):
        return not ((self.x == node.x) and (self.y == node.y))

    def __lt__(self, node):
        return (self.g_cost + self.h_cost) < (node.g_cost + node.h_cost)

    def __le__(self, node):
        return (self.g_cost + self.h_cost) <= (node.g_cost + node.h_cost)
    
    def __gt__(self, node):
        return (self.g_cost + self.h_cost) > (node.g_cost + node.h_cost)
    
    def __ge__(self, node):
        return (self.g_cost + self.h_cost) >= (node.g_cost + node.h_cost)



class AstarSearch:
    def __init__(self, start_x, start_y, goal_x, goal_y, map):
        self.map = map
        self.map_width = map.shape[0]
        self.map_height = map.shape[1]
        self.start = Node(start_x, start_y)
        self.goal = Node(goal_x, goal_y)
        self.open_list = PriorityQueue()
        self.open_list.put(self.start, 0, 0)
        self.closed_list = NodeList()
        self.searched_list = NodeList()
        self.path = NodeList()
        self.path_cells = []
        self.path_found = False

    def calc_h_cost_manhatten(self, cell):
        x = abs(self.goal.x - cell.x) 
        y = abs(self.goal.y - cell.y)
        return 10*(x + y)

    def calc_h_cost_crow(self, cell):
        x = 10*(self.goal.x - cell.x) 
        y = 10*(self.goal.y - cell.y)
        return math.floor(math.sqrt(x**2 + y**2))

    def calc_h_cost_grid(self, cell):
        x = abs(self.goal.x - cell.x) 
        y = abs(self.goal.y - cell.y)
        if(x >= y):
            cost = 14*y + 10*(x-y)
        else:
            cost = 14*x + 10*(y-x)
        return cost

    def calc_g_cost(self, parent, end):
        x = abs(parent.x - end.x) 
        y = abs(parent.y - end.y)
        if(x==1 & y==1):
            return parent.g_cost + 14
        else:
            return parent.g_cost + 10

    
    def find_path(self, algorithm, n_way):
        current_node = self.open_list.get()
        while(current_node != self.goal):
            self.closed_list.put(current_node)
            if(algorithm==1):
                self.expand_node_dijkstra(current_node,n_way)
            if(algorithm==2):
                self.expand_node_greedy(current_node,n_way)
            if(algorithm==3):
                self.expand_node_astar(current_node,n_way)
            current_node = self.open_list.get()
            if(current_node == self.goal):
                self.path_found = True
        self.extract_path(current_node)

    def step_path(self, algorithm, n_way):
        current_node = self.open_list.get()
        if(current_node != self.goal):
            self.closed_list.put(current_node)
            if(algorithm==1):
                self.expand_node_dijkstra(current_node,n_way)
            if(algorithm==2):
                self.expand_node_greedy(current_node,n_way)
            if(algorithm==3):
                self.expand_node_astar(current_node,n_way)
        else: 
            self.path_found = True
            self.extract_path(current_node)

    def expand_node_astar(self, node, mode):
        xDeltas = [1, -1, 0,  0, 1, 1, -1, -1]
        yDeltas = [0,  0, 1, -1, 1, -1, 1, -1]
        for i in range(mode):
            if(self.searched_list.is_member(node.x + xDeltas[i], node.y + yDeltas[i])):
                neighbor = self.searched_list.get(node.x + xDeltas[i], node.y + yDeltas[i])
            else:
                neighbor = Node(node.x + xDeltas[i], node.y + yDeltas[i])
            if (not (neighbor.is_in_list(self.closed_list)) and (neighbor.is_in_map(self.map)) and not (neighbor.is_obstacle(self.map))):
                if(not neighbor.is_in_list(self.searched_list)):
                    neighbor.g_cost = self.calc_g_cost(node, neighbor)
                    neighbor.h_cost = self.calc_h_cost_grid(neighbor)
                    neighbor.parent = node
                    self.open_list.put( neighbor, neighbor.f_cost(), neighbor.h_cost)
                    self.searched_list.put(neighbor)
                elif(neighbor.g_cost > self.calc_g_cost(node, neighbor)):
                    neighbor.g_cost = self.calc_g_cost(node, neighbor)
                    neighbor.parent = node
                    self.open_list.put(neighbor, neighbor.f_cost(),  neighbor.h_cost)

    def expand_node_greedy(self, node, mode):
        xDeltas = [1, -1, 0,  0, 1, 1, -1, -1]
        yDeltas = [0,  0, 1, -1, 1, -1, 1, -1]
        for i in range(mode):
            if(self.searched_list.is_member(node.x + xDeltas[i], node.y + yDeltas[i])):
                neighbor = self.searched_list.get(node.x + xDeltas[i], node.y + yDeltas[i])
            else:
                neighbor = Node(node.x + xDeltas[i], node.y + yDeltas[i])
            if (not (neighbor.is_in_list(self.closed_list)) and (neighbor.is_in_map(self.map)) and not (neighbor.is_obstacle(self.map))):
                if(not neighbor.is_in_list(self.searched_list)):
                    neighbor.g_cost = self.calc_g_cost(node, neighbor)
                    neighbor.h_cost = self.calc_h_cost_grid(neighbor)
                    neighbor.parent = node
                    self.open_list.put( neighbor, neighbor.h_cost, neighbor.h_cost)
                    self.searched_list.put(neighbor)
                elif(neighbor.g_cost > self.calc_g_cost(node, neighbor)):
                    neighbor.g_cost = self.calc_g_cost(node, neighbor)
                    neighbor.parent = node
                    self.open_list.put(neighbor, neighbor.h_cost,  neighbor.h_cost)

    def expand_node_dijkstra(self, node, mode):
        xDeltas = [1, -1, 0,  0, 1, 1, -1, -1]
        yDeltas = [0,  0, 1, -1, 1, -1, 1, -1]
        for i in range(mode):
            if(self.searched_list.is_member(node.x + xDeltas[i], node.y + yDeltas[i])):
                neighbor = self.searched_list.get(node.x + xDeltas[i], node.y + yDeltas[i])
            else:
                neighbor = Node(node.x + xDeltas[i], node.y + yDeltas[i])
            if (not (neighbor.is_in_list(self.closed_list)) and (neighbor.is_in_map(self.map)) and not (neighbor.is_obstacle(self.map))):
                if(not neighbor.is_in_list(self.searched_list)):
                    neighbor.g_cost = self.calc_g_cost(node, neighbor)
                    neighbor.h_cost = self.calc_h_cost_grid(neighbor)
                    neighbor.parent = node
                    self.open_list.put( neighbor, neighbor.g_cost, neighbor.g_cost)
                    self.searched_list.put(neighbor)
                elif(neighbor.g_cost > self.calc_g_cost(node, neighbor)):
                    neighbor.g_cost = self.calc_g_cost(node, neighbor)
                    neighbor.parent = node
                    self.open_list.put(neighbor, neighbor.g_cost,  neighbor.g_cost)

    def extract_path(self, node):
        current_node = node
        while(current_node != self.start):
            self.path.put(current_node)
            self.path_cells.append((current_node.x, current_node.y))
            print(current_node.x, current_node.y, current_node.h_cost, current_node.g_cost)
            current_node = current_node.parent
        self.path_cells.reverse()

def extract_start(map):
    result = np.where(map == 2)
    return list(zip(result[0], result[1]))[0]

def extract_goal(map):
    result = np.where(map == 3)
    return list(zip(result[0], result[1]))[0]

if __name__ == '__main__':
    map = np.array([[2, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 3]])

    (start_x, start_y) = extract_start(map)
    (goal_x, goal_y) = extract_goal(map)
    astar = AstarSearch(start_x,start_y,goal_x,goal_y,map)
    astar.find_path(ASTAR,8)
    print(astar.path_cells)