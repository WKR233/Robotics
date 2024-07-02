import numpy as np
import queue
import sys

# load the params
#rows = 473
#cols = 436

# load the map
def loadmap(file_path):
    Map = []
    with open(file_path, 'r') as file:
        for line in file:
            blocks = line.split()
            Map.append(blocks)
    return Map

# initialize the parent and cost
def init(rows, cols):
    for i in range(rows):
        Parent = []
        Cost = []
        parent = []
        cost = []
        for j in range(cols):
            parent.append((-1, -1))
            cost.append(1000000)
        Parent.append(parent)
        Cost.append(cost)
        return Parent, Cost

# Heuristic Function
def H(x_point, goal_point):
    return np.sqrt((x_point[0]-goal_point[0])**2+(x_point[1]-goal_point[1])**2)

# The map point and its cost
class Point:
    def __init__(self, cost, point):
        self.cost = cost
        self.point = point
    
    def __lt__(self, other):
        return self.cost<other.cost
    
    def __eq__(self, other):
        return self.point==other.point

def findpath(start_point, goal_point, Map, rows, cols, Cost, Parent):
    Q = queue.PriorityQueue()
    Q.put(Point(H(start_point),start_point))
    Cost[start_point[0]][start_point[1]] = H(start_point)

    # The priority queue
    path = []
    close_set = set()
    displacements = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    while Q.empty()==False:
        node = Q.get()
        if node.point == goal_point:
            print(node.cost)
            item = goal_point
            while Parent[item[0]][item[1]] != (-1, -1):
                path.append(item)
                item = Parent[item[0]][item[1]]
            break
        if node.point not in close_set:
            close_set.add(node.point)
            for d in displacements:
                child_point = (node.point[0]+d[0], node.point[1]+d[1])
                if 0<=child_point[0]<rows and 0<=child_point[1]<cols and Map[child_point[0]][child_point[1]]=='0':
                    child_cost = node.cost-H(node.point)+np.sqrt(d[0]**2+d[1]**2)+H(child_point)
                    child_node = Point(child_cost, child_point)
                    if Cost[child_point[0]][child_point[1]] > child_cost:
                        Cost[child_point[0]][child_point[1]] = child_cost
                        Parent[child_point[0]][child_point[1]] = node.point
                        Q.put(child_node)

    path.reverse()
    
    return path