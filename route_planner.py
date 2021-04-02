import math 
import heapq

class Node():
    """
    A node class for A* algorithm
    """
    
    def __init__(self,position=None, priority=None):
        self.position = position
        self.priority = priority  # F = G + H
        
        
    def __eq__(self,other):
        return self.position == other.position
    
    def __lt__(self,other):
        return self.priority < other.priority



def distance(x1,y1,x2,y2):
    """
    Method to get the Euclidean distance between the intersections
    """
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

 
def shortest_path(M,start,goal):
    
    """
    This is an adaptation of the A* algorithm from https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
    
    """
    # Initiate variables
    
    track_prev_nodes = {}
    current_cost = {}
    
    roads = M.roads # ge the roads
    intersections = M.intersections # get the intersections 
    
    frontier = [] # initiate a frontier to be explored in the network
    
    heapq.heapify(frontier) # make it a priority queue
    
    start_node = Node(start,0)
    goal_node = Node(goal,0)
    
    heapq.heappush(frontier,start_node) # add the start node to the frontier
    
    track_prev_nodes[start] = None
    current_cost[start] = 0
    
    path = []
    
    while len(frontier) > 0:
        
        current_node = heapq.heappop(frontier) # get the lowest poriority node
        
        # end if we have reached our goal
        if current_node == goal_node:
            break
            
        # loog through all possible neighbors and examine
        for neighbour in roads[current_node.position]:
            #get the x,y coordinates on the current node
            current_coordinate = intersections[current_node.position]
            x_current = current_coordinate[0]
            y_current = current_coordinate[1]
            
            #get the x,y coordinate of the neighbour node
            neighbour_coordinate = intersections[neighbour]
            x_neighbour = neighbour_coordinate[0]
            y_neighbour = neighbour_coordinate[1]
            
            # calculate the cost from the current to the intersection of the neighbour
            distance_cost = distance(x_current,y_current,x_neighbour,y_neighbour)
            new_cost = current_cost[current_node.position] + distance_cost
            
            if neighbour not in current_cost.keys() or new_cost < current_cost[neighbour]:
                current_cost[neighbour] = new_cost
                
                #get the x,y coordinate of our goal or destination
                goal_coordinate = intersections[goal]
                x_goal = goal_coordinate[0]
                y_goal = goal_coordinate[1]
                
                # now, we compute the priority to our goal, F = G + H
                priority = new_cost + distance(x_goal,y_goal, x_neighbour,y_neighbour)
                
                next_node = Node(neighbour,priority)
                heapq.heappush(frontier,next_node) # push the next_node onto the queue
                
                track_prev_nodes[neighbour] = current_node.position # tack the neighbouring nodes
    
    # now we can get the shortest path and return             
    while goal != start:
        path.append(goal)  
        goal = track_prev_nodes[goal]
    path.append(start)
    path.reverse()
            
    return path
