
# Map Router

import pandas as pd
from queue import PriorityQueue
import time
import sys
# import ipdb

# Here we are taking two arguments from user for finding the distance between two states.
if(len(sys.argv) == 3):
    # ipdb.set_trace()
    origin_state = sys.argv[1] # Source/Origin
    dest_state = sys.argv[2] # Destination
    straight_line = pd.read_csv("straightline.csv", index_col=0) # Reading given straightline.csv file using pandas library. 
    driving_distance = pd.read_csv("driving.csv", index_col=0) # Reading given driving.csv file using pandas library. 
    list_of_states = list(driving_distance.index) # Create a list of all the states.
    class state():
        def __init__(self, state_name, search_algorithm, heuristics, root=None, path_cost=0):
            self.state_name = state_name # This is used for Destination or Source/Origin
            self.connected_states = [] # Initializing an empty array for adding all the connected nodes between Source and Destination.
            self.path_cost_origin_to_node = path_cost # Total cost b/w Source and Destination.
            self.root = root # Test if each node has a parent. For backtracking.
            self.heuristics = heuristics # Additional data or weight b/w two intermediate states.
            self.list_of_connected_states()     
            if search_algorithm == 'a_star':
                self.ENV = self.path_cost_origin_to_node + self.heuristics
            elif search_algorithm == 'greedy_bfs':
                self.ENV = self.heuristics
        
        def __lt__(self, other):
            return self.get_env() < other.get_env()

        def get_env(self):
            return self.ENV

        # If the weight b/w Source and the Destination is equal to -1 then there is no path b/w those two states.
        def list_of_connected_states(self):
            for i in list_of_states:
                if(driving_distance[self.state_name][i] != -1 and i != self.state_name):
                    self.connected_states.append(i)

    # Min distance b/w each intermediate state from Source to Destination.
    def driving_distance_btw_states(state1, state2):
        if ((state1 in driving_distance.index) and (state2 in driving_distance.columns)):
            return (int(driving_distance[state1][state2]), state1)
        else:
            return False

    # Heuristics distance b/w two states. We use this later for the calculation of path cost in A* algorithm.
    def straight_distance_btw_states(state1, state2):
        if ((state1 in straight_line.index) and (state2 in straight_line.columns)):
            return (int(straight_line[state1][state2]), state1)
        else:
            return False

    # Finds the optimal path acc to GBST and A*
    def search_path(origin_state, dest_state, algo):
        src = origin_state
        dest = dest_state
        algo_used = algo
        frontier = PriorityQueue() # All distances are sorted, except for -1, and the least positive value is considered as a priority.
        explored = dict() # Initializing an empty dictionary for getting optimal states/paths that were explored.
        if(src == dest): # If your Source and Destination are the same.
            dest = state(dest_state, algo_used, 0) # The heuristics value is 0.
            explored[dest.state_name] = dest # So the explored dictionary will only contain the Destination state name.
            return explored

        else: # If the source and destination are different
            dist = straight_distance_btw_states(src, dest) # We find the distance b/w them.
            if(not dist): # If the distance is null
                return False # There is no path b/w Source and Destination
            else: # If the distance is a finite value.
                if(algo_used == "a_star"): # Verify if the algorithm is A*.
                    dist = dist[0] # Initially, distance is intialized to 0.
                src = state(origin_state, algo_used, dist) # Here we are calling class and passing dist as the heuristics value.
                dest = state(dest_state, algo_used, 0) # Here, since the destination and goal is same the heuristics value is 0.
                frontier.put((dist, src)) # Acc to priority queue, the least distance is considered as the first hop from start.
                explored[src.state_name] = src # Now, we add this start or source in explored dictionary.
                while (not frontier.empty()): # Run until the priority queue is not empty. That is it will be empty once it reaches the destination.
                    curr_node = frontier.get()[1] # Source state
                    if(curr_node.state_name == dest.state_name): # If the Source and destination is same, then the path cost will be 0 and the explored state will be the Destination State.
                        dest = curr_node
                        explored[dest.state_name] = dest
                        return explored
                    else: # If both source and destination are different...
                        for adj_node in curr_node.connected_states: # We consider each adjacent/connected node to the source...
                            p_cost = curr_node.path_cost_origin_to_node # Count of states to travel from Source to Destination.
                            d_cost = driving_distance_btw_states(
                                curr_node.state_name, adj_node)[0] # Driving distance b/w two intermediate states, from Source to Destination.( edge weight )
                            straightline_cost = straight_distance_btw_states(adj_node, dest.state_name)[
                                0] # Heuristic cost
                            neighbor = state(
                                adj_node, algo_used, straightline_cost, root=curr_node, path_cost=p_cost+d_cost) # Runs the class with all the parameters to get the final optimal path b/w source and destination.
                            if(neighbor.state_name not in explored.keys() or (neighbor.path_cost_origin_to_node < explored[neighbor.state_name].path_cost_origin_to_node)): # if the current found path weight is less than the already explored path weight...
                                explored[neighbor.state_name] = neighbor # Consider current path weight as the final distance or path b/w source and destination.
                                frontier.put((neighbor.ENV, neighbor)) # Return the value of all states b/w Source and Destination and the path_cost.
                return False # If, there is no path between Source and destination, the no. of states in the path = 1 and the path_cost = 0. Because, Source = Destination.

    # Explores the paths b/w Source and Destination
    def solution_path(dest_state):
        dest = dest_state
        path_list = []
        while (dest.root is not None):
            path_list.append(dest.state_name)
            dest = dest.root
        path_list.append(origin_state)
        path_list.reverse()
        return path_list

    a_star_start_time = time.time() # Captures the timestamp at the start of a* algo
    a_star = search_path(origin_state, dest_state, "a_star") # The search function is called to run this for A* algorithm type and returns the value of path_cost + heuristics.
    a_star_end_time = time.time() # Captures the timestamp after the execution of search function of A* algorithm.
    gbfs_start_time = time.time() # Capture the timestamp at the start of gfs algo.
    gbfs = search_path(origin_state, dest_state, "greedy_bfs") # For GBFS algo, this routes to class for returning the heuristics value.
    gbfs_end_time = time.time() # Captures the timpstamp after the execution of search function of gfs algo.

    # Representation of data on o/p or console.
    print("")
    print('Bodhankar, Komal, A20492705 solution:')
    print('Initial state: ',origin_state)
    print('Goal state: ',dest_state)
    print("")
    if(gbfs):
        g = gbfs[dest_state]
        pta1 = solution_path(g)
        print('Greedy Best First Search:')
        print('Solutionpath: ',pta1)
        print('Number of states on a path: ',len(pta1))
        print('Path Cost: ', g.path_cost_origin_to_node)
        print('Execution time: ',gbfs_end_time - gbfs_start_time,' seconds')
    else:
        print("FAILURE: NO PATH FOUND") 
    print("")
    if(a_star):
        a = a_star[dest_state]
        pta = solution_path(a)
        print('A* Search:')
        print('Solutionpath: ', pta)
        print('Number of states on a path: ', len(pta))
        print('Path Cost: ', a.path_cost_origin_to_node)
        print('Execution time: ', a_star_end_time - a_star_start_time,' seconds')
    else:
        print("FAILIURE: NO PATH FOUND")
    print("")
else:
    print("ERROR: INCORRECT INPUT ARGUMENTS.") #Few or more than required input arguments
