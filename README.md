# Map_Router

## Project compares two search algorithms - A* Search and Greedy Best First

* In this project we will consider an Initial state and a Goal state and find an optimal shortest path between them using two different algorithms
* In A* Search algorithm we add every state's distance with it's heuristic value to find the least farthest state from the current state chosen
* In Greedy Best First Solution, we will choose the next state only if it's distance is least compared to all the other neighboring states of current state


## Input File

* driving.csv - Driving distance between two states
* straightline.csv: Straight line distance between state capitals

## Priority Queue
* We use priority queue to sort all the adjacency list states with respect to their distances
* The one with least distance is popped

## State class

* Maintain a list variables needed for the algorithm - state_name, connected_states, path_cost, root, heuristics, and list_of_connected_states
> _Note: For A* we add heuristics value and for BFS just the path cost_
* List of connected states is formed using the driving.csv file
> _Note: We are using pandas to read the csv files_

## Working of the 'Map_Router'
* We call the 'search_path' function twice
* This function needs three attributes as input - Initial state, goal state, and the algorithm
* We even provide how fast each algorithm works by using timestamps

## Driving distance function
In this we are checking the condition that:
* If there is an 'int' value between chosen states (row,column) then output the 'int' value
* Else, return 'FALSE' - because there is no path between those two states

## Straight distance function
* If the state capitals are directly connected then output their distance in datatype 'int'
* Else, return 'FALSE'

## Search_Path function
* This function takes three attributes - Initial state, Goal State, and Algorithm name.
* First we check if initial=goal, if so the distance is considered to be 0
* If not, then for each algorithm type we execute below steps:
    * A* Search Algorithm:
        * We use queue.put() function to get the smallest distance state next
        * Using adjacency list data structure we calculate the path cost and driving distance cost 
        * We add heuristics value to oth these costs
    * Greedy Best First Search:
        * Similar to A* we use queue.put() function to get the smallest distance to next state
        * Then we just recursively add the entire path cost until final destination is reached

## Output
By the end of our algorithm, we return the solution in below format:

> Greedy Best First Search:
>
> 	Solution path: STATE1, STATE2, STATE3, …, STATEN-1, STATE N
>
>	Number of states on a path: X1
>
>	Path cost: Y1
>
>	Execution time: T1 seconds
>
> A* Search:
>
>	Solution path: STATE1, STATE2, STATE3, …, STATEN-1, STATE N
>
>	Number of states on a path: X2
>
>	Path cost: Y2
>
>	Execution time: T2 seconds

## Program execution

Run this file using below command:

    >$ python maprouter.py <initial_state> <goal_state>


> _Note: This project uses python3.7_
