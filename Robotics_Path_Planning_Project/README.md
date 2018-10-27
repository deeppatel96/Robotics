# Path Planning Project

Refer to the [assignment description](../assignment1.pdf) for full details of the path planning assignment behind this project.

This implementation is done in Python for a basic path planning problem in a two-dimensional space. The goal is to move a Turtlebot from a start to a goal location in Gazebo for different obstacle maps. The implementation is done for a grid-based approach and a graph based approach. For the grid-based approach, two algorithms are used to find a solution path: (1) the A* algorithm and (2) the free-direction A* (FDA*) algorithm, which are implemented in the files 'hw_astar.py' and 'hw_fdastar.py', respectively. The resulting paths are graphed and saved in folders map1, map2, etc. for A* and FDA*. Similarly, the problem is also solved in a graph based approach where a reduced visibility graph is built and then the A* algorithm is run on it to find the optimal path.

The complete report is in [Path Planning Report](../Path_Planning_Report.pdf).
