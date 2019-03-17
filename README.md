# motion-planning-algorithms
This is the folder for motion planning algorithms practice. 
- motion-planning-algorithms
   - RRT
      - rrt_algorithm:
         - basic_rrt.py : basis RRT planning algorithm
         - Replanning_RRT.py : replanning RRT based on Dave Ferguson et.al, Replanning with RRTs, 2006 
         - goal_change_rrt.py: Use basic RRT algorithm to plan robot's path and move the robot. The robot will replan the path if the goal changes.  
      - frrt:
         - flexible_rrt.py: Based on RRT* algorithm, I develop flexible RRT*. Considering the fact that human might need the robot to carry packages to some specific locations, the path will be biased to these locations. 
      - frrt_evaluate.py: Simulation to test the effectiveness of flexible RRT* compared to original RRT*. 
   - A-star
      - A-star.py : Implement basic A-star algorithm
      - breadth_first_search.py : Implementation of breadth first search algorithm
      - implementation.py : Useful classes for grid search forked from AtsushiSakai/PythonRobotics.git
      - search_class.py : Modified classes for A star algorithm
   - D-star
      - D_star.py : Implement basic D-star algorithm
      - search_class.py : Definition of useful classes for D start
