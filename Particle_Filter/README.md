# Localization with Particle Filter Project

Refer to the [assignment description](assignment3.pdf) for full details of the path planning assignment behind this project.

This implementation integrates Python code with ROS commands to a Turtlebot in Gazebo. The robot has a sensor attached to it that return laser scan data. Given multiple maps of the world, the TurtleBot performs indeterministic (noisy) actions in the environments and returns noisy scan data. The localization of the TurtleBot in the environment is solved using a particle filter approach with Sequential Importance Resampling. The results of some worlds at different action and observation noise levels can be found in the [graphs folder](graphs). 

The algorithm is also tested to solved the localization problem with an unknown initial location. Example of these results for different environments can be found in the [no_initialization_gifs folder](no_initialization_gifs). 

The complete report can be found at [report.pdf](report.pdf).
