# Maze escape project with Turtlebot3

Project for the lecture FWP: ROS in the Deggendorf Institute of Technology for Summer semester 2023

Professor: Dmitrii Dobriborsc

Group participants:
- Gil Angeles
- Ruben Contreras
- Felix Gatti
- Felipe Rojas

## Project description

The project aim is to develop and implement a search or navigation algorithm that lets the Turtlebot3 solve different kind of maze configurations effectively in a virtual environment.

![image](./06_images/maze_concept.png)

For the maze escape solution, two scenarios were thought.

### First scenario

The first scenario supposes that the robot has no "knowledge" about the environment. This means, it was to explore everything until it finds and exit of the maze.
For subsequent escapes from the maze, the robot will already "know" its way inside the maze and escape in much less time.

### Second scenario

For the second scenario, the robot has some information about the location of the exit. In this approach, the robot will try to escape while minimizing the exploration and the time inside the maze.


## General objectives

- Create an environment in Gazebo with a Maze that the Turtlebot3 can explore and solve.

- Defining a localization/mapping method to navigate the maze in an ordered behaviour.

- Implement a search algorithm for the Turtlebot3 so it can get out of the maze in the fastest way possible.

- Implement the same algorithm in a Real-world scenario, where a Physical Turtlebot3 is able to solve a simple maze.

## Project components

This project uses the Turtlebot3 Burger robot, which has a LiDAR sensor incorporated and will be used for the system navigation.
The other components that we will be working on are the following:

### Maze

Two mazes were randomly generated in the following [site.](https://www.mazegenerator.net)
The mazes were then build up inside Gazebo

                                                                        
<img
  src="./01_Maze-Worlds/Maze_2.png"
  alt="maze_01"
  title="Small maze"
  style="display: inline-block; margin: 0 auto; max-width: 300px">


### Navigation

For the navigation, the in-build SLAM package that generates a map using LiDar sensor will be used so that the robot is able to navigate through the maze without bumping into the walls and can find a way out of the maze                                                                


### Escape algorithm

For the two scenarios there are different algorithms that can be used, but one is commonly used for this systems.

The A* algorithim will be explored and we will try to use it for this project.

                                                        
                                                     
