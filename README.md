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

![image](./res/images/maze_concept.png)

For the maze escape solution, two scenarios were thought.

### Established scenario 

The established scenario for the project is that the robot has no "knowledge" about the environment. This means, the robot should do some exploration around the maze until it finds its way out.
The maze and the robot are going to be in a simulated environment inside Gazebo. The robot is going to always start in the center of the maze.  It should use the LiDAR sensor to detect its surroundings and to map the maze in RViz.
After the first escape, the map is saved for that specific maze. So, if the robot has to escape from the maze again, it will have a known route for this case.

## Project objectives

### General objectives
- Create a virtual environment inside Gazebo with a Turtlebot3 model "Burger" and a randomly generated maze in it.
- Develop python scripts for the localization of the Robot and mapping the environment around it.
- Implement an algorithm that allows the robot to navigate and map the maze without collision until the robot is out of the maze.
- Save the solution map for the maze and use it again for further escapes of the robot in the same maze.

## Specific objectives

- Simulate a robot in a Python environment.
- Implement a mathematical solution to escape a random maze for the Python simulation.
- With the solution, use an algorithm to identify the path from the exit to the starting point of the robot.
- Create an environment in Gazebo with a Maze that the Turtlebot3 can explore and solve.
- Get readings from the LiDAR sensor scan and 
- Implement the solution found for the Python simulation.
- 

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

                                                        
                                                     
