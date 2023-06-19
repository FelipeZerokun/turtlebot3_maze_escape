# SCARA robot for sorting between Cylindrical and Cubical objects in Gazebo

**Master course**: Artificial Intelligence for Smart Sensors and Actuators
**Lecture**: Case Study: Autonomous Systems
**Professor**: Ginu

### Group members:
- Angeles Gil
- Contreras Ruben
- Gatti Felix
- Rojas Felipe

## Project Description

The project simulates a working enviroment of a sorting system using SCARA-configuration robot that is able to pickup objects and sort them into different containers depending on their form

For the object detection, a depth camera on top of the objects is located and is able to discern between cubical and cylindrical objects by their contour.

For the path planning, the robot will go to and from the picking position to the containers following a *circular path*. After dropping the objects in the containers, it will return to the IDLE state in a *linear* path

## Step-by-step process

The sorting process is the following:
1. The robot will move to an pre-defined IDLE position, and will wait there for the next object.
2. When a object enters the detection zone, the camera will determine if the object is a cube or a cylinder.
3. When the object is in the pickup position, the robot will calculate the path it needs to follow to reach that position.
4. Depending on the detected object, a new path will be calculate from the pickup position to a random point generated in the corresponding container.
	1. If it is a cylinder, it will be moved to the nearest container.
	2. If it is a cube, it will be moved to the furtest container.
5. The robot will drop the object in the container, and will calculate its path back to the IDLE position.
6. The process will repeat for the next object indefinitely.

### Environment setup

The project was made for Ubuntu Focal (20.04), ROS noetic distribution and a Gazebo version XX.XX.
To setup the environment, follow this steps:


1. Copy the "scara_robot_project" folder into your src directory and then use catkin_make command
2. Then install the controller plugin:
	1. sudo apt update
	2. sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
	3. sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

## Running the simulation

Once the package has been setup and the controllers have been installed, the simulation on Gazebo can be launched by following the next steps:

1. launch Gazebo environment package
```python
roslaunch scara_robot_project initialization.launch
```
When Gazebo opens, you should be able to see the working environment with:
	1. The SCARA robot in a random position
	2. A "Conveyor belt" next to the robot
	3. Two containers, each one for putting the Cubes or the cylinders.
	4. A camera on top of the "conveyor belt"

2. In a new terminal, launch Object detection script
```python
python3 detect_object_with_area_threshold.py
```
The output of the camera should be printed in the terminal indicating which object is detected, or if there is no object at all.

3. In a new terminal, launch the Master Controller script
```python
python3 master_controller_w_pathplanning.py
```
Now the system loop will begin. In the terminal can be seen an output like the following:
```
Sorting Robot - Iteration = 0
==================================================
Real Obj ->                 Cube
Detected ->                 Cube

Move: Resting -> Pickup
Motion: circular -> 1452 Steps
Step 1451: Reachable = True, Theta 0 = 0.6097, Theta 1 = 1.2694

Move: Pickup -> Cube Box
Box center: [ 0.445 -0.255]
Drop location randomized: [ 0.394 -0.183]
Motion: circular -> 723 Steps
Step 722: Reachable = True, Theta 0 = -1.0270, Theta 1 = 1.3277

Move: Cube Box -> Resting Position
Motion: linear -> 718 Steps
Step 717: Reachable = True, Theta 0 = -0.3132, Theta 1 = 1.1916
==================================================
```

Indicating:
1. The Iteration step
2. The object detected by the camera module.
3. The movement the robot is performing at the moment. It can be:
	1. Resting to Pickup
	2. Pickup to Box
	3. Box to Resting
4. The location where the robot will drop the object in the respective container
5. The motion path that it is performing (circular or linear) 
6. Details of the path planning algorithm including:
	1. number of steps needed for that particular path
	2. If the position is reachable or not
	3. Position of the first link
	4. Position of the second link

