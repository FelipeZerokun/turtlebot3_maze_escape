\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*

***__IMPORTANT__***

***__Point location defined in meter for current Path planning:__***

* ***Spawn location (object): x=0.17 ; y=0.41***
* ***Box for dropping squares: x=0.32 ; y=-0.38***
* ***Box for dropping cylinders(next to converyor): x=0.32; y=-0.08***

***__Angle values in radians for robot (J0 and J1) to do current Path planning:__***

* ***__go to pick up: J0=0.577 ; J1=1.325__***
* ***__go to drop square: J0=-0 .165 ; J1=-0.766__***
* ***__got to drop cylinder: J0=-0.0462 ; J1=1.1249__***
* 

***__Values for J2(up/down) AND for J4(open/close)__***

* ***J2 --> up=0.05 ; down=-0.03***
* ***J4--> open=-0.001 ; close=0.035***

\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*\*



# SCARA robot for sorting between Cylindrical and Cubical objects in Gazebo

### Project Description

The project simulates a working enviroment of a sorting system using SCARA-configuration robot that is able to pickup objects and sort them into different containers depending on their form

For the object detection, a depth camera on top of the objects is located and is able to discern between cubical and cylindrical objects by their contour.

For the path planning, the robot will go to and from the picking position to the containers following a *circular path*. After dropping the objects in the containers, it will return to the IDLE state in a *linear* path

### Step-by-step process

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

   ```
   1\. sudo apt update
   
   2\. sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
   
   3\. sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
   
   ```

**__GAZEBO ENVIRONTMENT AND SIMULATION__**

1. launch Gazebo environment using

   ```
   1\. roslaunch scara_robot_project initialization.launch
   
   ```
2. launch spawn object

   ```
   2\. rosrun scara_robot_project ...
   
   ```
3. launch Master Controller

   ```
   3\. rosrun scara_robot_project ...
   
   ```
