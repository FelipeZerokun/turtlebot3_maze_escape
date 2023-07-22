#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion

def send_goal():
    # Initialize the ROS node
    rospy.init_node('send_goal_node')

    # Create a SimpleActionClient for the move_base action
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")

    # Wait for the action server to start
    client.wait_for_server()

    rospy.loginfo("Connected to move_base action server")

    # Create a goal position and orientation
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"

    # Set the goal position
    #goal.target_pose.pose.position = Point(0.2, -1.4, 0.0) # 1small (0.4, -1.4)  2small (-0.5, -1.49)
    goal.target_pose.pose.position = Point(-0.5, -1.49, 0.0)
    # goal.target_pose.pose.position = Point(1, -2.49, 0.0)
    
    # Set the goal orientation (as a quaternion+)
    goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 2.0) # sets orientation of the target pose to be in the default orientation with no rotationQuaternion(0.0, 0.0, 0.0, 1.0)

    # Send the goal to the action server
    rospy.loginfo("Sending goal")
    client.send_goal(goal)

    # Wait for the action to complete or fail
    client.wait_for_result()

    # Get the result of the action
    result = client.get_result()

    if result:
        rospy.loginfo("Goal reached!")
    else:
        rospy.loginfo("Failed to reach the goal!")

if __name__ == '__main__':
    try:
        send_goal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted by user")
