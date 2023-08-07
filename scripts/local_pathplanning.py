
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_about_axis

class RobotMove:

    def __init__(self):
        self.goal_reached = False
        # Create a SimpleActionClient for the move_base action
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

    def send_goal(self, goal_point, final_angle):

        # Wait for the action server to start
        self.client.wait_for_server()

        rospy.loginfo("Connected to move_base action server")

        # Create a goal position and orientation
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        quaternion_array = quaternion_about_axis(final_angle, (0,0,1 ))
		# quaternion_about_axis offers a convenient way for calculating the members of a quaternion.
		# In order to use it we need to convert it to a Quaternion message structure
		

        # Set the goal position
        # goal.target_pose.pose.position = Point(0.2, -1.4, 0.0) # 1small (0.4, -1.4)  2small (-0.5, -1.49)
        # goal.target_pose.pose.position = Point(-0.5, -1.49, 0.0)
        # goal.target_pose.pose.position = Point(1, -2.49, 0.0)

        # Set the goal orientation (as a quaternion+)
        #goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0,2.0)  # sets orientation of the target pose to be in the default orientation with no rotationQuaternion(0.0, 0.0, 0.0, 1.0)
        print(f"Moving to: {quaternion_array}")
        print(f"Moving to: {goal_point}")
        goal.target_pose.pose.orientation = Quaternion(quaternion_array[0],quaternion_array[1],quaternion_array[2],quaternion_array[3])

        #goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0,2.0)
        goal.target_pose.pose.position = Point(goal_point[0], goal_point[1], 0)
        

        # Send the goal to the action server
        rospy.loginfo("Sending goal")
        self.client.send_goal(goal)

        # Wait for the action to complete or fail
        self.client.wait_for_result()

        # Get the result of the action
        result = self.client.get_result()

        if result:
            rospy.loginfo("Goal reached!")
            self.goal_reached = True
        else:
            rospy.loginfo("Failed to reach the goal!")


if __name__ == '__main__':
    rospy.init_node('robot_move')
    my_goal = RobotMove()
    goal_point = [0.8, 0.8]
    try:
        my_goal.send_goal(goal_point)
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted by user")
