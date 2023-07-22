import rospy, sys, time
import numpy as np
from nav_msgs.srv import GetMap, GetMapRequest
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_msgs.msg import TFMessage
from tf2_geometry_msgs import PoseStamped
from sensor_msgs.msg import LaserScan, Imu
import tf2_ros  
import tf.transformations as tf
from std_msgs.msg import Int64MultiArray, MultiArrayLayout, MultiArrayDimension
import operator
import matplotlib.pyplot as plt

class RobotLocation:

    def __init__(self):
        # Subscribers
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        self.occupancy_grid_sub = rospy.Subscriber('/map', OccupancyGrid, self.occupancy_grid_callback)

        # Publishers
        self.map_pub = rospy.Publisher('/robot_location/2Dmap', Int64MultiArray, queue_size=10) #to publish location
        self.footprints_pub = rospy.Publisher('/robot_location/list_of_footprints', Int64MultiArray, queue_size=10) #to publish location

        self.occupancy_grid = OccupancyGrid()
        self.tf_msg = TFMessage()
        self.imu=Imu()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.robot_wrt_odom_translation = np.array([])
        self.robot_wrt_odom_orientation = np.array([])
        self.robot_wrt_map_orientation = np.array([])
        self.robot_wrt_map_translation = np.array([])
        self.pose = PoseStamped() 
        self.old_footprint = np.array([])
        self.list_of_footprints = list()
        self.old_map = np.array([])
        rospy.sleep(1)

    def scan_callback(self,data):
        self.ranges = data.ranges
        self.max_range = int(max(self.ranges)/0.05)

    def occupancy_grid_callback(self, data):
        self.occupancy_grid_data = data


        self.map_position_x = self.occupancy_grid_data.info.origin.position.x # origin of the map with respect to the absolute coordinate system
        self.map_position_y = self.occupancy_grid_data.info.origin.position.y # origin of the map with respect to the absolute coordinate system
        self.resolution = self.occupancy_grid_data.info.resolution # resolution of each cell, in meters. so 0.05 meters in width and in height  
        self.map_width = self.occupancy_grid_data.info.width # dimensions of the map
        self.map_height = self.occupancy_grid_data.info.height # dimensions of the map

        self.map_orientation_x = self.occupancy_grid_data.info.origin.orientation.x # orientation of the map
        self.map_orientation_y = self.occupancy_grid_data.info.origin.orientation.y # orientation of the map
        self.map_orientation_z = self.occupancy_grid_data.info.origin.orientation.z # orientation of the map
        self.map_orientation_w = self.occupancy_grid_data.info.origin.orientation.w # orientation of the map
        # print(self.map_width,self.map_height)
        # print(self.resolution)
        self.grid_data = np.reshape(np.asarray(self.occupancy_grid_data.data, dtype=np.int32),(self.map_width,self.map_height))
        # self.grid_data = np.rot90(self.grid_data, k=3)
        np.savetxt("raw_occupancy_grid.txt",self.grid_data, fmt='%i',delimiter="\t")

        # print(self.grid_data.shape)

    def tf_callback(self, data):
        self.tf_data = data

        for transform in data.transforms:
            # Get the frame IDs
            parent_frame = transform.header.frame_id
            child_frame = transform.child_frame_id
            
            try:
                trans = self.tfBuffer.lookup_transform(parent_frame, child_frame, rospy.Time(0))
                if child_frame == "base_footprint": # get pose and orientation with respect to the map (absolute) and the odom frame (relative)
                    
                    self.pose.header.frame_id = parent_frame
                    self.pose.pose.position.x = trans.transform.translation.x
                    self.pose.pose.position.y = trans.transform.translation.y
                    self.pose.pose.position.z = trans.transform.translation.z
                    
                    self.pose.pose.orientation.x = trans.transform.rotation.x
                    self.pose.pose.orientation.y = trans.transform.rotation.y
                    self.pose.pose.orientation.z = trans.transform.rotation.z
                    self.pose.pose.orientation.w = trans.transform.rotation.w
                    base_footprint_pose = self.tfBuffer.transform(self.pose, "map").pose

                    # saves the x,y,z, w values as an np array
                    self.robot_wrt_odom_translation = np.asarray([trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z])
                    self.robot_wrt_odom_orientation = np.asarray([trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w])

                    self.robot_wrt_map_orientation = np.asarray([base_footprint_pose.orientation.x,base_footprint_pose.orientation.y,base_footprint_pose.orientation.z,base_footprint_pose.orientation.w])
                    self.robot_wrt_map_translation = np.asarray([base_footprint_pose.position.x,base_footprint_pose.position.y,base_footprint_pose.position.z])

                    # print("Relative orientation:", self.robot_wrt_odom_orientation)
                    # print("Absolute orientation:", self.robot_wrt_map_orientation)
                    # print("Relative translation: ", self.robot_wrt_odom_translation)
                    # print("Absolute translation:", self.robot_wrt_map_translation)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(1)
                continue


    def set_robot_size(self,threshold=0.02,square=True):
        # adjust the robot of the robot
        robot_width = 0.18 # around 0.176
        robot_length = 0.14 # around 0.14 
        if square:
            self.robot_width = robot_width + threshold
            self.robot_height = self.robot_width
        else:
            self.robot_width = robot_width + threshold
            self.robot_height = robot_length + threshold
        return self.robot_width, self.robot_height
    
    def convert_to_map_pose(self): # get the position of the robot in the map, takes into account the size of the robot
        # gets the end to end cells in the grid that it occupies
        self.set_robot_size()
        x1 = np.int64((self.robot_wrt_map_translation[0] -self.robot_width/2 - self.map_position_x) / self.resolution)
        y1 = np.int64((self.robot_wrt_map_translation[1] -self.robot_height/2 - self.map_position_y) / self.resolution)

        x2 = np.int64((self.robot_wrt_map_translation[0] +self.robot_width/2 - self.map_position_x) / self.resolution)
        y2 = np.int64((self.robot_wrt_map_translation[1] +self.robot_height/2 - self.map_position_y) / self.resolution)
        return np.asarray([x1, y1,x2,y2]) # the lowest x2-x1, y2-y1 is the size of the robot in terms of the grid
        
    def get_robot_size(self): # gets the size / number of grid cells of the footprint    
        [x1, y1,x2,y2] = self.convert_to_map_pose()
        return (x2-x1+1)*(y2-y1+1)
    
    def convert_center_to_map_pose(self): # only the center of the robot is accounted for, may not be needed
        # the x position of the robot corresponds to the height of the map, 
        # and y position of the robot corresponds to the width of the map
        y = np.int64(((self.robot_wrt_map_translation[0] - self.map_position_x) / self.resolution)) #
        x = np.int64(((self.robot_wrt_map_translation[1] - self.map_position_y) / self.resolution)) #
        return np.asarray([y, x])
    

    def save_current_location(self):
        threshold = 1
        if self.old_footprint.size<1 or not np.allclose( self.old_footprint,self.convert_to_map_pose(), atol=threshold):
            self.list_of_footprints.append(self.convert_to_map_pose())
            self.old_footprint = self.convert_to_map_pose()

    def add_new_area(self,new_map,gradient_map):
        for row_index,row in enumerate(new_map):
            for column_index,column in enumerate(row):
                if gradient_map[row_index][column_index] % 3 == 0:
                    new_map[row_index][column_index] = gradient_map[row_index][column_index]
                else:
                    continue
        return new_map
                
    def iterate_occupancy_grid_v3(self,current_map,current_position,step_size,grad_size):
        grad = [[current_position[1],current_position[0]]] # center of the robot
        # grad size is the size of the gradient, 3 to skip the 100 of the wall
        # step size is the footprint size
        w_em,h_em = current_map.shape

        print(f'Occgrid Iterate: Footstep:{grad}, Map shape{current_map.shape}')

        for steps in range(step_size):
            #print(iteration)
            li=grad.copy()
            
            for P in li:
                if current_map[P[0],P[1]]!=100:
                    for i,j in [
                        [0,1],
                        [1,0],
                        [0,-1],
                        [-1,0],
                        [-1,-1],
                        [1,-1],
                        [-1,1],
                        [1,1],
                    ]:
                        if (0<=P[0]+i<w_em) and (0<=P[1]+j<h_em):

                            if current_map[P[0]+i,P[1]+j]!=100:
                                if [P[0]+i,P[1]+j] not in grad:
                                    grad.append([P[0]+i,P[1]+j])

        for P in grad:
            current_map[P[0],P[1]]+=grad_size
            #print("Currentposition",[[current_position[0],current_position[1]]] )
        return current_map

    
def main():
    rospy.init_node('robot_location')
    robot_location = RobotLocation()
    gradient_map = np.ones((384,384))
    grad_size = 7 # to skip the 100 which refers to the wall
    try:
        while not rospy.is_shutdown():
            if(robot_location.robot_wrt_map_translation.size>0): # when position is not empty
                # print(robot_location.grid_data)
                print("robot position:",robot_location.robot_wrt_map_translation,robot_location.convert_center_to_map_pose())
                gradient_map=robot_location.add_new_area(robot_location.grid_data,gradient_map) # adds the new area to the previous map
                gradient_map= robot_location.iterate_occupancy_grid_v3(gradient_map,
                                                                robot_location.convert_center_to_map_pose(), # gets the center of the robot
                                                                10, # takes into account size of robot
                                                                grad_size=grad_size
                                                        )
        
                a,b=robot_location.convert_center_to_map_pose()
                # gradient_map[a][b] = 50 # just to easily ctrl+F and find the current center, not needed in actual use
                np.savetxt("occupancy_grid_2.txt",gradient_map, fmt='%i',delimiter="\t") # saves the current gradient map to a file for reference
                
                ...
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
