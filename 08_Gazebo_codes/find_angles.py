import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy.ndimage import gaussian_filter1d
import scipy.signal as ss


class range_detection:

    def __init__(self):
        # Initializing the robot Pose values
        self.robot_x = 0
        self.robot_y = 0
        self.robot_theta = 0

        # Initializing the Lidar values we require
        self.lidar_values = []
        self.sensor_min = 0.12 # Values below this should be discarded
        self.sensor_max = 1.5 # Change depending on the behaviour of the robot

        # Initialize basic variables
        rospy.init_node('sensor_reading', anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz
        self.lidar_scan = LaserScan()
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

    def lidar_callback(self, msg):
        self.lidar_scan = msg
        # print("Reading sensor")
        self.rate.sleep()


    def update_values(self):
        self.lidar_values = []
        for i in range(len(self.lidar_scan.ranges)):
            if (self.lidar_scan.ranges[i] > self.sensor_max):
                # Limiting the values from the max rage
                self.lidar_values.append(self.sensor_max)
            elif (self.lidar_scan.ranges[i] < self.sensor_min):
                # Limiting the values from the min range
                self.lidar_values.append(self.sensor_min)
            else:
                self.lidar_values.append(round(self.lidar_scan.ranges[i], 4))

        self.lidar_values = np.array(self.lidar_values)
        print(len(self.lidar_values))
        np.savetxt('lidar_values.txt', self.lidar_values, fmt='%.4f', delimiter=' ', newline='\n')
        print("Laser values saved as a TXT file!")
        return self.lidar_values

    def get_local_minima_v3(self, scan_dist, include_max=True, include_oor=True, safety_distance=0):
        scan_range = self.sensor_max
        #safety_distance += self.sensor_min

        scan_dist[scan_dist > scan_range] = -1
        oor = np.where(scan_dist == -1)[0]

        max_dist = np.array([[], []])
        loc_max_dist = np.array([[], []])
        oor_dist = np.array([[], []])

        if include_max:
            max_dist_idx = np.random.choice(np.where(scan_dist == np.max(scan_dist))[0])
            max_dist = np.max(scan_dist) - 1 if np.max(scan_dist) > 0 else scan_range - 1
            max_dist = np.array([[max_dist_idx], [max_dist - safety_distance]])

            loc_max_idx = ss.argrelextrema(scan_dist, np.greater)[0]

            loc_max_dist = scan_dist[loc_max_idx] - safety_distance
            loc_max_dist[loc_max_dist <= 0] = scan_range - 1
            loc_max_dist = np.vstack([loc_max_idx, loc_max_dist])

        if oor.size != 0 and include_oor:

            indecies = np.where((oor[:-1] - oor[1:]) != -1)[0]
            if indecies.size != 0:
                slices = []
                if (0 in oor) and (359 in oor):
                    # print('around 0')
                    slices.append([oor[indecies[0]], oor[indecies[-1] + 1] - 360])
                else:
                    slices.append([oor[0], oor[indecies[0]]])
                    slices.append([oor[indecies[-1] + 1], oor[-1]])

                for idx in range(indecies.size):

                    if idx != 0:
                        slices.append([oor[indecies[idx - 1] + 1], oor[indecies[idx]]])

                slices = np.array(slices)

                oor_dist = np.mean(slices, axis=1).astype(np.int16)
            else:
                oor_dist = np.array([np.mean([oor[0], oor[-1]])])

            oor_dist = np.vstack((oor_dist, np.full((oor_dist.shape), (scan_range - 1 - safety_distance))))

        pos_angles = np.concatenate([oor_dist, max_dist, loc_max_dist], axis=1)

        return pos_angles


    def path_selection(self):
        max_val = 0.6
        min_val = 0.15
        total_dir = 8
        dir_values = []
        dir_weights = []
        laser_values = self.update_values()
        total_values = len(laser_values)

        for j in range(total_dir):
            lower_range = int(j * (total_values / total_dir))
            upper_range = int((j + 1) * (total_values / total_dir))
            dir_values.append(laser_values[lower_range:upper_range])
            dir_weights.append(0)
        print(len(dir_values))

        for dir in range(len(dir_values)):
            for val in dir_values[dir]:
                if val > max_val:
                    dir_weights[dir] += 1
                elif val < min_val:
                    dir_weights[dir] -= 1
        print(dir_weights)
        max_index = np.array(dir_weights).argmax()
        print(max_index)
        desired_angle = (max_index * (total_values / total_dir) + (max_index + 1) * (total_values / total_dir)) / 2
        desired_angle = max_index * (total_values / total_dir)
        print(f"Desired angle of movement is {desired_angle}")
        return dir_weights, desired_angle

    def path_selection_v2(self):
        dist = self.update_values()
        scan_range = self.sensor_max # Maximum scan range
        sigma = 10  # Smoothing of sensor signal

        # Filtering for calculation of local Maxima/possible angles
        adj_dist = np.copy(dist)
        adj_dist = gaussian_filter1d(adj_dist, sigma)
        adj_dist[adj_dist > 1.5] = -1

        actions = self.get_local_minima_v3(adj_dist)

        print(actions)
        return actions


if __name__ == '__main__':
    print("Testing Lidar sensor readings")
    my_sensor = range_detection()
    finished = False
    while not finished:
        user_input = input("New reading? (Y/n): ")
        if user_input == 'Y':
            finished = False
            my_sensor.path_selection()
            pos_angles = my_sensor.path_selection_v2()
            #print(pos_angles)
        else:
            finished = True
            print("No more readings. Turning off the system")


