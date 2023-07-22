#!/usr/bin/env python3

import numpy as np
import math
from PIL import Image as im
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy

from find_angles import RangeDetection
from robot_location import RobotLocation
from local_pathplanning import RobotMove
import matplotlib.pyplot as plt


def draw_map():
    map = np.loadtxt('occupancy_grid_2.txt')
    print(f"map size is {map.shape}")

    map_image = im.fromarray(map)
    map_image = map_image.convert('RGB')
    print("Saving the map as a image")
    map_image.save('final_map.png')



if __name__ == '__main__':
        
    finished = False
    move_dist = 0.45 # distance for the robot to move straight (in m)
    gradient_map = np.ones((384,384))
    grad_size = 6 # to skip the 100 which refers to the wall
    footprint_size = 15
    lidar_sensor =  RangeDetection()
    robot_location = RobotLocation()
    robot_move = RobotMove()
    iteration = 0
    


    try:
        while not finished and not rospy.is_shutdown():
            
            if(robot_location.robot_wrt_map_translation.size>0): # when position is not empty

                iteration+=1
                print(f'\n\nIteration {iteration}')
                print(50*'=')

                robot_map_pos = robot_location.convert_center_to_map_pose()
                pos_angles = lidar_sensor.path_selection_v2()
                print(f"Possible Angles: {np.around(pos_angles[0,:],decimals=2)}")
                print(f"Assosiated Dist: {np.around(pos_angles[1,:],decimals=2)}")

                # Getting the Robot GLOBAL coordinates for movement
                robot_global_pos = [robot_location.robot_wrt_odom_translation[0], robot_location.robot_wrt_odom_translation[1]]
                robot_global_quaternion = robot_location.robot_wrt_odom_orientation
                robot_global_orientation = euler_from_quaternion(robot_global_quaternion)[-1]

                if robot_global_orientation < 0:
                    robot_global_orientation = robot_global_orientation + 2*np.pi

                angle = robot_global_orientation
                
                print(f"\nCurrent Angle: {np.around(angle*180/np.pi,decimals=2)}")
                print(f'Map data shape: {robot_location.grid_data.shape}')
                print(f'Map target shape: {gradient_map.shape}')

                gradient_map=robot_location.add_new_area(robot_location.grid_data,gradient_map) # adds the new area to the previous map
                print(f'Map Overlay shape: {gradient_map.shape}')

                gradient_map= robot_location.iterate_occupancy_grid_v3(gradient_map,robot_map_pos, step_size=footprint_size, grad_size=grad_size)
                


                np.savetxt("occupancy_grid_2.txt",gradient_map, fmt='%i',delimiter="\t") # saves the current gradient map to a file for reference


                fig = plt.figure()
                ax = fig.add_subplot()
                gdmp =ax.imshow(gradient_map,cmap='gray',origin='lower')
                fig.colorbar(gdmp,ax=ax)
                fig.savefig('Gradient_map.png')


                
                p_grad = []
                pos_indecies = []

                for i in range(len(pos_angles[0])):
                        
                    # Calculate de X and Y values of the possible point to go with respect to the Robot location
                    beta =pos_angles[0][i]*np.pi/180+angle

                    point_from_robot_X = pos_angles[1][i] * math.cos(beta)
                    point_from_robot_Y = pos_angles[1][i] * math.sin(beta)
                    
                    dx_pixel = int((point_from_robot_X)/0.05)
                    dy_pixel = int((point_from_robot_Y)/0.05)
                    
                    # Obtain the gradiend value of the Possible point
                    x_grad = int(dx_pixel +robot_map_pos[0])
                    y_grad = int(dy_pixel + robot_map_pos[1])
                    #print("Pixel values for the map are: ")
                    #print(x_grad, y_grad)
                    grad_value = gradient_map[y_grad, x_grad]

                    x_target = point_from_robot_X + robot_global_pos[0]
                    y_target = point_from_robot_Y + robot_global_pos[1]


                    if grad_value!=-1:
                        p_grad.append(grad_value)
                        pos_indecies.append(i)
                    
                    elif (y_target>3.0) and (-1.5<x_target<0.5):
                        best_point = [x_target, y_target]
                        target_orientation = beta
                        print(f"Exit location: {best_point}")
                        robot_move.send_goal(best_point, target_orientation)
                        finished=True

                        

                    #print(grad_value)

                    print(f'Angle {i}\n',25*'-')
                    print(f"Radiant: {np.around(pos_angles[0][i],decimals=2)}rad,\nDegree: {np.around(pos_angles[0][i]*np.pi/180,decimals=2)} deg,\nTotal: {np.around(beta,decimals=2)}rad,\nDistance: {np.around(pos_angles[1][i],decimals=2)}")
                    print(f'Pixel Conversion: {point_from_robot_X,point_from_robot_Y} -> {dx_pixel,dy_pixel}')
                    print(f"Gradient = {grad_value} @ {(x_grad,y_grad)}\n")


                if not finished:
                    
                    p_grad = np.array(p_grad)
                    p_grad[p_grad==0]=1
                    p_grad = (1/p_grad)
                    p_grad = p_grad/np.sum(p_grad)

                    print(p_grad.shape,pos_angles.shape)


                    idx_choice = np.random.choice(pos_indecies,p=p_grad)
                    print(f'Selected direction:\n Index {idx_choice}\n Angle: {pos_angles[0,idx_choice]},\n Distance: {pos_angles[1,idx_choice]}')
                
                        
                    # Calculate de X and Y values of the possible point to go with respect to the Robot location
                    beta =pos_angles[0,idx_choice]*np.pi/180+angle

                    point_from_robot_X = pos_angles[1,idx_choice] * math.cos(beta)
                    point_from_robot_Y = pos_angles[1,idx_choice] * math.sin(beta)
                    
                    # Get a pixel value for the Possible point with respect to the Map origin
                    #x_pixel = int((robot_global_pos[0] + point_from_robot_X)/0.05)
                    #y_pixel = int((robot_global_pos[1] + point_from_robot_Y)/0.05)

                    x_target = point_from_robot_X + robot_global_pos[0]
                    y_target = point_from_robot_Y + robot_global_pos[1]

                        
                    best_point = [x_target, y_target]
                    target_orientation = beta

                    
                    print(f"Robot current position: {robot_global_pos}")
                    print(f'Robot map position: {robot_map_pos}')
                    print(f"Target position{best_point}")

                    robot_move.send_goal(best_point, target_orientation)
                
        draw_map()
    except KeyboardInterrupt:
        print("Shutting down")

