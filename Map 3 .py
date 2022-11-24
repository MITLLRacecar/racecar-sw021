#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import time
import math

from database import Database
from ackermann_msgs.msg import AckermannDrive
from parameter_list import Param

param = Param()

class Brain():
    def __init__(self, db=Database, map_number=int):
        self.db = db
        self.map_number = map_number
        self.parking_x_1 = param.parking_x_1
        self.parking_y_1 = param.parking_y_1
        self.parking_yaw_1 = param.parking_yaw_1
        self.parking_x_2 = param.parking_x_2
        self.parking_y_2 = param.parking_y_2
        self.parking_yaw_2 = param.parking_yaw_2
        self.first_park_success = False
        self.second_park_success = False
        self.cnt = 0
        self.alpha = 3
        self.beta = 2
        self.first_flag = False
        self.start_time = 0
        self.keep = []
        self.back_move = True
        self.back_move_real = False
        self.K = 0
    def isinParking(self, pose_data, parking_x_1, parking_y_1, parking_yaw_1):
        print("check: ", abs(pose_data[0] - parking_x_1), abs(pose_data[1] - parking_y_1), abs(pose_data[2] - parking_yaw_1))
        if abs(pose_data[0] - parking_x_1) < 0.12 and abs(pose_data[1] - parking_y_1) < 0.15 and abs(pose_data[2] - parking_yaw_1) < 10:
            return True
        
        else:
            return False
    
    
    def drive(self, way_point, lidar_data, pose_data, traffic_light, remaining_time):
        next_x = way_point[self.cnt][0]
        next_y = way_point[self.cnt][1]
        
        cur_x = pose_data[0]
        cur_y = pose_data[1]
        
        if math.sqrt((next_x - cur_x)**2 + (next_y - cur_y)**2) < 0.1:
            self.cnt += 1
        
        cur_yaw = pose_data[2]
        
        v1 = np.array([math.cos(cur_yaw*math.pi/180), math.sin(cur_yaw*math.pi/180), 0])

        v2 = np.array([next_x - cur_x, next_y - cur_y, 0])        
        
        
        v1_mag = np.sqrt(v1.dot(v1))
        v2_mag = np.sqrt(v2.dot(v2))
                
        print("------ Count ", self.cnt, "---------")
        
        if v2_mag == 0:
            v2_mag = 0.001        
        
        cos_data =   np.dot(v1, v2) / (v1_mag * v2_mag)
        if cos_data > 1.0:
            cos_data = 0.9999
        yaw_result = abs(math.acos(cos_data)*180/math.pi)
        cross_result = np.cross(v1, v2)
    
        if (cross_result[2] < 0):
            yaw_result = -yaw_result
        
        print("Yaw result", yaw_result)
        
        speed = 1.2
        
        if yaw_result < 0:
            converted_yaw = 360 + yaw_result
            for i in range(int(round(converted_yaw - self.alpha)%360), int(round(converted_yaw + self.alpha)%360)):
                print("index check : ", i)
                if lidar_data[i] < param.WIDTH:
                    yaw_result += self.beta
                    speed = 0.3
        
        elif yaw_result > 0:
            for i in range(int(round(yaw_result - self.alpha)%360), int(round(yaw_result + self.alpha)%360)):
                if lidar_data[i] < param.WIDTH:
                    yaw_result -= self.beta
                    speed = 0.3
        
        if yaw_result > 20:
            yaw_result = 20
        
        elif yaw_result < -20:
            yaw_result = -20
        
        
        print("Speed : ", speed)
        # Determine the angle & speed
        # angle : 20=LEFT ~ -20=RIGHT (degree)
        # speed : 0 ~ 4 (m/s)
        angle = yaw_result
        
        # Return angle & speed
        return angle, speed
    
    def main(self):
                   
        lidar_data = self.db.lidar_data
        pose_data = self.db.pose_data
        traffic_light = self.db.traffic_light
        remaining_time = self.db.traffic_remaining_time


        # Print for debugging
        # print(traffic_light, remaining_time)
        
        if self.map_number == 1:
            way_point = [(0.17, 0.00), (2.38,0.17), (5.33, 0.11), (6.9, -1.25), 
                            (10.49, -4.57), (7.56, -5.93), (5.97, -6.03), (1.37, -5.78), (0.42, -7.91),
                            (-0.78, -10.02), (0.31, -12.51), (-1.82, -13.88), (-4.53, -13.68),
                            (-5.53, -14.99), (-5.43, -20.51), (-4.46, -21.43), (0.88, -21.35), 
                            (2.33, -20.62), (3.76, -19.12), (8.65, -14.01), (13.54, -9.29)]
            
            angle, speed = self.drive(way_point, lidar_data, pose_data, traffic_light, remaining_time)
            print("ASDASDASDAS",self.cnt)
            
            # Return angle & speed
            speed = 0.7
            cnt_list = [3,5,8,9,10,11,13,15,17]

            if self.cnt in [19,20]:
                speed = 3.0
            if self.cnt not in cnt_list:
                speed = 1.8

            return angle, speed
        
        elif self.map_number == 2:
            way_point = [(4.19,0.14),(11.14,0.05),(20.35,0.20),
            (21.29,-0.80),(21.59,-2.35),(19.64,-3.33),(15.48,-3.31),
            (14.16,-4.28),(14.02,-5.95),(15.80, -6.90), (19.18,-7.18),
            (25.66,-6.81), (27.5, -7.76), (27.45,-9.89), (25.55,-10.53),(23.48,-10.53),(18.71,-10.51),
            (15.43,-10.51),(14.01, -11.76), (14.42, -13.44), (17.18, -13.85),
            (18.33, -15.19), (18.59, -16.53), (16.84, -17.54), (14.75, -17.72),
            (13.97, -19.47), (15.57, -21.08), (20.68, -21.08), (26.19, -20.96),
            (27.80, -22.30), (27.00, -24.53), (25.45, -24.83), (12.85, -24.64),
            (-0.91, -24.75), (-2.92, -23.64), (-4.00, -21.70), (-4.00, -17.34), (-4.00, -16.00)]
            
            way_point_left = [(-6.0, -15.80), (-8.23, -15.67), (-9.20, -14.60), (-9.18, -10), 
            (-8.05, -8.82), (-4.71, -8.67), (-3.97, -7.38), (-3.95, -6.00), (-3.92, -1.2)]
            way_point_right = [(-2.5, -15.80), (-0.2, -15.99), (1.22, -14.98), (1.35, -12.54),
            (1.52, -10.26), (0.48, -9.03), (-0.63, -8.57), (-3.21, -8.71), (-3.91, -4.38), (-3.92, -1.2)]
            
            print(traffic_light)
            
            if traffic_light != None:
                if traffic_light == 'STOP':
                    angle = 0
                    speed = 0
                    self.cnt = 0

                    return angle, speed
                
                if traffic_light == 'LEFT':
                    way_point = way_point_left
                    angle, speed = self.drive(way_point, lidar_data, pose_data, traffic_light, remaining_time)
                    return angle, speed

                
                if traffic_light == 'RIGHT':
                    way_point = way_point_right
                    angle, speed = self.drive(way_point, lidar_data, pose_data, traffic_light, remaining_time)
                    return angle, speed
                    
                    
            else:            
                angle, speed = self.drive(way_point, lidar_data, pose_data, traffic_light, remaining_time)
                
                return angle, speed

        elif self.map_number == 3:
            way_point = [(0.07, 0.02), (5.57, 0.08), (12.21, 0.37), 
                        (13.45, -1.43), (16.39, -7.51), (13.77, -8.61), 
                        (9.65, -8.40), (8.62, -10.13), (8.30, -11.85)]
            
            angle, speed = self.drive(way_point, lidar_data, pose_data, traffic_light, remaining_time)
            
            start_time = 0
            start_time1 = 0
            parking_x_1 = self.parking_x_1
            parking_y_1 = self.parking_y_1
            parking_yaw_1 = self.parking_yaw_1
            
            if traffic_light != None:
                if traffic_light == 'STOP':
                    angle = -90
                    speed = 0
                    self.cnt = 0
                    return angle, speed
                
                if traffic_light == 'GO' and self.first_park_success == False:
                    if parking_x_1 < 8.50:    
                        way_point_1_2 = [(8.73, -14.55), (6.80, -16.1), (2.77, -15.50), (1.05, -16.94)]
                        way_point_1_2.append((parking_x_1, parking_y_1 + 1.5))
                        way_point_1_2.append((parking_x_1, parking_y_1 - 0.5))
                        
                        way_point = way_point_1_2
                        
                        angle, speed = self.drive(way_point, lidar_data, pose_data, traffic_light, remaining_time)
                        boolean = 0
                        if self.isinParking(pose_data, parking_x_1, parking_y_1, parking_yaw_1) == True and boolean ==0:
                            angle = 0
                            speed = 0
                            if self.first_flag == False:
                                start_time = time.time()
                                self.start_time = start_time
                                self.first_flag = True
                                self.back_move = False
                            
                        
                            if time.time() - self.start_time > 3:
                                self.first_park_success = True
                                boolean += 1       
                    # if not self.first_flag:
                        else:
                            self.keep.append((angle,- speed))
                    return angle, speed
                    # else: return angle, - speed

                if self.back_move==False:
                    start = time.time()
                    self.back_move=True
                    self.start_time = start
                    self.back_move_real = True

                if self.back_move_real and time.time() - self.start_time < 13:
                    return self.keep.pop()
                else:
                    self.back_move_real=False


                # if (((pose_data[0], pose_data[1]) == (param.MAP_3_SPAWN_POINT[2][0], param.MAP_3_SPAWN_POINT[2][1])) or ((pose_data[0], pose_data[1]) == (param.MAP_3_SPAWN_POINT[3][0], param.MAP_3_SPAWN_POINT[3][1]))) and self.first_park_success == True:
                if self.first_park_success == True and self.second_park_success == False:
                    if self.K == 0:
                        self.cnt=0
                    # angle = param.MAP_3_SPAWN_POINT[2][2]
                    way_point_middle = [(7.8, -14.5), (9.6, -16.1), (24.80, -15.47), (26.07, -17.22), (26.47, -19.21), (24.57, -20.88),(15.71, -26.34),
                                        (3.70,-32.80), (2.53, -33.35), (2.57, -35.32), (5.51, -37.38), (25.07, -48.16), (25.91, -50.32)]
                    self.K += 1
                    way_point = way_point_middle                           
                    angle, speed = self.drive(way_point, lidar_data, pose_data, traffic_light, remaining_time)
                    return angle, speed                    



                if traffic_light == 'GO' and self.first_park_success == True:
                    way_point_end = [(26.59, -52.40), (24.47, -53.45), (20.11, -53.69), (14.82, -54.05), 
                                    (12.50, -52.44), (12.65, -50.75), (12.54, -49.28), (26.59, -52.40), 
                                    (24.47, -53.45), (20.11, -53.69), (7.81, -53.43), (5.59, -56.11),
                                    (3.13, -61.2), (5.01, -62.7), (15.0, -62.4), (21.7, -62.5), (26.21, -62.39)]
                    way_point = way_point_end
                    angle, speed = self.drive(way_point, lidar_data, pose_data, traffic_light, remaining_time)
                    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    return angle, speed
            else:
                
                if (((pose_data[0], pose_data[1]) == (param.MAP_3_SPAWN_POINT[2][0], param.MAP_3_SPAWN_POINT[2][1])) or ((pose_data[0], pose_data[1]) == (param.MAP_3_SPAWN_POINT[3][0], param.MAP_3_SPAWN_POINT[3][1]))) and self.first_park_success == True:
                    
                    angle = param.MAP_3_SPAWN_POINT[2][2]
                    way_point_middle = [(24.80, -15.47), (26.07, -17.22), (26.47, -19.21), (24.57, -20.88),(15.71, -26.34),
                                        (3.70,-32.80), (2.53, -33.35), (2.57, -35.32), (5.51, -37.38), (25.07, -48.16), (25.91, -50.32)]
                    
                    way_point = way_point_middle
                    
                    self.cnt = 0
                
                angle, speed = self.drive(way_point, lidar_data, pose_data, traffic_light, remaining_time)
                return angle, speed

if __name__ == "__main__":
    db = Database(lidar=True)
    test_brain = Brain(db)
    rate = rospy.Rate(param.thread_rate)
    control_pub = rospy.Publisher('drive', AckermannDrive, queue_size=1)
    while not rospy.is_shutdown():
        car_angle, car_speed = test_brain.main()
        motor_msg = AckermannDrive()
        motor_msg.steering_angle = car_angle
        motor_msg.speed = car_speed
        motor_msg.steering_angle_velocity = param.car_angular_velocity
        motor_msg.acceleration = param.car_acceleration
        motor_msg.jerk = param.car_jerk
        rate.sleep()