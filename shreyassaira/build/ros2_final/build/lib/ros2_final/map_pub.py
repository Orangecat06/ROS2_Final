import sys
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from irobot_create_msgs.msg import IrIntensityVector
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

import time
import numpy as np
import matplotlib.pyplot as plt
import math

namespace = 'robot_0'

ir_threshold = 36
tolerance = 0.2
tolerance1 = 0.05
tolerance2 = 0.02
starting_state = 0
finished_mapping = 0
goingtox = 0
turningright = 0
dancing = 0
goingtoy = 0
centerdone = 0
xmax = 0
ymax = 0
xmin = 0
ymin = 0

class Mapping_Midterms(Node):
    def __init__(self):
        super().__init__('mapping_midterm')
        print('IR Sensing (subscriber), Twist (publisher)')
        self.subscription = self.create_subscription(IrIntensityVector, namespace + '/ir_intensity', self.ir_sense, qos_profile_sensor_data)
        self.subscription = self.create_subscription(Odometry, namespace + '/odom', self.odom_sense, qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, namespace + '/cmd_vel', 10)
        
        self.ir_val = [0,0,0]
        
        self.position = [0,0,0,0]
        self.initial_position = [0,0,0,0]
        self.last_turn_position = [0,0,0,0]

        self.odom_status = None
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        
    def ir_sense(self, irmsg:IrIntensityVector):
        self.ir_val[0] = irmsg.readings[0].value
        self.ir_val[1] = irmsg.readings[3].value
        self.ir_val[2] = irmsg.readings[2].value

    def odom_sense(self, odommsg:Odometry):
        global starting_state
        global finished_mapping
        global xmax
        global ymax
        global xmin
        global ymin
        global goingtox 
        global turningright 
        global dancing 
        global goingtoy
        global centerdone 

        self.position[0] = odommsg.pose.pose.position._x
        self.position[1] = odommsg.pose.pose.position._y
        self.position[2] = odommsg.pose.pose.position._z

        plt.plot(self.position[0], self.position[1], 'r.')
        plt.show(block=False)
        plt.pause(0.001)

        Qx = odommsg.pose.pose.orientation._x
        Qy = odommsg.pose.pose.orientation._y
        Qz = odommsg.pose.pose.orientation._z
        Qw = odommsg.pose.pose.orientation._w
        siny_cosp = 2.0*(Qw*Qz + Qx*Qy)
        cosy_cosp = 1.0 - 2.0*(Qy*Qy + Qz*Qz) 
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        degyaw = math.degrees(yaw)
        self.position[3] = degyaw

        if starting_state == 0:
            self.initial_position[0] = self.position[0]
            self.initial_position[1] = self.position[1]
            self.initial_position[2] = self.position[2]
            self.initial_position[3] = self.position[3]
            starting_state = starting_state+1
            xmax = self.position[0]
            ymax = self.position[1]
            xmin = self.position[0]
            ymin = self.position[1]
        starting_state = starting_state+1

        if self.position[0] > xmax:
            xmax = self.position[0]
        if self.position[1] > ymax:
            ymax = self.position[1]

        if self.position[0] < xmin:
            xmin = self.position[0]
        if self.position[1] < ymin:
            ymin = self.position[1]

        if starting_state > 100:
            if ((self.position[3] > self.initial_position[3] - 3) and (self.position[3] < self.initial_position[3]+3)) and ((self.position[0] > (self.initial_position[0] - tolerance)) and (self.position[0] < (self.initial_position[0] + tolerance))) and ((self.position[1] > self.initial_position[1] - tolerance) and (self.position[1] < (self.initial_position[1] + tolerance))):
                finished_mapping = 1
                #print(self.initial_position)
                #print(self.position) 

        if finished_mapping == 0:
             self.move()

        if finished_mapping == 1:
            xmiddle = (xmin + xmax)/2
            ymiddle = (ymin + ymax)/2
            #print("finished mapping")
            #print('ymiddle', ymiddle)
            print('xmiddle', xmiddle, 'xpos', self.position[0])
            if ((self.position[0] < (xmiddle - tolerance2)) or (self.position[0] > (xmiddle + tolerance2))) and (goingtox == 0):
                print("ji")
                if self.ir_val[0] > 10 and self.ir_val[1] > 20: #turn right
                    self.twist.linear.x = 0.225  
                    self.twist.angular.z = -1.0
                    self.publisher.publish(self.twist)
                elif self.ir_val[0] < 40 and self.ir_val[1] < ir_threshold: #adjust left
                    self.twist.linear.x = 0.1  
                    self.twist.angular.z = 0.5
                    self.publisher.publish(self.twist)
                elif self.ir_val[0] > 40 and self.ir_val[1] < ir_threshold: #adjust right
                    self.twist.linear.x = 0.1  
                    self.twist.angular.z = -0.5
                    self.publisher.publish(self.twist)
                elif self.ir_val[0] > 40 and self.ir_val[1] < ir_threshold:#go straight
                    self.twist.linear.x = 0.15
                    self.twist.angular.z = 0.0
                    self.publisher.publish(self.twist)
                elif self.ir_val[0] < 30 and self.ir_val[1] < 30: #turn left
                    self.twist.linear.x = 0.225  
                    self.twist.angular.z = 1.0
                    self.publisher.publish(self.twist)
            elif ((xmiddle + tolerance2) > self.position[0]) and ((xmiddle - tolerance2) < self.position[0]):
                goingtox = goingtox + 1
            if (goingtox >= 1) and (turningright < 31):
                self.twist.linear.x = 0.0
                self.twist.angular.z = -1.0
                self.publisher.publish(self.twist)
                turningright = turningright + 1
                goingtoy = 1
            elif ((self.position[1] < (ymiddle - tolerance2)) or (self.position[1] > (ymiddle + tolerance2))) and (goingtoy == 1):
                print("hi")
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0.0
                self.publisher.publish(self.twist)
            elif ((ymiddle + tolerance2) > self.position[1]) and ((ymiddle - tolerance2) < self.position[1]):
                goingtoy = goingtoy + 1
                centerdone = 1
                print('going to y', goingtoy)
            elif (centerdone == 1):
                self.twist.angular.z = 1.0
                self.publisher.publish(self.twist)

            
            
            


                

         
    def move(self):
        if self.ir_val[0] > 10 and self.ir_val[1] > 20: #turn right
            self.twist.linear.x = 0.225  
            self.twist.angular.z = -1.0
            self.publisher.publish(self.twist)
        elif self.ir_val[0] < 40 and self.ir_val[1] < ir_threshold: #adjust left
            self.twist.linear.x = 0.1  
            self.twist.angular.z = 0.5
            self.publisher.publish(self.twist)
        elif self.ir_val[0] > 40 and self.ir_val[1] < ir_threshold: #adjust right
            self.twist.linear.x = 0.1  
            self.twist.angular.z = -0.5
            self.publisher.publish(self.twist)
        elif self.ir_val[0] > 40 and self.ir_val[1] < ir_threshold:#go straight
            self.twist.linear.x = 0.15
            self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)
        elif self.ir_val[0] < 30 and self.ir_val[1] < 30: #turn left
            self.twist.linear.x = 0.225  
            self.twist.angular.z = 1.0
            self.publisher.publish(self.twist)
            
    def dance(self):
        print('Dancing')
        radius = 0.1
        speed = 0.1
        self.twist.linear.x = speed
        self.twist.angular.z = speed/radius
        self.publisher.publish(self.twist)
        print("finished dance")
            

def main(args=None):
    rclpy.init(args=args)
    Ir_subscriber = Mapping_Midterms()
    print('Callbacks are called.')
    try:
        rclpy.spin(Ir_subscriber)
    except KeyboardInterrupt:
        print('\nCaught Keyboard interupt')
    finally:
        print("Done")
        Ir_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
        
