#!/usr/bin/env python  
import rospy  
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math


class Bug0():  
    '''
    Que sepas que me encanto
    '''
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
        ###******* INIT PUBLISHERS *******###  
        rospy.Subscriber("base_scan", LaserScan, self.get_lidar_cb)
        rospy.Subscriber('/odom', Odometry, self.get_odom)
        self.gtg_topic = rospy.Publisher('gtg_topic', Bool, queue_size=1)
        self.fw_topic = rospy.Publisher('fw_topic', Bool, queue_size=1)  
        ###******* INIT CONSTANTS/VARIABLES *******###  
        # Posicion del Robot
        self.robot_pos = Point()
        self.robot_theta = 0.0

        # Definicion de punto inicial y goal
        self.initial_pos = Point()
        self.initial_pos.x = 0.0
        self.initial_pos.y = 0.0
        self.target = Point()
        self.target.x = 5.0
        self.target.y = 0.0

        # Definicion de estados
        self.current_state = "GTG"

        states = {
        "GTG" : "GO_TO_GOAL",
        "FW" : "WALL_FOLLOWER",
        "S" : "STOP"
        }

        progress = 0.1 # Para verificar si el robot avanzo esta distancia antes de cambiar de estado
        self.d_t = 0.0
        self.D_Fw = 0.0
    
        # Bandera para indicar que el lidar recibio datos 
        self.region_active = False 

        rate = rospy.Rate(50) # The rate of the while loop will be 50Hz 
        rospy.loginfo("Starting Message!")     
        # Grab possible simulation Error
        # while rospy.get_time() == 0: 
        #   print("no simulated time has been received yet") 
        # start_time = rospy.get_time()  #Get the current time in float seconds 
        ###******* PROGRAM BODY *******###  
        while not rospy.is_shutdown(): 
            # * Whole Body of the Program
            if self.region_active is False:
                rate.sleep()
                continue

            # Avoid obstacle theta
            thetaAO = self.get_theta_ao(self.closest_angle) 

            # Go To Goal obstacle theta 
            thetaGTG = self.get_theta_gtg(self.target.x, self.target.y, self.robot_pos.x, self.robot_pos.y, self.robot_theta)
            
            # Distance to Goal
            self.d_t = np.sqrt((self.target.x-self.robot_pos.x)**2+(self.target.y-self.robot_pos.y)**2) 
            
            if self.current_state == "GTG":
                print("Going to Goal")
                print(self.Front > 0.50)
                print(self.Front < 1.0)
                if self.Front > 0.30 and self.Front < 1.0:
                    self.change_state("FW")
                    print("Changed to Follow Walls")

            elif self.current_state == "FW":
                theta_clear_shot = self.clear_shot(thetaAO, thetaGTG)
                print("Distancia: ", self.D_Fw)
                print(self.d_t < (self.D_Fw - progress))
                print(theta_clear_shot)
                print(" ")

                if self.d_t < (self.D_Fw - progress) and theta_clear_shot:
                    self.change_state("GTG")

            rate.sleep()

    def limit_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))    
    
    def get_lidar_cb(self,msg):
      # Para delimitar region de al frente del robot
      self.region_active = True
      aux = msg.ranges
      self.Front = min(aux[503:645])

      # Para obtener closest angle
      min_idx = np.argmin(aux)
      closest_angle = msg.angle_min + min_idx * msg.angle_increment 
      # limitar el angulo
      self.closest_angle = self.limit_angle(closest_angle) 

    def change_state(self,state):
        self.current_state = state
        if self.current_state == "GTG":
            self.gtg_topic.publish(True)
            self.fw_topic.publish(False)
            #rospy.sleep(2)
        elif self.current_state == "FW":
            self.D_Fw = self.d_t
            self.gtg_topic.publish(False)
            self.fw_topic.publish(True)

    #?# ********** COMPORTAMIENTOS #?#**********
    def get_theta_ao(self, theta_closest): 
        ##This function returns the angle for the Avoid obstacle behavior  
        # theta_closest is the angle to the closest object [rad] 
        #This functions returns the angle for the Avoid obstacle behavior [rad] 
        ############################################################ 
        thetaAO=theta_closest-np.pi 
        #limit the angle to [-pi,pi] 
        thetaAO = self.limit_angle(thetaAO)
        return thetaAO

    def get_theta_gtg(self, x_target, y_target, x_robot, y_robot, theta_robot): 
        #This function returns the angle to the goal 
        theta_target=np.arctan2(y_target - y_robot, x_target - x_robot) 
        e_theta= theta_target - theta_robot 
        #limit e_theta from -pi to pi 
        #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi 
        e_theta = self.limit_angle(e_theta)
        return e_theta
    
    def get_theta_fw(self, thetaAO, clockwise): 
        ## This function computes the linear and angular speeds for the robot 
        # It receives thetaAO [rad] and clockwise [bool]
        if clockwise is True:
            thetaFWC = -np.pi/2.0 + thetaAO
        else:
            thetaFWC = np.pi/2.0 +  thetaAO
        thetaFWC = np.arctan2(np.sin(thetaFWC), np.cos(thetaFWC))

        return thetaFWC
    
    def clear_shot(self, thetaAO, thetaGTG):
        if(abs(self.limit_angle(thetaAO-thetaGTG)<np.pi/2.0)):
            return True
        else:
            return False


    def get_odom(self, msg=Odometry()):
      # position
      self.robot_pos = msg.pose.pose.position

      # Angulo
      quaternion = (
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w
      )
      euler = euler_from_quaternion(quaternion)
      self.robot_theta = euler[2]

    def cleanup(self):  
      '''This function is called just before finishing the node.'''
      print("Finish Message!!!")  
############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('Bug0') # Node Name
    try: Bug0()  # Class Name
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")
