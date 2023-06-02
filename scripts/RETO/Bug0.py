#!/usr/bin/env python  
import rospy  
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math


class Bug0():  
    '''
    Un super twisting con los colagenos o que?
    Que sepas que me encanto
    '''
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
        ###******* INIT PUBLISHERS *******###  
        rospy.Subscriber("base_scan", LaserScan, self.get_lidar_cb)
        rospy.Subscriber('/odom', Odometry, self.get_odom)
        rospy.Subscriber('GOAL', Point, self.get_goal)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.gtg_topic = rospy.Publisher('gtg_topic', Bool, queue_size=1)
        self.fw_topic = rospy.Publisher('fw_topic', Bool, queue_size=1)  
        ###******* INIT CONSTANTS/VARIABLES *******###  
        # Posicion del Robot
        self.robot_pos = Point()
        self.robot_theta = 0.0

        # Definicion de punto inicial y goal
        self.initial_pos = Point()
    
        self.target = Point()
    
        # Bandera para indicar que el lidar recibio datos 
        self.region_active = False 

        self.lidar_data = LaserScan()

        self.gtg_active = False

        self.fw_active = False
        
        self.goal_recieved = False

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
        goal_tolance = 0.1


        rate = rospy.Rate(10) # The rate of the while loop will be 50Hz 
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

            if self.goal_recieved is False:
                rospy.loginfo("Waiting for Goal")
                rate.sleep()
                continue

            if self.gtg_active is False and self.fw_active is False:
                self.current_state = "GTG"
                self.gtg_topic.publish(True)
                self.fw_topic.publish(False)
                self.gtg_active = True
                self.fw_active = False

            # Closest object 
            closest_dist, closest_angle = self.get_closest_object(self.lidar_data)

            # Avoid obstacle theta
            thetaAO = self.get_theta_ao(closest_angle) 

            # Go To Goal obstacle theta 
            thetaGTG = self.get_theta_gtg(self.robot_theta, self.target, self.robot_pos)
            
            # Distance to Goal
            self.d_t = self.get_distance_to_goal(self.target,self.robot_pos)

            rospy.logerr(self.current_state)

            if self.d_t < goal_tolance:
                rospy.loginfo("AT GOAL")
                self.change_state("STOP")

            elif self.current_state == "GTG":
                print("Going to Goal")
                if closest_dist < 0.35:
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

    def get_lidar_cb(self, msg=LaserScan()):
        self.lidar_data = msg
        self.region_active = True
    
    def get_closest_object(self,lidar_data = LaserScan()):
      # Para delimitar region de al frente del robot
      aux = lidar_data.ranges[360:788]

      Front = min(min(aux), 10)

      # Para obtener closest angle
      min_idx = 503 + np.argmin(aux)
      closest_angle = lidar_data.angle_min + min_idx * lidar_data.angle_increment 
      # limitar el angulo
      closest_angle = self.limit_angle(closest_angle) 
      return Front, closest_angle

    def change_state(self,state):
        self.current_state = state
        if self.current_state == "STOP":
            self.gtg_topic.publish(False)
            self.fw_topic.publish(False)
            self.gtg_active = False
            self.fw_active =  False
            self.done()
            self.goal_recieved =  False
            self.initial_pos = self.robot_pos

        elif self.current_state == "GTG":
            self.gtg_topic.publish(True)
            self.fw_topic.publish(False)
            self.gtg_active = True
            self.fw_active =  False
            rospy.sleep(2)

        elif self.current_state == "FW":
            self.D_Fw= self.d_t
            self.gtg_topic.publish(False)
            self.fw_topic.publish(True)
            self.gtg_active = False
            self.fw_active =  True
            #rospy.sleep(2)

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

    def get_theta_gtg(self, theta_robot, target_pos = Point(), robot_pos = Point()): 
        #This function returns the angle to the goal 
        x_target, y_target = target_pos.x, target_pos.y
        x_robot, y_robot = robot_pos.x, robot_pos.y

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
        if(np.abs(self.limit_angle(thetaAO-thetaGTG)) < np.pi/2.0):
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

    def get_goal(self, msg=Point()):
        self.target = msg
        self.goal_recieved = True

    def get_distance_to_goal(self, target = Point(), robot_pos = Point()):
        delta_x = target.x - robot_pos.x
        delta_y = target.y - robot_pos.y
        distance = np.sqrt((delta_x) ** 2 + (delta_y) ** 2)
        return distance
  
    def done(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(vel_msg)

    def cleanup(self):  
      self.gtg_topic.publish(False)
      self.fw_topic.publish(False)
      self.gtg_active = False
      self.fw_active = False
      self.done()
      '''This function is called just before finishing the node.'''
      print("Finish Message!!!")  
############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('Bug0') # Node Name
    try: Bug0()  # Class Name
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")
