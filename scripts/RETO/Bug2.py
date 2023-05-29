#!/usr/bin/env python  
import rospy  
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
import math

class Bug2():  
  '''
    Tonteria de Template
  '''
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    ###******* INIT PUBLISHERS *******###  
    rospy.Subscriber("base_scan", LaserScan, self.get_lidar_cb)
    rospy.Subscriber('/odom', Odometry, self.get_odom)
    self.gtg_topic = rospy.Publisher('gtg_topic', bool, queue_size=1)
    self.fw_topic = rospy.Publisher('fw_topic', bool, queue_size=1)    

    ###******* INIT CONSTANTS/VARIABLES *******###  
    # Posicion del Robot
    self.robot_pos = Point()
    self.robot_theta = 0.0

    # Definicion de punto inicial y goal
    self.initial_pos = Point()
    self.initial_pos.x = 0.0
    self.initial_pos.y = 0.0
    self.target = Point()
    self.target.x = 4.0
    self.target.y = 0.0

    # Definicion de estados
    self.current_state = "GTG"
    states = {
       "GTG" : "GO_TO_GOAL",
       "WF" : "WALL_FOLLOWER"
    }

    # Bandera para indicar cuando tenga datos del las regiones
    self.region_recive = False

    # Tiempo que se dura en un estado
    self.time = 0
    loop = 0

    self.change_state("GTG")

    rate = rospy.Rate(50) # The rate of the while loop will be 50Hz 
    rospy.loginfo("Starting Message!")     
    
    ###******* PROGRAM BODY *******###  
    while not rospy.is_shutdown(): 
      # Espera a que reciba regiones del lidar
      if self.region_recive is False:
         rate.sleep()
         continue
      
      # Calcula la distancia del robot a la linea
      distance_line = self.getDistanceLine()


      if self.current_state == "GTG":
         if self.Front > 0.15 and self.Front < 1:
            self.change_state("WF")
      elif self.change_state == "WF":
         if self.time == 5.0 and distance_line < 0.1:
            self.change_state("GTG")
      
      loop = loop + 1
      if loop == 20:
         self.time = self.time + 1
         loop = 0
               
            
      rate.sleep() 

  def change_state(self,state):
     self.time = 0
     self.current_state = state
     
     if self.current_state == "GTG":
        self.gtg_topic.publish(True)
        self.fw_topic.publish(False)
     if self.current_state == "FW":
        self.gtg_topic.publish(False)
        self.fw_topic.publish(True)
        
  def getDistanceLine(self,actual_pos):
     up = math.fabs((self.target.y - self.initial_pos.y) * actual_pos.x - (self.target.x - self.initial_pos.x) * actual_pos.y + (self.target.x * self.initial_pos.y) - (self.target.y * self.initial_pos.x))
     down = math.sqrt(pow(self.target.y - self.initial_pos.y, 2) + pow(self.target.x - self.initial_pos.x, 2))
     return up / down
  
  def get_lidar_cb(self,msg):
     self.region_recive = True
     aux = msg.ranges
     self.Front = min(aux[503:645])
  
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
    rospy.init_node('Bug2_node') # Node Name
    try: Bug2()
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")