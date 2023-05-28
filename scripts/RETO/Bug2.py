#!/usr/bin/env python  
import rospy  
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class Bug2():  
  '''
    Tonteria de Template
  '''
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    ###******* INIT PUBLISHERS *******###  
    #? Exmaple: self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 
    # rospy.Subscriber('/nodoDeLidar', ???, self.regions_cb)
    rospy.Subscriber('/odom', Odometry, self.get_odom)

    ###******* INIT SERVICES *******### 
    # service= rospy.Service('nombreeeee', objeto, self.callback)
    # TODO Agregar servicios de GoToGoalSwitch y WallFollowerSwitch

    ###******* INIT CONSTANTS/VARIABLES *******###  
    # Posicion del Robot
    self.robot_pos = Point()
    self.robot_theta = 0.0

    # Definicion de estados
    self.current_state = "GTG"
    states = {
       "GTG" : "GO_TO_GOAL",
       "WF" : "WALL_FOLLOWER"
    }

    # Definicion de regiones del lidar
    self.region_recive = False
    self.regions = None

    # Tiempo que se dura en un estado
    self.time = 0
    loop = 0

    rate = rospy.Rate(50) # The rate of the while loop will be 50Hz 
    rospy.loginfo("Starting Message!")     
    
    ###******* PROGRAM BODY *******###  
    while not rospy.is_shutdown(): 
      # Espera a que reciba regiones del lidar
      if self.region_recive is False:
         continue
      
      # Calcula la distancia del robot a la linea
      distance_line = self.getDistanceLine()

      if self.current_state == "GTG":
         if self.regions["F"] > 0.15 and self.regions["F"] < 1:
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

     # TODO aqui se cambia el estado en los servicios
        
  def getDistanceLine(self,actual_pos):
     pass
     
  
  def regions_cb(self):
     pass # TODO llenar esta funcion para que reciva de nodo de divide_line
  
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