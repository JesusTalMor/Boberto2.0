#!/usr/bin/env python  
import rospy  
import numpy as np
from geometry_msgs.msg import Twist

class GoToGoal():  
  '''
    Tonteria de Template
  '''
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    ###******* INIT PUBLISHERS *******###  
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 
  
    ###******* INIT SERVICES *******### 
    # service= rospy.Service('nombreeeee', objeto, self.callback)

    ###******* INIT CONSTANTS/VARIABLES *******###  
    # Robot pos
    self.robot_x = 0.0
    self.robot_y = 0.0
    self.robot_theta = 0.0

    self.active = False 
    self.current_state = "STOP"

    # Define goal point
    self.x_target = 0.0
    self.y_target = 0.0

    # goal tolerance +/- error 2°
    self.angle_precision = np.pi/90.0

    # goal tolerance
    self.distance_precision = 0.3

    rate = rospy.Rate(20) # The rate of the while loop will be 50Hz 
    rospy.loginfo("Starting Message!")     
    # Grab possible simulation Error
    # while rospy.get_time() == 0: 
    #   print("no simulated time has been received yet") 
    # start_time = rospy.get_time()  #Get the current time in float seconds 
    ###******* PROGRAM BODY *******###  
    while not rospy.is_shutdown(): 
      # if the node is not active, do nothing
      if self.active is False: 
        rate.sleep() 
        continue

      if self.current_state == "FIX":
        #  TODO IMPLEMENTAR FUNCION DE GIRO
        pass
      elif self.current_state == "GO":
        #  TODO IMPLEMENTAR FUNCION DE IR RECTO
        pass
      elif self.current_state == "HERE":
        # TODO IMPLEMENTAR FUNCION DE LLEGAR AL GOAL
        pass
      elif self.current_state == "STOP":
        # TODO IMPLEMENTAR STOP
        pass

      rate.sleep() 

  def fix_angle(self,x_target,y_target):
    # Calculate thetaGTG
    thetaGTG = np.arctan2(y_target-self.robot_y,x_target-self.robot_x)
    error_theta = self.limit_angle(thetaGTG - self.robot_theta)
    rospy.loginfo(error_theta)

    vel_msg = Twist()

    if np.abs(error_theta) > self.angle_precision:
      vel_msg.angular.z = 0.2 if error_theta > 0 else -0.2

    self.cmd_vel_pub.publish(vel_msg)

    # * To change state...
    if np.abs(error_theta) <= self.angle_precision:
      rospy.loginfo("Error theta: ", round(error_theta,2))
      self.current_state = "GO"

  def go_straight(self,x_target,y_target):
    # Calculate thetaGTG
    thetaGTG = np.arctan2(y_target-self.robot_y,x_target-self.robot_x)
    error_theta = self.limit_angle(thetaGTG - self.robot_theta)
    error_dist = np.sqrt(pow(y_target-self.robot_y,2)+pow(x_target-self.robot_x,2))
    # rospy.loginfo(error_theta)

    vel_msg = Twist()

    if error_dist > self.distance_precision:
      vel_msg.linear.x = 0.4
      vel_msg.angular.z = 0.1 if error_theta > 0 else -0.1

    self.cmd_vel_pub.publish(vel_msg)

    # * To change state...
    if error_dist <= self.distance_precision:
      rospy.loginfo("Error distance: ", round(error_dist,2))
      self.current_state = "HERE"
    
    if np.abs(error_theta) > self.angle_precision:
      rospy.loginfo("Error theta: ", round(error_theta,2))
      self.current_state = "FIX"

  def done(self):
    vel_msg = Twist()
    self.cmd_vel_pub.publish(vel_msg)

  def limit_angle(self,angle):
    return np.arctan2(np.sin(angle),np.cos(angle))

    
  def cleanup(self):  
      '''This function is called just before finishing the node.'''
      print("Finish Message!!!")  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('move_forward_some_time') # Node Name
    try: TemplateClass()  # Class Name
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")