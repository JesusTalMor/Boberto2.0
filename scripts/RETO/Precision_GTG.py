#!/usr/bin/env python  
import rospy  
import numpy as np
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool

class GoToGoal():  
  """ Clase para implementar un GO TO GOAL
  """
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    ###******* INIT PUBLISHERS *******###  
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 
    rospy.Subscriber('position', Point, self.get_odom) # Comes From KalmanFilter
    rospy.Subscriber('gtg_topic', Bool, self.gtg_Switch)
    rospy.Subscriber('GOAL',Point, self.get_goal)
  
    ###******* INIT CONSTANTS/VARIABLES *******###  
    # Posicion del Robot
    self.robot_pos = Point()
    self.odom_received = False

    self.active = False
    self.current_state = "FIX"
    states = {
      "FIX" : "FIX_ANGLE",
      "GO" : "GO_STRAIGHT",
      "HERE" : "ON_GOAL",
      "STOP" : "STOP"
    }

    # Define goal point
    self.target = Point()
    self.goal_received = True

    self.inital_angle_precision = (np.pi/180.0) * 2.0 # Fix estate in 2 degrees    
    self.angle_precision = (np.pi/180.0) * 45.0 # angle error 45 degrees
    self.distance_precision = 0.05 # goal tolerance

    rate = rospy.Rate(10) # The rate of the while loop will be 10 Hz
    rospy.loginfo("STARTING GO TO GOAL NODE")     
    ###******* PROGRAM BODY *******###  
    while not rospy.is_shutdown(): 
      # if the node is not active, do nothing
      if self.active is False: 
        rospy.logfatal("NODE OFF")
        self.current_state = "FIX"
        # self.stop_robot()
        rate.sleep() 
        continue

      if self.goal_received is False: 
        rospy.logwarn("WAIT GOAL")
        self.current_state = "FIX"
        self.stop_robot()
        rate.sleep() 
        continue

      if self.odom_received is False:
        rospy.logwarn("NO ODOM")
        rate.sleep()
        continue


      if self.current_state == "FIX":
        self.fix_angle(self.target, self.robot_pos)
      
      elif self.current_state == "GO":
        self.go_straight(self.target, self.robot_pos)
      
      elif self.current_state == "HERE":
        rospy.loginfo("GOAL REACHED - CHANGE TO WAIT GOAL")
        self.stop_robot()
        self.goal_received = False
      
      elif self.current_state == "STOP":
        self.stop_robot()

      self.odom_received = False
      rate.sleep() 

  def fix_angle(self, target=Point(), robot_pos=Point()):
    # Calculate thetaGTG
    x_target = target.x
    y_target = target.y
    robot_theta = robot_pos.z
    thetaGTG = np.arctan2(y_target-robot_pos.y,x_target-robot_pos.x)
    error_theta = self.limit_angle(thetaGTG - robot_theta)
    error_dist = np.sqrt(pow(y_target-robot_pos.y,2)+pow(x_target-robot_pos.x,2))
    rospy.loginfo("TURN TO GOAL: " + str(round(error_theta,2)))

    # * To change state...
    if error_dist <= self.distance_precision:
      rospy.logwarn("GOAL REACHED - CHANGE TO DONE")
      self.current_state = "HERE"
      return
    elif np.abs(error_theta) <= self.inital_angle_precision:
      rospy.logwarn("ANGLE ADJUSTED - CHANGE TO GO")
      self.current_state = "GO"
      return

    vel_msg = Twist()
    kwmax = 5.0  #angular angular speed maximum gain 
    aw = 5.0 #Constant to adjust the exponential's growth rate 
    kw = kwmax*(1 - np.exp(-aw * error_theta**2))/abs(error_theta) if error_theta != 0.0 else 0.0 #Constant to change the speed  
    w = kw * 0.4 if error_theta > 0.0 else kw * -0.4
    w = self.limit_vel(w, 0.4)
    
    vel_msg.angular.z = w
    vel_msg.linear.x = 0.0

    self.cmd_vel_pub.publish(vel_msg)


  def go_straight(self, target=Point(), robot_pos=Point()):
    # Calculate thetaGTG
    y_target = target.y
    x_target = target.x
    robot_theta = robot_pos.z
    thetaGTG = np.arctan2(y_target-robot_pos.y,x_target-robot_pos.x)
    error_theta = self.limit_angle(thetaGTG - robot_theta)
    error_dist = np.sqrt(pow(y_target-robot_pos.y,2)+pow(x_target-robot_pos.x,2))
    # rospy.loginfo(error_theta)
    rospy.loginfo("REACHING GOAL: " + str(round(error_dist,2)))

    # * To change state...
    if error_dist <= self.distance_precision:
      rospy.logwarn("GOAL REACHED - CHANGE TO DONE")
      self.current_state = "HERE"
    
    elif np.abs(error_theta) > self.angle_precision:
      rospy.logwarn("ANGLE ERROR HUGE - CHANGE TO FIX")
      self.current_state = "FIX"

    vel_msg = Twist()

    kvmax = 1.0  #linear speed maximum gain  
    kwmax = 2.0  #angular angular speed maximum gain 
    av = 5.0 #Constant to adjust the exponential's growth rate   
    aw = 5.0 #Constant to adjust the exponential's growth rate 

    #Compute the robot's angular speed 
    kw = kwmax*(1 - np.exp(-aw * error_theta**2))/abs(error_theta) if error_theta != 0.0 else 0.0 #Constant to change the speed  
    w = kw*error_theta 
    w = self.limit_vel(w,0.4)
    kv=kvmax*(1-np.exp(-av*error_dist**2))/abs(error_dist) if error_dist != 0.0 else 0.0 #Constant to change the speed  
    v=kv*error_dist #linear speed  
    v = self.limit_vel(v, 0.2)
    vel_msg.angular.z = w
    vel_msg.linear.x = v

    self.cmd_vel_pub.publish(vel_msg)


  def stop_robot(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    self.cmd_vel_pub.publish(vel_msg)

  def limit_angle(self,angle):
    return np.arctan2(np.sin(angle),np.cos(angle))
  
  def limit_vel(self, vel, lim):
    sign = 1 if vel > 0.0 else -1
    vel = vel * sign if np.abs(vel) <= lim else lim * sign

    return vel

  def get_odom(self, msg=Point()):
    # position
    if self.odom_received is False:
      self.robot_pos = msg
    self.odom_received = True

  def gtg_Switch(self,msg=Bool()):
    self.active = msg.data

  def get_goal(self, msg=Point()):
    self.target = msg
    self.goal_received = True
    

  def cleanup(self):  
      '''This function is called just before finishing the node.'''
      print("FINISH MESSAGE STOPPING ROBOT")  
      vel_msg = Twist()
      self.cmd_vel_pub.publish(vel_msg)

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('move_forward_some_time') # Node Name
    try: GoToGoal()  # Class Name
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")