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

    self.angle_precision = (np.pi/180.0) * 20.0 # angle error 45 degrees
    self.distance_precision = 0.01 # goal tolerance

    rate = rospy.Rate(10) # The rate of the while loop will be 10 Hz
    rospy.loginfo("STARTING GO TO GOAL NODE")     
    ###******* PROGRAM BODY *******###  
    while not rospy.is_shutdown(): 
      # if the node is not active, do nothing
      if self.active is False: 
        rospy.logfatal("NODE OFF")
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
        # rospy.logwarn("NO ODOM")
        rate.sleep()
        continue

      # Calculate thetaGTG
      y_target = self.target.y
      x_target = self.target.x
      robot_theta = self.robot_pos.z
      thetaGTG = np.arctan2(y_target-self.robot_pos.y,x_target-self.robot_pos.x)
      error_theta = self.limit_angle(thetaGTG - robot_theta)
      error_dist = np.sqrt(pow(y_target-self.robot_pos.y,2)+pow(x_target-self.robot_pos.x,2))
      # rospy.loginfo(error_theta)
      rospy.loginfo("REACHING GOAL: " + str(round(error_dist,2)))

      vel_msg = Twist()

      kvmax = 1.0  #linear speed maximum gain  
      kwmax = 1.0  #angular angular speed maximum gain 
      av = 5.0 #Constant to adjust the exponential's growth rate   
      aw = 5.0 #Constant to adjust the exponential's growth rate 

      #Compute the robot's angular speed 
      kw = kwmax*(1 - np.exp(-aw * error_theta**2))/abs(error_theta) if error_theta != 0.0 else 0.0 #Constant to change the speed  
      kv=kvmax*(1-np.exp(-av*error_dist**2))/abs(error_dist) if error_dist != 0.0 else 0.0 #Constant to change the speed  
      w = kw*error_theta 
      if error_dist <= self.distance_precision:
        w = 0.0
        v = 0.0
      elif np.abs(error_theta) > self.angle_precision:
        v = 0.0
      else:
        v=kv*error_dist #linear speed  
      w = self.limit_vel(w,0.2)
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