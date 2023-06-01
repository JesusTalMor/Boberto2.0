#!/usr/bin/env python  
import rospy  
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

class GoToGoal():  
  """ Clase para implementar un GO TO GOAL
  """
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    ###******* INIT PUBLISHERS *******###  
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 
    rospy.Subscriber('/odom', Odometry, self.get_odom) # Comes From KalmanFilter
    rospy.Subscriber('gtg_topic', Bool, self.gtg_Switch)
    rospy.Subscriber('GOAL',Point, self.get_goal)
  
    ###******* INIT CONSTANTS/VARIABLES *******###  
    # Posicion del Robot
    self.robot_pos = Point()
    self.robot_theta = 0.0

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

    self.initial_angle_precision = (np.pi/180.0) * 5.0  # goal tolerance +/- error 2
    self.angle_precision = (np.pi/180.0) * 45.0 # goal tolerance +/- error 2    
    self.distance_precision = 0.1 # goal tolerance

    rate = rospy.Rate(20) # The rate of the while loop will be 50Hz 
    rospy.loginfo("Starting Message!")     
    ###******* PROGRAM BODY *******###  
    while not rospy.is_shutdown(): 
      # if the node is not active, do nothing
      if self.active is False: 
        # self.goal_received = False
        self.current_state = "FIX"
        rate.sleep() 
        continue

      if self.goal_received is False: 
        rate.sleep() 
        continue

      if self.current_state == "FIX":
        self.fix_angle(self.robot_theta, self.target, self.robot_pos)
      elif self.current_state == "GO":
        self.go_straight(self.robot_theta, self.target, self.robot_pos)
      elif self.current_state == "HERE":
        self.done()
      elif self.current_state == "STOP":
        self.done()

      rate.sleep() 

  def fix_angle(self, robot_theta, target=Point(), robot_pos=Point()):
    # Calculate thetaGTG
    y_target = target.y
    x_target = target.x
    thetaGTG = np.arctan2(y_target-robot_pos.y,x_target-robot_pos.y)
    error_theta = self.limit_angle(thetaGTG - robot_theta)
    error_dist = np.sqrt(pow(y_target-robot_pos.y,2)+pow(x_target-robot_pos.x,2))
    # rospy.loginfo(error_theta)

    vel_msg = Twist()

    if np.abs(error_theta) > self.initial_angle_precision:
      # vel_msg.angular.z = 0.4 if error_theta > 0 else -0.4
      vel_msg.angular.z = 0.4
      vel_msg.linear.x = 0.0

    self.cmd_vel_pub.publish(vel_msg)

    # * To change state...
    if error_dist <= self.distance_precision:
      rospy.loginfo("Error distance: " + str(round(error_dist,2)))
      self.current_state = "HERE"
    elif np.abs(error_theta) <= self.initial_angle_precision:
      rospy.loginfo("Error theta: " + str(round(error_theta,2)))
      self.current_state = "GO"

  def go_straight(self,robot_theta, target=Point(), robot_pos=Point()):
    # Calculate thetaGTG
    y_target = target.y
    x_target = target.x
    thetaGTG = np.arctan2(y_target-robot_pos.y,x_target-robot_pos.x)
    error_theta = self.limit_angle(thetaGTG - robot_theta)
    error_dist = np.sqrt(pow(y_target-robot_pos.y,2)+pow(x_target-robot_pos.x,2))
    # rospy.loginfo(error_theta)

    vel_msg = Twist()

    if error_dist > self.distance_precision:
      vel_msg.linear.x = 0.2
      vel_msg.angular.z = 0.0
      # vel_msg.angular.z = 0.1 if error_theta > 0 else -0.1

    self.cmd_vel_pub.publish(vel_msg)

    # * To change state...
    if error_dist <= self.distance_precision:
      rospy.loginfo("Error distance: " + str(round(error_dist,2)))
      self.current_state = "HERE"
    
    elif np.abs(error_theta) > self.angle_precision:
      rospy.loginfo("Error theta: " + str(round(error_theta,2)))
      self.current_state = "FIX"

  def done(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    self.cmd_vel_pub.publish(vel_msg)

  def limit_angle(self,angle):
    return np.arctan2(np.sin(angle),np.cos(angle))

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

  def gtg_Switch(self,msg):
    self.active = msg.data

  def get_goal(self, msg):
    self.target = msg
    self.goal_received = True
    

  def cleanup(self):  
      '''This function is called just before finishing the node.'''
      print("Finish Message!!!")  
      vel_msg = Twist()
      self.cmd_vel_pub.publish(vel_msg)

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('move_forward_some_time') # Node Name
    try: GoToGoal()  # Class Name
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")