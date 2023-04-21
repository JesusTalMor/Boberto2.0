#!/usr/bin/env python  

import rospy  
from geometry_msgs.msg import Twist  
from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import Float32 
from tf.transformations import quaternion_from_euler 
import numpy as np 


class PuzzlebotKinClass():  
  """ 
  This class will do the following: 
  subscribe to the /cmd_vel topic  
  publish the simulated pose of the robot to /pose_sim topic  
  publish to /wr and /wl the simulated wheel speed.  
  """
  def __init__(self):  
    #?#********** SUBSCRIBERS/PUBLISHERS **********###  
    #* Create the subscriber to cmd_vel topic 
    rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb) 
    #* Create ROS publishers 
    self.pose_sim_pub = rospy.Publisher('pose_sim', PoseStamped ,queue_size=1) #Publisher to pose_sim topic 
    self.wr_pub = rospy.Publisher('wr', Float32 ,queue_size=1) # Publisher to wr topic 
    self.wl_pub = rospy.Publisher('wl', Float32 ,queue_size=1) # Publisher to wl topic 
    self.wr_pos_pub = rospy.Publisher('wr_pos', Float32 ,queue_size=1) # Publisher to wr_pos topic 
    self.wl_pos_pub = rospy.Publisher('wl_pos', Float32 ,queue_size=1) # Publisher to wl_pos topic 
    
    #?#********** ROBOT CONSTANTS **********###    
    self.r = 0.05 #puzzlebot wheel radius [m] 
    self.L = 0.18 #puzzlebot wheel separation [m] 
    self.delta_t = 0.02 # Desired time to update the robot's pose [s] 
    rate = rospy.Rate(int(1.0/self.delta_t)) # The rate of the while loop will be the inverse of the desired delta_t 
    
    #?#********** VARIABLES **********###  
    self.w = 0.0 # robot's angular speed [rad/s] 
    self.v = 0.0 # robot's linear speed [m/s] 
    x = 0.0   # robot position in X axis [m]
    y = 0.0   # robot position in Y axis [m]
    theta = 0.0 # robot orientation in Z axis [rad]
    wr_pos = 0.0 # Right Wheel position [rad]
    wl_pos = 0.0 # Left Wheel position [rad]
    
    #?#********** MAIN LOOP **********###  
    while not rospy.is_shutdown(): 
      #* Get current position
      [x, y, theta] = self.get_robot_pose(self.v, self.w, (x, y, theta)) 
      #* Fill pose_stamped object
      pose_stamped = self.get_pose_stamped(x, y, theta)
      #* Get current wheel angular velocity and position
      [wl, wr] = self.get_wheel_speeds(self.v, self.w) # TODO DONE
      [wl_pos, wr_pos] = self.get_wheel_pose((wl,wr), (wl_pos, wr_pos)) # TODO DONE
      
      #********** PUBLISH to TOPICS **********# 
      self.pose_sim_pub.publish(pose_stamped) 
      self.wr_pub.publish(wr) 
      self.wl_pub.publish(wl) 
      self.wl_pos_pub.publish(wl_pos)
      self.wr_pos_pub.publish(wr_pos)
      rate.sleep() 

  def cmd_vel_cb(self, msg=Twist()): 
    """ Callback para sacar velocidades del robot"""
    self.v = msg.linear.x 
    self.w = msg.angular.z 

  def get_wheel_speeds(self, v, w): 
    """ Calcula la velocidades de las llantas """
    wr = ((2*v) + (w*self.L))/(2*self.r)
    wl = ((2*v) - (w*self.L))/(2*self.r) 
    return [wl, wr] 

  def get_wheel_pose(self, wheel_vel, pose_ant):
    # Desempaquetar las variables
    wl, wr = wheel_vel
    wl_pos, wr_pos = pose_ant
    wl_pos += wl*self.delta_t
    wr_pos += wr*self.delta_t
    return [wl_pos, wr_pos]

  def get_pose_stamped(self, x, y, yaw): 
    # x, y and yaw are the robot's position (x,y) and orientation (yaw) 
    # Write the data as a ROS PoseStamped message 
    pose_stamped = PoseStamped() 
    pose_stamped.header.frame_id = "odom"    #This can be changed in this case I'm using a frame called odom. 
    pose_stamped.header.stamp = rospy.Time.now() 
    # Position 
    pose_stamped.pose.position.x = x 
    pose_stamped.pose.position.y = y 
    # Rotation of the mobile base frame w.r.t. "map" frame as a quaternion 
    quat = quaternion_from_euler(0,0,yaw) 
    pose_stamped.pose.orientation.x = quat[0] 
    pose_stamped.pose.orientation.y = quat[1] 
    pose_stamped.pose.orientation.z = quat[2] 
    pose_stamped.pose.orientation.w = quat[3] 
    return pose_stamped 

  def get_robot_pose(self, v, w, pose): 
    """
    This functions receives the robot speed v [m/s] and w [rad/s] 
    and gets the robot's pose (x, y, theta) where (x,y) are in [m] and (theta) in [rad] 
    is the orientation,     
    """
    # Desempaquetar pose
    [x, y, theta] = pose
    # Calcular aqui la posicion actual del robot
    x_act = x + (v * (np.cos(theta)) * self.delta_t)
    y_act = y + (v * (np.sin(theta)) * self.delta_t)
    theta_act = theta + (w * self.delta_t)
    # print('X:', x_act, 'Y:', y_act, 'theta', theta_act)
    return [x_act, y_act, theta_act] 

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
  # first thing, init a node! 
  rospy.init_node('puzzlebot_kinematic_model')  
  PuzzlebotKinClass()