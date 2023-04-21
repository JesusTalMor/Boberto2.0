#!/usr/bin/env python  

import rospy  
from geometry_msgs.msg import PoseStamped 
from visualization_msgs.msg import Marker 
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32  
# Because of transformations 
from geometry_msgs.msg import TransformStamped 
import tf2_ros 


class PuzzlebotTfClass():  
  def __init__(self):  
    rospy.Subscriber("pose_sim", PoseStamped, self.pose_sim_cb) 
    rospy.Subscriber("wl_pos", Float32, self.get_wl_pose) 
    rospy.Subscriber("wr_pos", Float32, self.get_wr_pose) 
    # self.marker1_pub = rospy.Publisher("puzzlebot_marker", Marker, queue_size = 1)
    # self.marker2_pub = rospy.Publisher("left_wheel_marker", Marker, queue_size = 1)
    # self.marker3_pub = rospy.Publisher("right_wheel_marker", Marker, queue_size = 1) 
    self.tf_br = tf2_ros.TransformBroadcaster() 
    self.robot_pose = PoseStamped() 
    self.left_wheel_pos = 0.0
    self.right_wheel_pos = 0.0
    self.robot_pose.pose.orientation.w = 1.0 #this is necessary to avoid errors with the quaternion. 
    rate = rospy.Rate(50) # The rate of the while loop 
    while not rospy.is_shutdown(): 
      self.send_base_link_tf(self.robot_pose) 
      self.send_left_wheel_tf(self.left_wheel_pos)
      self.send_right_wheel_tf(self.right_wheel_pos)
      ######## Publish a marker to rviz ######### 
      # marker = self.fill_base_link_marker() 
      # self.marker1_pub.publish(marker) 
      # marker = self.fill_left_wheel_marker()
      # self.marker2_pub.publish(marker) 
      # marker = self.fill_right_wheel_marker()
      # self.marker3_pub.publish(marker) 
      rate.sleep() 

    
  def pose_sim_cb(self, msg): self.robot_pose = msg 
  def get_wl_pose(self, msg=Float32()): self.left_wheel_pos = msg.data
  def get_wr_pose(self, msg=Float32()): self.right_wheel_pos = msg.data

  def send_base_link_tf(self, pose_stamped=PoseStamped()): 
    # This receives the robot's pose and broadcast a transformation. 
    t = TransformStamped() 
    t.header.stamp = rospy.Time.now() 
    t.header.frame_id = "odom" 
    t.child_frame_id = "base_link" 
    #Copy data from the received pose to the tf  
    t.transform.translation.x = pose_stamped.pose.position.x 
    t.transform.translation.y = pose_stamped.pose.position.y 
    t.transform.translation.z = pose_stamped.pose.position.z 
    t.transform.rotation.x = pose_stamped.pose.orientation.x 
    t.transform.rotation.y = pose_stamped.pose.orientation.y 
    t.transform.rotation.z = pose_stamped.pose.orientation.z 
    t.transform.rotation.w = pose_stamped.pose.orientation.w 
    # Send the transformation 
    self.tf_br.sendTransform(t) 
  def fill_base_link_marker(self): 
    marker = Marker() 
    marker.header.frame_id = "base_link" 
    marker.header.stamp = rospy.Time.now() 
    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3; Mesh: 10 
    marker.type = 10 
    marker.id = 0 
    # Use the stl file MCR2_1000_13_Chassis.stl,  
    marker.mesh_resource = "package://puzzlebot_sim/descriptions/MCR2_1000_13_Chassis.stl" 
    # Set the scale of the marker 
    marker.scale.x = 1 
    marker.scale.y = 1 
    marker.scale.z = 1 
    # Set the color 
    marker.color.r = 0.8 
    marker.color.g = 0.0 
    marker.color.b = 1.0 
    marker.color.a = 1.0 
    # Set the pose of the marker 
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0 
    # Rotacion 90 grados en z y en y
    marker.pose.orientation.x = 0.5
    marker.pose.orientation.y = 0.5
    marker.pose.orientation.z = 0.5
    marker.pose.orientation.w = 0.5

    return marker 


  def send_left_wheel_tf(self, position):
    # This receives the robot's pose and broadcast a transformation. 
    t = TransformStamped() 
    t.header.stamp = rospy.Time.now() 
    t.header.frame_id = "base_link" 
    t.child_frame_id = "left_wheel" 
    #Copy data from the received pose to the tf  
    t.transform.translation.x = 0.05
    t.transform.translation.y = 0.09
    t.transform.translation.z = 0
    quat = quaternion_from_euler(0,position,0)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    # Send the transformation 
    self.tf_br.sendTransform(t)
  def fill_left_wheel_marker(self):
    marker = Marker() 
    marker.header.frame_id = "left_wheel" 
    marker.header.stamp = rospy.Time.now() 
    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3; Mesh: 10 
    marker.type = 10 
    marker.id = 0 
    # Use the stl file MCR2_1000_13_Chassis.stl,  
    marker.mesh_resource = "package://puzzlebot_sim/descriptions/MCR2_1000_1_1_Wheel_Coupler_2.stl" 
    # Set the scale of the marker 
    marker.scale.x = 1 
    marker.scale.y = 1 
    marker.scale.z = 1 
    # Set the color 
    marker.color.r = 0.8 
    marker.color.g = 0.0 
    marker.color.b = 1.0 
    marker.color.a = 1.0 
    # Set the pose of the marker 
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0 
    # Rotacion 90 grados en z y en y
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1

    return marker 


  def send_right_wheel_tf(self, position):
    # This receives the robot's pose and broadcast a transformation. 
    t = TransformStamped() 
    t.header.stamp = rospy.Time.now() 
    t.header.frame_id = "base_link" 
    t.child_frame_id = "right_wheel" 
    #Copy data from the received pose to the tf  
    t.transform.translation.x = 0.05
    t.transform.translation.y = -0.09
    t.transform.translation.z = 0
    quat = quaternion_from_euler(0,position,0)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    # Send the transformation 
    self.tf_br.sendTransform(t)
  def fill_right_wheel_marker(self):
    marker = Marker() 
    marker.header.frame_id = "right_wheel" 
    marker.header.stamp = rospy.Time.now() 
    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3; Mesh: 10 
    marker.type = 10 
    marker.id = 0 
    # Use the stl file MCR2_1000_13_Chassis.stl,  
    marker.mesh_resource = "package://puzzlebot_sim/descriptions/MCR2_1000_1_1_Wheel_Coupler_2.stl" 
    # Set the scale of the marker 
    marker.scale.x = 1 
    marker.scale.y = 1 
    marker.scale.z = 1 
    # Set the color 
    marker.color.r = 0.8 
    marker.color.g = 0.0 
    marker.color.b = 1.0 
    marker.color.a = 1.0 
    # Set the pose of the marker 
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0 
    # Rotacion 90 grados en z y en y
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1

    return marker 

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
  # first thing, init a node! 
  rospy.init_node('puzzlebot_tf_broadcaster')  
  PuzzlebotTfClass()  