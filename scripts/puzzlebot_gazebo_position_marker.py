#! /usr/bin/env python 
import rospy 
from geometry_msgs.msg import Pose 
from visualization_msgs.msg import Marker 
from std_msgs.msg import Header 
from gazebo_msgs.srv import GetModelState, GetModelStateRequest 
from tf.transformations import quaternion_from_euler 


def fill_marker(pose=Pose()): 
    marker = Marker() 
    marker.header.frame_id = "odom" 
    marker.header.stamp = rospy.Time.now() 
    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3; Mesh: 10 
    marker.type = 10 
    marker.id = 0 

    # Use the stl file MCR2_1000_13_Chassis.stl,  
    #marker.mesh_resource = "package://puzzlebot_sim/meshes/MCR2_1000_0_Puzzlebot.stl" 
    marker.mesh_resource = "package://puzzlebot_sim/meshes/MCR2_1000_0_Puzzlebot.stl" 

    # Set the scale of the marker 
    marker.scale.x = 1 
    marker.scale.y = 1 
    marker.scale.z = 1 

    # Set the color 
    marker.color.r = 1.0 
    marker.color.g = 1.0 
    marker.color.b = 0.0 
    marker.color.a = 1.0 

    # Set the pose of the marker 
    marker.pose.position.x = pose.position.x 
    marker.pose.position.y = pose.position.y 
    marker.pose.position.z = 0.0 

    # Set the marker orientation 
    #quat=quaternion_from_euler(0.0,0.0,theta) 
    marker.pose.orientation.x = pose.orientation.x 
    marker.pose.orientation.y = pose.orientation.y 
    marker.pose.orientation.z = pose.orientation.z 
    marker.pose.orientation.w = pose.orientation.w 

    return marker 


rospy.init_node('puzzlebot_gazebo_marker') 
marker_odom_pub = rospy.Publisher("/gazebo_marker", Marker, queue_size = 1) 
odom_marker=Marker() 
odom_marker.pose.orientation.w=1.0 
rospy.wait_for_service ('/gazebo/get_model_state') 
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState) 

model = GetModelStateRequest() 
model.model_name='puzzlebot' 
r = rospy.Rate(10) 
print("Init node") 
while not rospy.is_shutdown(): 
    result = get_model_srv(model) 
    odom_marker = fill_marker(result.pose) 
    marker_odom_pub.publish(odom_marker) 
    r.sleep() 