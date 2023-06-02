#!/usr/bin/env python  
import rospy  
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan   #Lidar 
from tf.transformations import euler_from_quaternion

class FollowWalls():  
  """ Clase para implementar un Follow Walls
  """
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    ###******* INIT PUBLISHERS *******###  
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 
    rospy.Subscriber("base_scan", LaserScan, self.get_ranges) 
    rospy.Subscriber("GOAL", Point, self.get_goal)
    rospy.Subscriber('fw_topic', Bool, self.get_active)
    rospy.Subscriber('/odom', Odometry, self.get_odom) # Comes From KalmanFilter

    #?#********** NODE MANAGER VARIABLES **********#?#
    self.active = False 
    self.lidar_received = False
    self.current_state = "FIND"
    self.goal_received = False

    #?#********** REGIONS OF INTEREST OF THE ROBOT **********#?#
    self.areas = {
      "right" : 0.0,
      "fright": 0.0,
      "front" : 0.0,  
      "fleft" : 0.0,
      "left"  : 0.0,
      "MIN_ANGLE" : 0.0,
    }

    states = {

      "FIND" : "Find The Wall",
      "T_LEFT" : "Turn Left corner",
      "T_RIGHT" : "Turn Right corner",
      "FOLLOW" : "Follow the Wall",
      "T_LEFTH" : "Turn Left Hard",
      "T_RIGHTH" : "Turn Right Hard",      
    }
    
    self.turn_decision = "T_LEFTH"
    #?#********** ROBOT POSITION AND GOAL **********#?#
    # Posicion del Robot
    self.robot_pos = Point()
    self.robot_theta = 0.0

    # Define goal point
    self.target = Point()

    #?#********** FOLLOW WALL CONSTANTS **********#?#

    rate = rospy.Rate(10) # The rate of the while loop will be 50Hz 
    rospy.loginfo("STARTING NODE FW")     
    ###******* PROGRAM BODY *******###  
    while not rospy.is_shutdown(): 
      # if the node is not active, do nothing
      if self.active is False: 
        rospy.logfatal("NODE OFF")
        self.current_state = "FIND"
        rate.sleep() 
        continue

      if self.goal_received is False:
        rospy.loginfo("WAITING GOAL")
        rate.sleep()
        continue

      # if no lidar_received do nothing
      if self.lidar_received is False:
        rate.sleep() 
        continue

      self.get_state()

      if self.current_state == "FIND":
        self.find_wall()
      elif self.current_state == "T_LEFT":
        self.turn_left()
      elif self.current_state == "T_RIGHT":
        self.turn_right()
      elif self.current_state == "T_LEFTH":
        self.turn_left_h()
      elif self.current_state == "T_RIGHTH":
        self.turn_right_h()
      elif self.current_state == "FOLLOW":
        self.follow_wall()
      elif self.current_state == "SENTINEL":
        self.sentinel()
      elif self.current_state == "STOP":
        self.stop_robot()
      
      self.lidar_received = False

      rate.sleep() 

  #?#********** FOLLOW WALL BEHAVIOURS **********#?#
  def find_wall(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.2
    vel_msg.angular.z = 0.0
    self.cmd_vel_pub.publish(vel_msg)
  def turn_left_h(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.4
    self.cmd_vel_pub.publish(vel_msg)
  def turn_right_h(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = -0.4
    self.cmd_vel_pub.publish(vel_msg)
  def turn_right(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.07
    vel_msg.angular.z = -0.5
    self.cmd_vel_pub.publish(vel_msg)
  def turn_left(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.07
    vel_msg.angular.z = 0.5
    self.cmd_vel_pub.publish(vel_msg)
  def follow_wall(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.2
    # * Calcular giro de seguridad
    # What side are we following ?
    R = self.areas["Right"]
    L = self.areas["Left"]
    side = True if R < 0.3 else False
    # Wall on the Right Side
    if side is True:
      control = 0.15 if R < 0.20 else -0.0
    # Wall on the Left Side
    else:
      control = -0.15 if L < 0.20 else 0.0
    vel_msg.angular.z = control
    # vel_msg.angular.z = 0.0
    self.cmd_vel_pub.publish(vel_msg)
  def sentinel(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.4
    self.cmd_vel_pub.publish(vel_msg)
  def stop_robot(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    self.cmd_vel_pub.publish(vel_msg)

  #?#********** GETTERS AND CALLBACK **********#?#
  def get_ranges(self, msg=LaserScan()):
    if self.lidar_received is False:
      self.areas = {
        "Right" : min(min(msg.ranges[215:359]), 10),
        "FRight": min(min(msg.ranges[360:502]), 10),
        "Front" : min(min(msg.ranges[503:645]), 10),  
        "FLeft" : min(min(msg.ranges[646:788]), 10),
        "Left"  : min(min(msg.ranges[789:931]), 10),
        "MIN_ANGLE" : self.get_closet_object(msg),
      }
      self.lidar_received = True
  def get_state(self):
    state_description = ""
    areas = self.areas 

    d_lateral = 0.30
    d_diagonal = 0.30
    R = areas['Right'] < d_lateral
    FR = areas['FRight'] < d_diagonal
    F = areas['Front'] < d_lateral
    FL = areas['FLeft'] < d_diagonal
    L = areas['Left'] < d_lateral


    # * Giro Forzoso Derecha
    if F and FL and L and not R and not FR:
      state_description = "Esquina - Girar Derecha"
      self.current_state = "T_RIGHTH"
    
    # * Giro Forzoso Izquierda
    elif F and FR and R and not L and not FL:
      state_description = "Esquina - Girar Izquierda"
      self.current_state = "T_LEFTH"
    
    # ! Hacer Giro de 180 grados
    elif F and FR and R and L and FL:
      state_description = "Cierre Tipo U - Girar 180 Grados"
      self.current_state = "T_LEFTH"
    
    # * Girar siguiendo la esquina a la Derecha
    elif R and not F and not FR:
      state_description = "Seguir Esquina - Derecha"
      self.current_state = "T_RIGHT"
    
    # * Girar siguiendo la esquina a la izquierda
    elif L and not F and not FL:
      state_description = "Seguir Esquina - Izquierda"
      self.current_state = "T_LEFT"
    
    # * Giro Arbitrario con decision
    elif F and not L and not R:
      state_description = "Muro Enfrente - Giro Arbitrario"
      self.current_state = self.turn_decision
    
    # * Buscar Muro 
    elif not F and not L and not FL and not FR and not R:
      state_description = "Find Wall"
      self.current_state = "FIND"
      self.turn_decision = self.choose_side()
    
    # * Seguir muro
    elif not F:
      state_description = "Seguir el Muro"
      self.current_state = "FOLLOW"
    
    # * Modo sentinela
    else:
      state_description = "Modo sentinela"
      self.current_state = self.turn_decision
    
    rospy.loginfo(state_description)
  def get_active(self, msg=Bool):
    self.active = msg.data
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
  def get_closet_object(self, lidar_data=LaserScan()):
    """ This function returns the closest object to the robot
    This functions receives a ROS LaserScan message and returns the distance and direction to the closest object
    returns  closest_range [m], closest_angle [rad], 
    """ 
    idx_low = 360
    idx_high = 931
    data = lidar_data.ranges[360:931]
    min_idx = np.argmin(data) 
    closest_angle = lidar_data.angle_min + (min_idx + 360) * lidar_data.angle_increment 
    # limit the angle to [-pi, pi] 
    closest_angle = self.limit_angle(closest_angle) 
    return closest_angle 
  def get_theta_goal(self):
    delta_y = self.target.y - self.robot_pos.y
    delta_x = self.target.x - self.robot_pos.x
    theta_gtg = np.arctan2(delta_y, delta_x)
    return theta_gtg
  def get_goal(self, msg=Point):
    self.target = msg
    self.goal_received = True
  #?#********** HELPERS **********#?#
  def choose_side(self):
    thetaAO = self.areas["MIN_ANGLE"] - np.pi
    thetaAO = self.limit_angle(thetaAO)
    thetaFW = thetaAO + np.pi/2.0
    thetaFW = self.limit_angle(thetaFW)
    theta_goal = self.get_theta_goal()
    diff_theta = np.abs(thetaFW - theta_goal)
    # diff_theta = self.limit_angle(diff_theta)
    if diff_theta < np.pi/2.0:
      return "T_RIGHTH"
    else:
      return "T_LEFTH"
  def limit_angle(self, angle):
    """Funcion para limitar de -PI a PI cualquier angulo de entrada"""
    return np.arctan2(np.sin(angle), np.cos(angle))

  def cleanup(self):  
      '''This function is called just before finishing the node.'''
      print("FINISH MESSAGE STOPPING ROBOT")  
      vel_msg = Twist()
      self.cmd_vel_pub.publish(vel_msg)

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('FOLLOWALL') # Node Name
    try: FollowWalls()  # Class Name
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")