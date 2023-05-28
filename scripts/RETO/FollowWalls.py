#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan   #Lidar 

class GoToGoal():  
  """ Clase para implementar un GO TO GOAL
  """
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    ###******* INIT PUBLISHERS *******###  
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 
    rospy.Subscriber("base_scan", LaserScan, self.get_ranges) 
  
    ###******* INIT SERVICES *******### 
    # service= rospy.Service('nombreeeee', objeto, self.callback)

    ###******* INIT CONSTANTS/VARIABLES *******###  
    self.active = False 
    self.current_state = "FIND"

    # Regions of Interest for the robot
    self.areas = {
      "right" : 0.0,
      "fright": 0.0,
      "front" : 0.0,  
      "fleft" : 0.0,
      "left"  : 0.0,
    }

    states = {
      "FIND" : "Find The Wall",
      "T_LEFT" : "Turn Left corner",
      "T_RIGHT" : "Turn Right corner",
      "FOLLOW" : "Follow the Wall",
      "T_LEFTH" : "Turn Left Hard",
      "T_RIGHTH" : "Turn Right Hard",      
    }

    rate = rospy.Rate(20) # The rate of the while loop will be 50Hz 
    rospy.loginfo("Starting Message!")     
    ###******* PROGRAM BODY *******###  
    while not rospy.is_shutdown(): 
      # if the node is not active, do nothing
      if self.active is False: 
        rate.sleep() 
        continue

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

      rate.sleep() 

  def find_wall(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.2
    vel_msg.angular.z = 0.1
    self.cmd_vel_pub.publish(vel_msg)
  
  def turn_left_h(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.2
    self.cmd_vel_pub.publish(vel_msg)

  def turn_right_h(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = -0.2
    self.cmd_vel_pub.publish(vel_msg)
  
  def turn_right(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.1
    vel_msg.angular.z = -0.1
    self.cmd_vel_pub.publish(vel_msg)

  def turn_left(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.1
    vel_msg.angular.z = 0.1
    self.cmd_vel_pub.publish(vel_msg)

  def follow_wall(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.2
    vel_msg.angular.z = 0.0
    self.cmd_vel_pub.publish(vel_msg)

  def get_ranges(self, msg=LaserScan()):
    # TODO Cambiar los rangos
    self.areas = {
      "Right" : min(min(msg.ranges[215:359]), 10),
      "FRight": min(min(msg.ranges[360:502]), 10),
      "Front" : min(min(msg.ranges[503:645]), 10),  
      "FLeft" : min(min(msg.ranges[646:788]), 10),
      "Left"  : min(min(msg.ranges[789:931]), 10),
    }

  def get_state(self):
    vel_msg = Twist()
    state_description = ""
    areas = self.areas 

    d = 1.5
    R = areas['Right'] < d
    FR = areas['FRight'] < d
    F = areas['Front'] < d
    FL = areas['FLeft'] < d
    L = areas['Left'] < d


    # * Giro Forzoso Derecha
    if F and FL and L and not R and not FR:
      self.current_state = "T_RIGHTH"
    # * Giro Forzoso Izquierda
    elif F and FR and R and not L and not FL:
      self.current_state = "T_LEFTH"
    # ! Hacer Giro de 180 grados
    elif F and FR and R and L and FL:
      self.current_state = "T_LEFTH"
    # * Girar por la derecha
    elif R and not F and not FR:
      self.current_state = "T_RIGHT"
    # * Girar usando Following Walls
    elif L and not F and not FL:
      self.current_state = "T_LEFT"
    elif F and not L and not R:
      self.current_state = "T_LEFTH"
    else:
      self.current_state = "FOLLOW"

  def cleanup(self):  
      '''This function is called just before finishing the node.'''
      print("Finish Message!!!")  
      vel_msg = Twist()
      self.cmd_vel_pub.publish(vel_msg)

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('FOLLOWALL') # Node Name
    try: GoToGoal()  # Class Name
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")