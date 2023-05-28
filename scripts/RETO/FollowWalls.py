#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan   #Lidar 

class GoToGoal():  
  """ Clase para implementar un Follow Walls
  """
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    ###******* INIT PUBLISHERS *******###  
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 
    rospy.Subscriber("base_scan", LaserScan, self.get_ranges) 
  
    ###******* INIT SERVICES *******### 
    # service= rospy.Service('nombreeeee', objeto, self.callback)

    ###******* INIT CONSTANTS/VARIABLES *******###  
    self.active = True 
    self.lidar_received = False
    self.current_state = "FOLLOW"

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

  def find_wall(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.2
    vel_msg.angular.z = 0.1
    self.cmd_vel_pub.publish(vel_msg)
  
  def turn_left_h(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.5
    self.cmd_vel_pub.publish(vel_msg)

  def turn_right_h(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = -0.5
    self.cmd_vel_pub.publish(vel_msg)
  
  def turn_right(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.1
    vel_msg.angular.z = -0.5
    self.cmd_vel_pub.publish(vel_msg)

  def turn_left(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.1
    vel_msg.angular.z = 0.5
    self.cmd_vel_pub.publish(vel_msg)

  def follow_wall(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.2
    vel_msg.angular.z = 0.0
    rospy.logwarn("Distancias laterales: " + str(self.R) + str(self.L))
    self.cmd_vel_pub.publish(vel_msg)
  
  def sentinel(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.5
    self.cmd_vel_pub.publish(vel_msg)
  
  def stop_robot(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    self.cmd_vel_pub.publish(vel_msg)


  def get_ranges(self, msg=LaserScan()):
    if self.lidar_received is False:
      self.areas = {
        "Right" : min(min(msg.ranges[215:359]), 10),
        "FRight": min(min(msg.ranges[360:431]), 10),
        "Front" : min(min(msg.ranges[432:717]), 10),  
        "FLeft" : min(min(msg.ranges[718:789]), 10),
        "Left"  : min(min(msg.ranges[790:933]), 10),
      }
      self.lidar_received = True


  def get_state(self):
    state_description = ""
    areas = self.areas 
    self.R = areas['Right']
    self.L = areas['Left']

    d_lateral = 0.25
    d_diagonal = 0.25
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
    
    # * Giro Arbitrario a la izquierda
    elif F and not L and not R:
      state_description = "Muro Enfrente - Giro Arbitrario"
      self.current_state = "T_LEFTH"
    
    # * Seguir hacia adelante
    elif not F:
      state_description = "Seguir el Muro"
      self.current_state = "FOLLOW"
    
    # * Modo sentinela
    else:
      state_description = "Modo sentinela"
      self.current_state = "SENTINEL"
    # # * Detener Robot
    # else:  
    #   state_description = "Detener Robot"
    #   rospy.logerr("Estado:")
    #   rospy.logerr("Fontral:" + str(F))
    #   rospy.logerr("Izquierdo:" + str(L))
    #   rospy.logerr("Derecho:" + str(R))
    #   rospy.logerr("FIzq:" + str(FL))
    #   rospy.logerr("FDer:" + str(FR))
    #   self.current_state = "STOP"
    
    rospy.loginfo(state_description)

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