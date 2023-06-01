#!/usr/bin/env python  
import rospy  
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class Bug2():  
   '''
   Tonteria de Template
   '''
   def __init__(self):  
      rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
      ###******* INIT PUBLISHERS *******###  
      rospy.Subscriber("base_scan", LaserScan, self.get_lidar_cb)
      rospy.Subscriber('/odom', Odometry, self.get_odom)
      rospy.Subscriber('GOAL',Point, self.get_goal)
      self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 
      self.gtg_topic = rospy.Publisher('gtg_topic', Bool, queue_size=1)
      self.fw_topic = rospy.Publisher('fw_topic', Bool, queue_size=1)    

      ###******* INIT CONSTANTS/VARIABLES *******###  
      # Posicion del Robot
      self.robot_pos = Point()
      self.robot_theta = 0.0

      # Definicion de punto inicial y goal
      self.initial_pos = Point()
      self.target = Point()
      self.goal_received = False

      self.gtg_active = False
      self.fw_active = False

      # Definicion de estados
      self.current_state = "GTG"
      states = {
      "GTG" : "GO_TO_GOAL",
      "FW" : "WALL_FOLLOWER",
      "S" : "STOP" 
      }

      # Bandera para indicar cuando tenga datos del las regiones
      self.region_recive = False

      # # Tiempo que se dura en un estado
      # self.time = 0
      # loop = 0

      # Variables
      progress = 0.1 # Para verificar si el robot avanzo esta distancia antes de cambiar de estado
      tolerance = 0.05 # 0.05 Si el robot esta asi de cerca de la linea con respecto a cuando cambio a FW, cambiara a gtg
      self.d_t = 0.0
      self.D_Fw = 0.0
      goal_tolerance = 0.05
      #self.change_state("GTG")

      rate = rospy.Rate(50) # The rate of the while loop will be 50Hz 
      rospy.loginfo("Starting Message!")     

      ###******* PROGRAM BODY *******###  
      while not rospy.is_shutdown(): 
         # Espera a que reciba regiones del lidar
         if self.region_recive is False:
            rate.sleep()
            continue

         if self.goal_received is False:
            rate.sleep()
            continue

         print(self.gtg_active)
         print(self.fw_active)
         if self.gtg_active is False and self.fw_active is False:
            print("Entre una vez")
            self.gtg_topic.publish(True)
            self.fw_topic.publish(False)  
            self.gtg_active = True
            self.fw_active = True
         

         # Calcula la distancia del robot a la linea
         distance_line = self.getDistanceLine(self.robot_pos)
   

         # Calcula el angulo para evitar obstaculo
         thetaAO = self.get_theta_ao(self.closest_angle)

         # Calcula el angulo para rodear obstaculo 
         thetaGTG =self.get_theta_gtg(self.target.x, self.target.y, self.robot_pos.x, self.robot_pos.y, self.robot_theta) 

         # Distancia al goal
         self.d_t = np.sqrt((self.target.x-self.robot_pos.x)**2+(self.target.y-self.robot_pos.y)**2) 
         
         if self.d_t < goal_tolerance:
            self.change_state("STOP")
            self.done()
            self.goal_received = False
            self.initial_pos = self.robot_pos
         elif self.current_state == "GTG":
            # Si hay un obstaculo, cambia a comportamiento de FW
            if self.Front > 0.30 and self.Front < 0.40:
               self.change_state("FW")
               print ("se cambio a fw")
         elif self.current_state == "FW":
            # Calcular angulos 
            theta_clear_shot = abs(self.limit_angle(thetaAO-thetaGTG))
            print("Distancia: " ,self.D_Fw)
            print(self.d_t < (self.D_Fw - progress))
            print(theta_clear_shot < np.pi/2.0)
            print(distance_line < tolerance)
            print(" ")

            #       Ya avanzo una distancia ?     Tiene obstaculos a la vista ?     Esta cerca de la ruta ?
            if self.d_t < (self.D_Fw - progress) and theta_clear_shot < np.pi/2.0 and distance_line < tolerance:
               self.change_state("GTG")

         rate.sleep() 

   #?#********** LIMITADORES #?#**********
   def limit_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

   #?#********** MANEJO DE ESTADOS #?#**********
   def change_state(self,state):
      self.current_state = state
      if self.current_state == "GTG":
         self.gtg_topic.publish(True)
         self.fw_topic.publish(False)
         rospy.sleep(3)
      if self.current_state == "FW":
         self.D_Fw = self.d_t # Guarda la distancia al goal cuando se hace el cambio de comportamiento a FW
         print("Distancia: " ,self.D_Fw)
         self.gtg_topic.publish(False)
         self.fw_topic.publish(True)
         self.gtg_active = False
         self.fw_active = True
      if self.current_state == "STOP":
         self.gtg_topic.publish(False)
         self.fw_topic.publish(False)


   #?# ********** CALLBACKS #?#**********
   def get_lidar_cb(self,msg):
      # Para delimitar region de al frente del robot
      self.region_recive = True
      aux = msg.ranges
      self.Front = min(aux[503:645])

      # Para obtener closest angle
      min_idx = np.argmin(aux)
      closest_angle = msg.angle_min + min_idx * msg.angle_increment 
      # limitar el angulo
      self.closest_angle = self.limit_angle(closest_angle)

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

   def get_goal(self, msg=Point()):
      self.target = msg
      self.goal_received = True
   
   #?# ********** COMPORTAMIENTOS #?#**********
   def get_theta_ao(self, theta_closest): 
        ##This function returns the angle for the Avoid obstacle behavior  
        # theta_closest is the angle to the closest object [rad] 
        #This functions returns the angle for the Avoid obstacle behavior [rad] 
        ############################################################ 
        thetaAO=theta_closest-np.pi 
        #limit the angle to [-pi,pi] 
        thetaAO = self.limit_angle(thetaAO)
        return thetaAO

   def get_theta_gtg(self, x_target, y_target, x_robot, y_robot, theta_robot): 
        #This function returns the angle to the goal 
        theta_target=np.arctan2(y_target - y_robot, x_target - x_robot) 
        e_theta= theta_target - theta_robot 
        #limit e_theta from -pi to pi 
        #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi 
        e_theta = self.limit_angle(e_theta)
        return e_theta
   
   def getDistanceLine(self,actual_pos):
      up = math.fabs((self.target.y - self.initial_pos.y) * actual_pos.x - (self.target.x - self.initial_pos.x) * actual_pos.y + (self.target.x * self.initial_pos.y) - (self.target.y * self.initial_pos.x))
      down = math.sqrt(pow(self.target.y - self.initial_pos.y, 2) + pow(self.target.x - self.initial_pos.x, 2))
      return up / down

   #?# ********** CLEAN #?#**********
   def done(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    self.cmd_vel_pub.publish(vel_msg)

   def cleanup(self):  
      '''This function is called just before finishing the node.'''
      print("Finish Message!!!")  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
   rospy.init_node('BUG2_NODE') # Node Name
   try: Bug2()
   except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")