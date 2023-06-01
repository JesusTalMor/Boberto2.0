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
      rospy.Subscriber('odom', Odometry, self.get_odom)
      rospy.Subscriber('GOAL',Point, self.get_goal)
      self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 
      self.gtg_topic = rospy.Publisher('gtg_topic', Bool, queue_size=1)
      self.fw_topic = rospy.Publisher('fw_topic', Bool, queue_size=1)    

      #?#******* INIT CONSTANTS/VARIABLES *******#?#  
      # Posicion del Robot
      self.robot_pos = Point()
      self.robot_theta = 0.0

      # Definicion de punto inicial y goal
      self.initial_pos = Point()
      self.target = Point()

      # Banderas de estados
      self.goal_received = False
      self.region_received = False
      self.lidar_data = LaserScan()
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

      # # Tiempo que se dura en un estado
      # self.time = 0
      # loop = 0

      # Variables
      progress = 0.1 # Para verificar si el robot avanzo esta distancia antes de cambiar de estado
      tolerance = 0.05 # 0.05 Si el robot esta asi de cerca de la linea con respecto a cuando cambio a FW, cambiara a gtg
      self.d_t = 0.0
      self.D_Fw = 0.0
      goal_tolerance = 0.1
      #self.change_state("GTG")

      rate = rospy.Rate(10) # The rate of the while loop will be 50Hz 
      rospy.loginfo("Starting Message!")     

      ###******* PROGRAM BODY *******###  
      while not rospy.is_shutdown(): 
         # El programa jala hasta que reciba un GOAL
         if self.goal_received is False:
            rospy.loginfo("Esperando GOAL")
            rate.sleep()
            continue
         
         # Espera a que reciba regiones del lidar
         if self.region_received is False:
            rate.sleep()
            continue

         # print(self.gtg_active)
         # print(self.fw_active)
         # * Si ambos estados estan em Falso iniciar con GTG
         if self.gtg_active is False and self.fw_active is False:
            self.current_state = "GTG"
            self.gtg_topic.publish(True)
            self.fw_topic.publish(False)  
            self.gtg_active = True
            self.fw_active = False
         

         # Calcula la distancia del robot a la linea
         distance_line = self.get_distance_to_line(self.target, self.initial_pos, self.robot_pos)

         # Obtener objeto mas cercano
         closest_dist, closest_angle = self.get_closet_object(self.lidar_data)

         # Calcula el angulo para evitar obstaculo
         thetaAO = self.get_theta_ao(closest_angle)

         # Calcula el angulo para rodear obstaculo 
         thetaGTG = self.get_theta_gtg(self.robot_theta, self.target, self.robot_pos) 

         # Distancia al goal
         self.d_t = self.get_distance_to_goal(self.target, self.robot_pos)

         # Calcular Angulo de Clear SHOT
         theta_clear_shot = np.abs(self.limit_angle(thetaAO-thetaGTG))
         
         #?#********** MAQUINA DE ESTADOS **********#?#
         rospy.logerr(self.current_state)
         print("Distancia al Goal: " , round(self.d_t, 2))
         print("Progreso FW: ",(self.d_t < (self.D_Fw - progress)))
         print("Angulo ClearShot: ", (theta_clear_shot < np.pi/2.0))
         print("Distancia Recta: ", round(distance_line, 2), " ",(distance_line < tolerance))
         print("--------------------------------")
         if self.d_t < goal_tolerance:
            rospy.loginfo("LLEGUE AL GOAL")
            self.change_state("STOP")
         
         elif self.current_state == "GTG":
            # Si hay un obstaculo, cambia a comportamiento de FW
            if closest_dist > 0.30 and closest_dist < 0.40:
               self.change_state("FW")
               rospy.loginfo("Muro cerca: Cambio a FW")
         
         elif self.current_state == "FW":
            #       Ya avanzo una distancia ?     Tiene obstaculos a la vista ?     Esta cerca de la ruta ?
            if self.d_t < (self.D_Fw - progress) and theta_clear_shot < np.pi/2.0 and distance_line < tolerance:
               self.change_state("GTG")

         rate.sleep() 

   #?#********** LIMITADORES #?#**********
   def limit_angle(self, angle):
      return np.arctan2(np.sin(angle), np.cos(angle))

   #?#********** MANEJO DE ESTADOS #?#**********
   def change_state(self, state="STOP"):
      self.current_state = state
      if self.current_state == "STOP":
         #* Apagar nodos de GTG y FW
         self.gtg_topic.publish(False)
         self.fw_topic.publish(False)
         self.gtg_active = False
         self.fw_active = False
         #* Detener el robot
         self.done()
         #* Preparar para recibir nueva GOAL
         self.goal_received = False
         self.initial_pos = self.robot_pos
      
      elif self.current_state == "GTG":
         self.gtg_topic.publish(True)
         self.fw_topic.publish(False)
         self.gtg_active = True
         self.fw_active = False
         rospy.sleep(3)
      
      elif self.current_state == "FW":
         self.D_Fw = self.d_t # Guarda la distancia al goal cuando se hace el cambio de comportamiento a FW
         # print("Distancia: " ,self.D_Fw)
         self.gtg_topic.publish(False)
         self.fw_topic.publish(True)
         self.gtg_active = False
         self.fw_active = True


   #?# ********** CALLBACKS #?#**********
   def get_lidar_cb(self, msg=LaserScan()):
      self.lidar_data = msg
      self.region_received = True

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
   
   #?# ********** GETTERS **********#?#
   def get_closet_object(self, lidar_data=LaserScan()):
      # Para delimitar region de al frente del robot
      aux = lidar_data.ranges[360:788]
      # Encontrar distancia mas cercana
      Front = min(min(aux), 10)

      # Para obtener closest angle
      min_idx = 503 + np.argmin(aux)
      closest_angle = lidar_data.angle_min + min_idx * lidar_data.angle_increment
      # limitar el angulo
      closest_angle = self.limit_angle(closest_angle)
      return Front, closest_angle

   def get_theta_ao(self, theta_closest): 
      ##This function returns the angle for the Avoid obstacle behavior  
      # theta_closest is the angle to the closest object [rad] 
      #This functions returns the angle for the Avoid obstacle behavior [rad] 
      ############################################################ 
      thetaAO=theta_closest-np.pi 
      #limit the angle to [-pi,pi] 
      thetaAO = self.limit_angle(thetaAO)
      return thetaAO

   def get_theta_gtg(self, theta_robot, target_pos=Point(), robot_pos=Point()): 
      """ This function returns the angle to the goal  """
      # Desempaquetar variables
      y_target, x_target = target_pos.y, target_pos.x
      y_robot, x_robot = robot_pos.y, robot_pos.x
      theta_target=np.arctan2(y_target - y_robot, x_target - x_robot) 
      e_theta= theta_target - theta_robot 
      #limit e_theta from -pi to pi 
      #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi 
      e_theta = self.limit_angle(e_theta)
      return e_theta
   
   def get_distance_to_line(self, target=Point(), initial_pos=Point(), actual_pos=Point()):
      up = math.fabs((target.y - initial_pos.y) * actual_pos.x - (target.x - initial_pos.x) * actual_pos.y + (target.x * initial_pos.y) - (target.y * initial_pos.x))
      down = math.sqrt(pow(target.y - initial_pos.y, 2) + pow(target.x - initial_pos.x, 2))
      return float(up) / float(down)

   def get_distance_to_goal(self, target=Point(), robot_pos=Point()):
      delta_x = target.x - robot_pos.x
      delta_y = target.y - robot_pos.y
      distance = np.sqrt((delta_x)**2+(delta_y)**2) 
      return distance

   #?# ********** CLEAN #?#**********
   def done(self):
      vel_msg = Twist()
      vel_msg.linear.x = 0.0
      vel_msg.angular.z = 0.0
      self.cmd_vel_pub.publish(vel_msg)

   def cleanup(self):  
      '''This function is called just before finishing the node.'''
      self.gtg_topic.publish(False)
      self.fw_topic.publish(False)
      self.gtg_active = False
      self.fw_active = False
      #* Detener el robot
      self.done()
      print("Finish Message!!!")  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
   rospy.init_node('BUG2_NODE') # Node Name
   try: Bug2()
   except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")