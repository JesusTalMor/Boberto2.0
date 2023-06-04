#!/usr/bin/env python  
import rospy  
import numpy as np
from geometry_msgs.msg import Point, Twist
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
      rospy.Subscriber('position', Point, self.get_odom)
      self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 
      self.gtg_topic = rospy.Publisher('gtg_topic', Bool, queue_size=1)
      self.fw_topic = rospy.Publisher('fw_topic', Bool, queue_size=1)    
      self.goal_pub = rospy.Publisher('GOAL', Point, queue_size=1)

      #?#******* INIT CONSTANTS/VARIABLES *******#?#  
      # Posicion del Robot
      self.robot_pos = Point()
      self.odom_received = False

      # Definicion de punto inicial y goal
      self.initial_pos = Point()
      GOAL_ARRAY = [
         (0.51, 2.08),
         (2.39, 4.77),
         (1.79, 3.57),
         (1.53, 1.30),
         (1.42, 0.21)
      ]

      GOAL_INDX = 0
      self.target = Point()
      self.target.x, self.target.y = GOAL_ARRAY[GOAL_INDX]
      self.goal_received = True

      # Banderas de estados
      self.lidar_data = LaserScan()
      self.region_received = False
      self.gtg_active = False
      self.fw_active = False

      # Definicion de estados
      self.current_state = "GTG"
      states = {
         "GTG" : "GO_TO_GOAL",
         "FW" : "WALL_FOLLOWER",
         "S" : "STOP" 
      }

      
      # Variables
      progress = 0.1 # Para verificar si el robot avanzo esta distancia antes de cambiar de estado
      tolerance = 0.1 # 0.05 Si el robot esta asi de cerca de la linea con respecto a cuando cambio a FW, cambiara a gtg
      self.d_t = 0.0
      self.D_Fw = 0.0
      goal_tolerance = 0.02

      rate = rospy.Rate(10) # The rate of the while loop will be 50Hz 
      rospy.loginfo("Starting Message!")     

      ###******* PROGRAM BODY *******###  
      while not rospy.is_shutdown(): 
         # El programa jala hasta que reciba un GOAL
         if self.goal_received is False:
            rospy.logwarn("WAIT GOAL")
            self.gtg_topic.publish(False)
            self.fw_topic.publish(False)
            self.gtg_active = False
            self.fw_active = False
            self.stop_robot()
            if GOAL_INDX + 1 < len(GOAL_ARRAY):
               GOAL_INDX += 1
               self.target.x, self.target.y = GOAL_ARRAY[GOAL_INDX]
               self.goal_received = True
            else:
               rospy.logwarn("RUTINA COMPLETA")
            rate.sleep()
            continue

         if self.odom_received is False:
            # rospy.logwarn("NO ODOM")
            rate.sleep()
            continue
         
         # Espera a que reciba regiones del lidar
         if self.region_received is False:
            rate.sleep()
            continue

         # * Si ambos estados estan en Falso iniciar con GTG
         if self.gtg_active is False and self.fw_active is False:
            self.current_state = "GTG"
            self.initial_pos = self.robot_pos
            self.gtg_topic.publish(True)
            self.fw_topic.publish(False)  
            self.gtg_active = True
            self.fw_active = False

         self.goal_pub.publish(self.target)
         # Calcula la distancia del robot a la linea
         distance_line = self.get_distance_to_line(self.target, self.initial_pos, self.robot_pos)

         # Obtener objeto mas cercano
         closest_dist, closest_angle = self.get_closet_object(self.lidar_data)

         # Calcula el angulo para evitar obstaculo
         thetaAO = self.get_theta_ao(closest_angle)

         # Calcula el angulo para rodear obstaculo 
         thetaGTG = self.get_theta_gtg(self.target, self.robot_pos) 

         # Distancia al goal
         self.d_t = self.get_distance_to_goal(self.target, self.robot_pos)

         # Calcular Angulo de Clear SHOT
         theta_clear_shot = np.abs(self.limit_angle(thetaAO-thetaGTG))

         FW_progress = self.D_Fw - self.d_t
         
         #?#********** MAQUINA DE ESTADOS **********#?#
         rospy.logerr("CURRENT STATE: " + str(self.current_state))
         rospy.loginfo("CURRENT GOAL: " + str(self.target.x) + "," + str(self.target.y))
         rospy.loginfo("----------------------------------------------")
         rospy.loginfo("DISTANCE TO GOAL: " + str(round(self.d_t, 2)))
         rospy.loginfo("FW PROGRESS: " + str(round(FW_progress,2)) + " | " + str(FW_progress >= progress))
         rospy.loginfo("CLEASHOT ANGLE: " + str(self.RadToDeg(theta_clear_shot)) + " | " + str(theta_clear_shot < np.pi/2.0))
         rospy.loginfo("DISTANCE LINE: " + str(round(distance_line,2)) + " | " + str(distance_line < tolerance))
         rospy.loginfo("----------------------------------------------\n\n")
         
         if self.d_t <= goal_tolerance:
            rospy.logwarn("GOAL REACHED")
            self.change_state("STOP")
         
         elif self.current_state == "GTG":
            # Si hay un obstaculo, cambia a comportamiento de FW
            if closest_dist <= 0.40:
               rospy.logwarn("WALL DETECTED - CHANGE TO FW")
               self.change_state("FW")
         
         elif self.current_state == "FW":
            #       Ya avanzo una distancia ?     Tiene obstaculos a la vista ?     Esta cerca de la ruta ?
            if FW_progress >= progress and theta_clear_shot < np.pi/2.0 and distance_line < tolerance:
               rospy.logwarn("CLEAR SHOT - CHANGE TO GTG")
               self.change_state("GTG")

         self.odom_received = False
         self.region_received = False
         rate.sleep() 

   #?#********** LIMITADORES #?#**********
   def limit_angle(self, angle):
      return np.arctan2(np.sin(angle), np.cos(angle))
   
   def RadToDeg(self, angle):
      return round(angle*180.0/np.pi, 2)

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
         self.stop_robot()
         #* Preparar para recibir nueva GOAL
         self.goal_received = False
         self.initial_pos = self.robot_pos
      
      elif self.current_state == "GTG":
         self.gtg_topic.publish(True)
         self.fw_topic.publish(False)
         self.gtg_active = True
         self.fw_active = False
         rospy.sleep(5)
      
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

   def get_odom(self, msg=Point()):
      self.robot_pos = msg
      self.odom_received = True

   def get_goal(self, msg=Point()):
      self.target = msg
      self.goal_received = True
   
   #?# ********** GETTERS **********#?#
   def get_closet_object(self, lidar_data=LaserScan()):
      # Para delimitar region de al frente del robot
      idx_low = 360
      idx_high = 788
      Front_data = lidar_data.ranges[idx_low:idx_high]
      # Encontrar distancia mas cercana
      Front = min(min(Front_data), 10)

      # Para obtener closest angle
      min_idx = np.argmin(lidar_data.ranges)
      closest_angle = lidar_data.angle_min + min_idx * lidar_data.angle_increment
      # limitar el angulo
      closest_angle = self.limit_angle(closest_angle)
      return Front, closest_angle

   def get_theta_ao(self, theta_closest): 
      ##This function returns the angle for the Avoid obstacle behavior  
      # theta_closest is the angle to the closest object [rad] 
      #This functions returns the angle for the Avoid obstacle behavior [rad] 
      ############################################################ 
      thetaAO= theta_closest-np.pi 
      #limit the angle to [-pi,pi] 
      thetaAO = self.limit_angle(thetaAO)
      return thetaAO

   def get_theta_gtg(self, target_pos=Point(), robot_pos=Point()): 
      """ This function returns the angle to the goal  """
      # Desempaquetar variables
      y_target, x_target = target_pos.y, target_pos.x
      y_robot, x_robot, theta_robot = robot_pos.y, robot_pos.x, robot_pos.z
      theta_target=np.arctan2(y_target - y_robot, x_target - x_robot) 
      e_theta= theta_target - theta_robot 
      #limit e_theta from -pi to pi 
      #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi 
      e_theta = self.limit_angle(e_theta)
      return e_theta
   
   def get_distance_to_line(self, target=Point(), initial_pos=Point(), actual_pos=Point()):
      up = math.fabs((target.y - initial_pos.y) * actual_pos.x - (target.x - initial_pos.x) * actual_pos.y + (target.x * initial_pos.y) - (target.y * initial_pos.x))
      down = math.sqrt(pow(target.y - initial_pos.y, 2) + pow(target.x - initial_pos.x, 2))
      try:
         return float(up) / float(down)
      except:
         return 0.0

   def get_distance_to_goal(self, target=Point(), robot_pos=Point()):
      delta_x = target.x - robot_pos.x
      delta_y = target.y - robot_pos.y
      distance = np.sqrt((delta_x)**2+(delta_y)**2) 
      return distance

   #?# ********** CLEAN #?#**********
   def stop_robot(self):
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
      self.stop_robot()
      print("Finish Message!!!")  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
   rospy.init_node('BUG2_NODE') # Node Name
   try: Bug2()
   except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")