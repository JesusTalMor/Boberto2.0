#!/usr/bin/env python  
import rospy  
from nav_msgs.msg import Odometry 
from std_msgs.msg import Float32 
from tf.transformations import quaternion_from_euler 

import numpy as np
#? Funcionalidad para imprimir matrices en terminal usando Numpy
np.set_printoptions(suppress=True) 
np.set_printoptions(formatter={'float': '{: 0.4f}'.format}) 

class LocalizationClass():  
  def __init__(self):  
    #?#********** PUBLICADORES **********###  
    self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1) 
    #?#********** SUSCRIPTORES **********###  
    rospy.Subscriber("wl", Float32, self.wl_cb ) 
    rospy.Subscriber("wr", Float32, self.wr_cb ) 

    odom = Odometry() # Mensaje de odometria para RVIZ
    #?#********** ROBOT CONSTANTS **********###  
    r=0.05 #[m] Radio de las llantas
    L=0.18 #[m] Separacion entre llantas 
      
    #?#********** ROBOT STATE **********### 
    self.wl = 0.0 # Velocidad angular llanta izquierda [rad/s]
    self.wr = 0.0 # Velocidad angular llanta derecha [rad/s]
    x=0.0     # Posicion del robot en X [m]
    y = 0.0   # Posicion del robot en Y [m]
    theta = 0.0   # Orientacion del robot en Z [rad]
    dt = 0.0  # Intervalo de tiempo
    v=0.0 # Velocidad lineal en eje X [m/s]
    w=0.0 # Velocidad angular en eje Z 

    #?#********** MATRIX COVARIAZNA **********### 
    covar_matrix = np.zeros([3,3])

    #?#********** GANANCIAS COVARIANZA **********### 
    wr_ruido = 1
    wl_ruido = 1

    #? Esperar hasta que recibamos un tiempo de simulacion
    while rospy.get_time() == 0:  print("Sin tiempo de simulacion recibido") 
    print("Tiempo detectado. Iniciando Proceso") 
    init_time = rospy.get_time() # Tiempo inicio simulacion
    # Banderas para recibir informacion
    self.received_wl = False
    self.received_wr = False 
    rate = rospy.Rate(20) # The rate of the while loop 
    while not rospy.is_shutdown(): 
      #! Si no se recibe alguna lectura de las ruedas
      if not self.received_wl or not self.received_wr:
        rate.sleep() 
        continue
      
      #* Tiempo de muestreo
      current_time = rospy.get_time() 
      dt = current_time - init_time
      
      #* Hacer copia de variables para evitar problemas
      wr = self.wr 
      wl = self.wl 
      
      #* Calcular velocidades del robot  
      v= r*(wr+wl)/2.0  # Lineal en X 
      w = r*(wr-wl)/L   # Angular en Z 

      #?#********** CALCULAR MATRIX HACOBIANO **********###
      comp_h_1 = - dt * v * np.sin(theta)
      comp_h_2 = dt * v * np.cos(theta)
      H = np.array([
        [1, 0, comp_h_1],
        [0, 1, comp_h_2],
        [0, 0, 1]
      ])

      #?#********** CALCULAR MATRIX RUIDO **********### 
      #* Calcular Matrix sigma_ruido
      sigma_ruido = np.array([
        [(wr_ruido * abs(wr)), 0],
        [0, (wl_ruido * abs(wl))]
      ])

      #* Calcular Hacobiano para ruido
      H_ruido = np.array([
        [np.cos(theta), np.cos(theta)],
        [np.sin(theta), np.sin(theta)],
        [2/L, -2/L]
      ])
      
      H_ruido = H_ruido * 0.5 * r * dt 
      # print("Matriz de Hacobo Ruido")
      # print(H_ruido)

      #* Calcular matrix Ruido Q
      Q = H_ruido.dot(sigma_ruido).dot(H_ruido.T)

      #?#********** CALCULAR MATRIX COVARIANZA **********### 
      covar_matrix = (H.dot(covar_matrix).dot(H.T)) + Q
      # print("Mostrar Matriz de Hacobiano")
      # print(H)
      # print("Mostrar Matriz de Ruido")
      # print(Q)
      # print("Mostar Matriz de Covarianza")
      # print(covar_matrix)

      #?#********** ACTUALIZAR POSICION **********### 
      x=x + (v*np.cos(theta)*dt)
      y=y + (v*np.sin(theta)*dt)
      theta = theta + w*dt 
      #* Cortar than en rangos en -pi a pi
      theta =np.arctan2(np.sin(theta), np.cos(theta))

      #* Actualizar valor de tiempo
      init_time = current_time 

      odom = self.fill_odom(x, y, theta, covar_matrix, v, w) 
      self.odom_pub.publish(odom) 
      rate.sleep() 

  def wl_cb(self, msg): 
      self.wl = msg.data 
      self.received_wl = 1 

  def wr_cb(self, msg): 
      self.wr = msg.data 
      self.received_wr = 1 

  def fill_odom(self,x, y, theta, Sigma_pose, v, w): 
      # (x,y) -> robot position 
      # theta -> robot orientation 
      # Sigma_pose -> 3x3 pose covariance matrix 
      odom=Odometry() 
      odom.header.stamp =rospy.Time.now() 
      odom.header.frame_id = "odom"
      odom.child_frame_id = "chassis" 
      odom.pose.pose.position.x = x 
      odom.pose.pose.position.y = y 
      odom.pose.pose.position.z = 0.0 
      quat=quaternion_from_euler(0.0, 0.0, theta) 
      odom.pose.pose.orientation.x = quat[0] 
      odom.pose.pose.orientation.y = quat[1] 
      odom.pose.pose.orientation.z = quat[2] 
      odom.pose.pose.orientation.w = quat[3] 
      odom.pose.covariance = [0.0]*36 
      # Fill the covariance matrix 
      odom.pose.covariance[0] = Sigma_pose[0,0] 
      odom.pose.covariance[1] = Sigma_pose[0,1] 
      odom.pose.covariance[5] = Sigma_pose[0,2] 
      odom.pose.covariance[6] = Sigma_pose[1,0] 
      odom.pose.covariance[7] = Sigma_pose[1,1] 
      odom.pose.covariance[11] = Sigma_pose[1,2] 
      odom.pose.covariance[30] = Sigma_pose[2,0] 
      odom.pose.covariance[31] = Sigma_pose[2,1] 
      odom.pose.covariance[35] = Sigma_pose[2,2] 
      odom.twist.twist.linear.x = v 
      odom.twist.twist.angular.z = w 
      return odom 
############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node('localization')  
    LocalizationClass()  