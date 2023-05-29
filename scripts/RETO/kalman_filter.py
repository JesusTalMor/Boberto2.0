#!/usr/bin/env python  
import rospy  
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker 


import numpy as np

#?#********** Variables Globales **********###
NUMVAR = 3
ix, iy, itheta = 0, 1, 2
r = 0.05 #[m] Radio de las llantas
L = 0.18 #[m] Separacion entre llantas 


class KalmanFilter:
  """ Clase para implementar una fusion de datos usando el Filtro de Kalman
  """
  def __init__(self):
    """ Iniciar Filtro en Valores Base 
      Por defecto todos los valores son 0,0 de inicio"""
    # Vector de Estados a Manejar 3x1
    self._x = np.zeros(NUMVAR) 
    self._x[ix] = 0
    self._x[iy] = 0
    self._x[itheta] = 0

    # Matriz de Covarianza 3x3 Inicial en zeros
    self._P = np.zeros((NUMVAR,NUMVAR))
  
  def predict(self, w_ruedas, w_error, dt):
    """ Realiza una prediccion de la posicion del Robot.
    
    Parametros
    ----------
    w_ruedas : list [wr, wl] 
      velocidad angular de las llantas del robot
    w_error : list [wr_e, wl_e] 
      Error de mediciones angulares de las llantas
    """
    #* Desempaquetar las variables
    wr, wl = w_ruedas
    wr_e, wl_e = w_error
    v = r * (wr+wl) / 2.0 # Velocidad Eje X m/s
    w = r * (wr-wl) / L # Velocidad Eje Z rad/s
    cos_theta = np.cos(self._x[itheta])
    sin_theta = np.sin(self._x[itheta])

    #?#********** CALCULAR MATRIX HACOBIANO **********###
    comp_h_1 = - dt * v * sin_theta
    comp_h_2 = dt * v * cos_theta
    H = np.array([
      [1, 0, comp_h_1],
      [0, 1, comp_h_2],
      [0, 0, 1]
    ])

    #?#********** CALCULAR MATRIX RUIDO **********### 
    #* Calcular Matrix sigma_ruido
    sigma_ruido = np.array([
      [(wr_e * abs(wr)), 0],
      [0, (wl_e * abs(wl))]
    ])

    #* Calcular Hacobiano para ruido
    H_ruido = np.array([
      [cos_theta, cos_theta],
      [sin_theta, sin_theta],
      [2/L, -2/L]
    ])
    
    H_ruido = H_ruido * 0.5 * r * dt 

    #* Calcular matrix Ruido Q
    Q = H_ruido.dot(sigma_ruido).dot(H_ruido.T)

    #?#********** CALCULAR MATRIX COVARIANZA **********### 
    self._P = (H.dot(self._P).dot(H.T)) + Q

    #?#********** ESTIMAR NUEVAS POSICIONES **********### 
    self._x[ix] += v*cos_theta*dt
    self._x[iy] += v*sin_theta*dt
    theta = self._x[itheta] + w*dt
    theta = np.arctan2(np.sin(theta), np.cos(theta))
    self._x[itheta] = theta
  
  def update(self, aruco_coord, aruco_noise, aruco_diff):
    """ Actualizar las Predicciones del Filtro de Kalman 
    
    Parametros
    ----------
      aruco_coord : (list)
        Una lista de 2x1 con las coordenadas X, Y de la posicion del aruco
      aruco_noise : (list)
        Lista 2x2 con los ruidos 'Aparentes' en la mediciones de la posicion del aruco
      aruco_diff : (list)
        Valores de Fiducial Transform, Con X, Z y rotacion en Y
    """
    #?#********** CALCULAR MATRIX HACOBIANO **********###
    delta_x = aruco_coord[ix] - self._x[ix]
    delta_y = aruco_coord[iy] - self._x[iy]
    phi = delta_x**2 + delta_y**2
    # Forma Matriz 2x3, 
    H = np.array([ 
      [-delta_x/np.sqrt(phi), -delta_y/np.sqrt(phi), 0],
      [delta_y/np.sqrt(phi), delta_x/np.sqrt(phi), -1]
    ])
    
    #?#********** CALCULAR MATRIZ Z **********###
    Rk = np.array([
      [aruco_noise, 0],
      [0, aruco_noise]
    ])
    # 2x3 * 3x3 = 2x3 * 3x2 = 2x2 + 2x2
    Z = H.dot(self._P).dot(H.T) + Rk

    #?#********** CALCULAR GANANCIA DE KALMAN **********###
    # 3x3 * 3x2 = 3x2 * 2x2 = 3x2
    K = self._P.dot(H.T).dot(np.linalg.inv(Z))

    #?#********** ACTUALIZAR COVARIANZA KALMAN **********###
    # 3x3 - 3x2 * 2x3 = 3x3 * 3x3 = 3x3  Correcto
    self._P = (np.eye(NUMVAR) - (K.dot(H))).dot(self._P)

    #?#********** ACTUALIZAR POSICIONES KALMAN **********###
    componente_phi = np.sqrt(delta_x**2 + delta_y**2) 
    componente_alpha = np.arctan2(delta_y,delta_x) - self._x[itheta] 
    observacion_estimada = np.array([componente_phi, componente_alpha])
    componente_phi = np.sqrt(aruco_diff[ix]**2 + aruco_diff[iy]**2) 
    componente_alpha = np.arctan2(aruco_diff[iy],aruco_diff[ix]) - aruco_diff[itheta]
    # componente_alpha = np.arctan2(np.sin(aruco_diff[itheta]), np.cos(aruco_diff[itheta]))
    observacion_aruco = np.array([componente_phi, componente_alpha])
    # 3x1 + 3x2 * 2x1 = 3x1
    self._x = self._x + K.dot(observacion_aruco - observacion_estimada)
    #* Limitar theta
    theta = np.arctan2(np.sin(self._x[itheta]), np.cos(self._x[itheta]))
    self._x[itheta] = theta

  @property
  def medidas(self):
    return self._x
  
  @property
  def covarianza(self):
    return self._P

class KFNode:  
  """ Nodo para el manejo de Odometria usando Filtro de Kalman"""
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    #?#********** INIT PUBLISHERS #?#**********
    self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1) 
    self.marker_pub = rospy.Publisher("perfect_marker", Marker, queue_size=1)    
    #?#********** SUSCRIPTORES **********###  
    rospy.Subscriber("wl", Float32, self.wl_cb ) 
    rospy.Subscriber("wr", Float32, self.wr_cb ) 
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.get_fiducial)
    odom = Odometry() # Mensaje de odometria
    KF = KalmanFilter() # Iniciar filtro de Kalaman

    ###******* INIT CONSTANTS/VARIABLES *******###  
    rate = rospy.Rate(10) # The rate of the while loop will be 50Hz 
    self.wl = 0.0
    self.wr = 0.0
    self.received_wl = False
    self.received_wr = False
    self.fiducial_received = False
    self.fiducial_data = []
    #? TODOS LOS ID de los ARUCOS y sus coordenadas en el mundo
    # self.POS_ARUCOS = {
    #   "701": (0.48,3.15), 
    #   "702": (2.29,2.85),
    #   "703": (1.04,4.65),
    #   "704": (1.43,2.45),
    #   "705": (1.20,0.98),
    # }
    self.POS_ARUCOS = {
      "701" : (1.79, 1.19),
      "712" : (2.98, -1.19),
    }
    v = 0.0
    w = 0.0
    rospy.loginfo("Starting Message!")     
    # Grab possible simulation Error
    while rospy.get_time() == 0: print("no simulated time has been received yet") 
    init_time = rospy.get_time()  #Get the current time in float seconds 
    ###******* PROGRAM BODY *******###  
    while not rospy.is_shutdown(): 
      #? Siempre vamos a estar realizando predicciones con kalman
      #* Si tenemos datos de las llantas hacemos una prediccion
      if self.received_wl is True and self.received_wr is True:     
        #* Tiempo de muestreo
        current_time = rospy.get_time() 
        dt = current_time - init_time

        #* Hacer copia de variables wl y wr
        wr = self.wr
        wl = self.wl

        v = r * (wr+wl) / 2.0 # Velocidad Eje X m/s
        w = r * (wr-wl) / L # Velocidad Eje Z rad/s
        KF.predict([wr,wl],[0.087, 0.087],dt)

        #* Actualizar valor de tiempo
        init_time = current_time
        self.received_wl = False
        self.received_wr = False
      
        #* Si detectamos un ARUCO, realizamos una actualizacion de Kalman
        if self.fiducial_received is True:
          #* Por cada Aruco detectado se hace un update
          for fiducial in self.fiducial_data:
            # prueba = FiducialTransform()
            # prueba.object_error
            roll, pitch, yaw = euler_from_quaternion([
              fiducial.transform.rotation.x,
              fiducial.transform.rotation.y,
              fiducial.transform.rotation.z,
              fiducial.transform.rotation.w,
            ])
            aruco_pos = self.POS_ARUCOS[str(fiducial.fiducial_id)]
            aruco_noise = fiducial.object_error # Ruido de la medicion de los arucos
            aruco_diff = [
              fiducial.transform.translation.z,
              fiducial.transform.translation.x,
              pitch
            ]
            distancia_aruco = np.sqrt(aruco_diff[ix]**2 + aruco_diff[iy]**2) 
            if distancia_aruco < 0.8:
              rospy.logwarn("Aruco Detected: Updating Position")
              KF.update(aruco_pos, aruco_noise, aruco_diff)
          
          self.fiducial_received = False
      
      #* Sacamos los datos del filtro de Kalman
      x = KF.medidas[ix]
      y = KF.medidas[iy]
      theta = KF.medidas[itheta]
      posicion = [round(x,2), round(y,2), round(theta,2)]
      rospy.loginfo("Posicion Estimada: ")
      rospy.loginfo(posicion)
      covarianza = KF.covarianza

      odom = self.fill_odom(x, y, theta, covarianza, v, w)
      marker = self.fill_marker_perfect(odom)
      self.odom_pub.publish(odom)  
      self.marker_pub.publish(marker)
      rate.sleep()

  def wl_cb(self, msg): 
      self.wl = msg.data 
      self.received_wl = True

  def wr_cb(self, msg): 
      self.wr = msg.data 
      self.received_wr = True
  
  def get_fiducial(self, msg_array=FiducialTransformArray()):
    need_fiducial_array = []
    if self.fiducial_received is False:
      for fiducial in msg_array.transforms:
        if str(fiducial.fiducial_id) in self.POS_ARUCOS:
          self.fiducial_received = True
          need_fiducial_array.append(fiducial)
      self.fiducial_data = need_fiducial_array

  def fill_odom(self,x, y, theta, covarianza, v, w): 
      # (x,y) -> robot position 
      # theta -> robot orientation 
      # Sigma_pose -> 3x3 pose covariance matrix 
      odom=Odometry() 
      odom.header.stamp =rospy.Time.now() 
      odom.header.frame_id = "odom"  # "odom" 
      odom.child_frame_id = "base_link" #"chassis" 
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
      odom.pose.covariance[0] = covarianza[0,0] 
      odom.pose.covariance[1] = covarianza[0,1] 
      odom.pose.covariance[5] = covarianza[0,2] 
      odom.pose.covariance[6] = covarianza[1,0] 
      odom.pose.covariance[7] = covarianza[1,1] 
      odom.pose.covariance[11] = covarianza[1,2] 
      odom.pose.covariance[30] = covarianza[2,0] 
      odom.pose.covariance[31] = covarianza[2,1] 
      odom.pose.covariance[35] = covarianza[2,2] 
      odom.twist.twist.linear.x = v 
      odom.twist.twist.angular.z = w 
      return odom 

  def fill_marker_perfect(self, odom = Odometry()):
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()

    marker.type = 10
    marker.id = 1

    marker.mesh_resource = "package://puzzlebot_sim/descriptions/MCR2_1000_0_Puzzlebot.stl" 

    # Set the scale of the marker 
    marker.scale.x = 1 
    marker.scale.y = 1 
    marker.scale.z = 1 

    # Set the color 
    marker.color.r = 1.0 
    marker.color.g = 0.0 
    marker.color.b = 1.0 
    marker.color.a = 1.0     

        # Set the pose of the marker 
    marker.pose.position.x = odom.pose.pose.position.x
    marker.pose.position.y = odom.pose.pose.position.y
    marker.pose.position.z = 0.0 

    # Set the marker orientation 
    #quat=quaternion_from_euler(0.0,0.0,theta) 
    marker.pose.orientation.x = odom.pose.pose.orientation.x
    marker.pose.orientation.y = odom.pose.pose.orientation.y
    marker.pose.orientation.z = odom.pose.pose.orientation.z
    marker.pose.orientation.w = odom.pose.pose.orientation.w

    return marker
  
  def cleanup(self):  
      '''This function is called just before finishing the node.'''
      print("Finish Message!!!")  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('KALMANFILTER') # Node Name
    try: KFNode()  # Class Name
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")