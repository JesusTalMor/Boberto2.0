#!/usr/bin/env python  
import rospy  
import numpy as np
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool

class GoToGoal():  
  """ Clase para implementar un GO TO GOAL
  """
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    #?#********** PUBLICADORES Y SUBSCRIPTORES #?#********** 
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 
    rospy.Subscriber('position', Point, self.get_odom) # Comes From KalmanFilter
    rospy.Subscriber('gtg_topic', Bool, self.gtg_Switch)
    rospy.Subscriber('GOAL',Point, self.get_goal)
  
    #?#********** CONSTANTES / VARIABLES #?#********** 
    # Posicion del Robot
    self.robot_pos = Point()
    self.odom_received = False

    self.active = True
    self.current_state = "FIX"
    #? ESTADOS POSIBLES DEL PROGRAMA
    states = {
      "FIX" : "FIX_ANGLE",
      "GO" : "GO_STRAIGHT",
      "HERE" : "ON_GOAL",
      "STOP" : "STOP"
    }
    GOALS = [(1.2, 1.2), (1.2, 2.4)]
    GOAL_IND = 0

    # Define goal point
    self.target = Point()
    self.target.x, self.target.y = GOALS[GOAL_IND]
    self.goal_received = True

    self.angle_precision = (np.pi/180.0) * 20.0 # Rango de error de 20 grados
    self.distance_precision = 0.025 # Tolerancia a llegar al Goal

    rate = rospy.Rate(10) # Realizar comportamiento cada 10 segundos
    rospy.loginfo("STARTING GO TO GOAL NODE")     
    #?#********** MAIN LOOP #?#********** 
    while not rospy.is_shutdown(): 
      # Nodo Apagado
      if self.active is False: 
        rospy.logfatal("NODE OFF")
        rate.sleep() 
        continue

      if self.goal_received is False: 
        rospy.logwarn("WAIT GOAL")
        self.current_state = "FIX"
        self.stop_robot()
        rate.sleep() 
        continue

      if self.odom_received is False:
        rate.sleep()
        continue

      # Calcular estatus de navegacion
      y_target = self.target.y
      x_target = self.target.x
      robot_theta = self.robot_pos.z
      thetaGTG = np.arctan2(y_target-self.robot_pos.y,x_target-self.robot_pos.x)
      error_theta = self.limit_angle(thetaGTG - robot_theta)
      error_dist = np.sqrt(pow(y_target-self.robot_pos.y,2)+pow(x_target-self.robot_pos.x,2))
      rospy.loginfo("REACHING GOAL: " + str(round(error_dist,4)))

      if error_dist <= self.distance_precision:
        rospy.logwarn("GOAL REACHED")
        # self.goal_received = False
        if GOAL_IND + 1 < len(GOALS): GOAL_IND += 1
        else: GOAL_IND = 0
        self.target.x ,self.target.y = GOALS[GOAL_IND]

      self.compute_GTG(error_theta, error_dist)
      rate.sleep()


  #?#********** CALLBACKS #?#********** 
  def get_odom(self, msg=Point()):
    self.robot_pos = msg
    self.odom_received = True

  def gtg_Switch(self,msg=Bool()):
    self.active = msg.data

  def get_goal(self, msg=Point()):
    self.target = msg
    self.goal_received = True

  #?#********** COMPORTAMIENTOS #?#********** 
  def stop_robot(self):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    self.cmd_vel_pub.publish(vel_msg)
  
  def compute_GTG(self, error_theta, error_dist):
      vel_msg = Twist()

      # * Si ya llegaste al objetivo detenerse
      if error_dist <= self.distance_precision:
        w = 0.0
        v = 0.0
      # * Si el error en theta es muy grande apuntar al goal
      elif np.abs(error_theta) > self.angle_precision:
        kwmax = 0.5 #angular angular speed maximum gain 
        aw = 1.75 #Constant to adjust the exponential's growth rate 
        
        kw = self.compute_gain(kwmax, aw, error_theta)

        w = kw*error_theta 
        v = 0.0
      #* Llegar al objetivo ajustando poco el angulo
      else:
        kvmax = 0.5 #linear speed maximum gain  
        kwmax = 0.65  #angular angular speed maximum gain 
        av = 0.5 #Constant to adjust the exponential's growth rate   
        aw = 1.75 #Constant to adjust the exponential's growth rate 

        #Compute the robot's angular speed 
        kw = self.compute_gain(kwmax, aw, error_theta)
        kv = self.compute_gain(kvmax, av, error_dist)
        
        w = kw*error_theta 
        v = kv*error_dist + 0.1
      
      w = self.limit_vel(w, 0.8)
      v = self.limit_vel(v, 0.8)
      
      vel_msg.angular.z = w
      vel_msg.linear.x = v

      self.cmd_vel_pub.publish(vel_msg)

  #?#********** HELPERS #?#********** 
  def limit_angle(self,angle):
    return np.arctan2(np.sin(angle),np.cos(angle))
  
  def limit_vel(self, vel, lim):
    if np.abs(vel) > lim: rospy.logwarn("VELOCITY LIMITED")
    sign = 1 if vel > 0.0 else -1
    vel = vel if np.abs(vel) <= lim else lim * sign
    return vel
  
  def compute_gain(self, Kmax=0.0, ak=0.0, error=0.0):
    """ Calcular la ganancia en base a un comportamiento exponencial
    __________
    
    Parametros
    ----------
      Kmax : float
        La ganancia maxima que puede obtener retornar
      ak : float
        comportamiento que tendra, la ganacia
        Mas grande = Mas Agresiva la exponencial
    """
    # Evitar problemas de nan
    if error == 0.0: return 0.0
    Kgain = Kmax*(1 - np.exp(-ak * error**2))/abs(error)
    return Kgain

  def cleanup(self):  
      '''This function is called just before finishing the node.'''
      print("FINISH MESSAGE STOPPING ROBOT")  
      self.stop_robot()

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('move_forward_some_time') # Node Name
    try: GoToGoal()  # Class Name
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")