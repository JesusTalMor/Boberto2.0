#!/usr/bin/env python 
from operator import truediv
import rospy 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class follow_wall():
  '''
    This class will make the robot follow some points in the space
  '''
  def __init__(self): 
    rospy.on_shutdown(self.cleanup) # Callback fuction when the node is turned off
    #?#*********** PUBLISHERS ***********
    self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    #?#*********** SUBSCRIBERS ***********
    rospy.Subscriber('base_scan', LaserScan, self.laser_cb)
    rospy.Subscriber('wl',Float32,self.wl_cb)
    rospy.Subscriber('wr',Float32,self.wr_cb)

    #?#*********** CONSTANTES CLASE ***********
    rate = 10
    Dt = 1/float(rate) # Dt is the time between one calculation and the next one
    vel_msg = Twist()
    goal_coords = [0,5]
    dx = 0
    dy = 0
    theta = 0
    #?#*********** VARIABLES CLASE ***********
    self.wr = 0
    self.wl = 0
    self.ranges_list = []
    self.angle_increment = 0.0
    self.min_angle = 0.0
    
    #?#*********** CICLO COMPLETO ***********
    r = rospy.Rate(rate) #1Hz 
    print("Node initialized " + str(rate)+ " hz")
    while not rospy.is_shutdown():
      #* Rangos detectados por el lidar
      main_ranges = self.ranges_list
      angle_increm = self.angle_increment
      start_angle = self.min_angle
      # * Calcular la posicion actual del robot
      dx, dy, theta = self.cal_pos(dx,dy,theta,Dt)
      
      #* Sacar el rango mas cercano al robot
      min_range = self.min_exclude(main_ranges)
      #* Calcular distancia al goal y angulo al goal
      e_theta2goal, e_d2goal = self.cal_error2goal(goal_coords,[dx,dy],theta)
      #* Calcular la theta para avoid obstacle
      atheta = self.cal_avoidTheta(main_ranges,angle_increm,start_angle, min_range)
      
      #? Seccion para sacar el coportamiento de following walls
      fwtheta_c = atheta - (np.pi/2)
      fwtheta_c = np.arctan2(fwtheta_c,fwtheta_c)
      
      print("Distancia mas cercana: " + str(np.round(min_range,2)))
      print("------------------")

      #?#*********** Programar comportamiento ***********
      vel = 0 # Velocidades siempre inicializadas en 0
      giro = 0

      #* Comportamiento de Go to Goal
      if min_range > 1:
        # dtao = 0
        # # print("Error Theta: " + str(np.round(e_theta2goal,2)))
        # # print("Error Distance: " + str(np.round(e_d2goal,2)))
        print("GoToGoal")
        # TODO Modificar a un comportamiento para corregir el angulo antes de mover el robot
        #* Checar el error en el angulo
        # Corregir primero el angulo 
        if abs(e_theta2goal) < 0.4: vel = e_d2goal
        else: giro = e_theta2goal
        
        #* Ya llego al objetivo
        if abs(e_d2goal) < 0.4: 
          print("Goal Reached") 
          vel = 0
          giro = 0
      
      #* Comportamiento para Following Wall
      elif min_range > 0.2:
        # TODO Implementar aqui el following wall
        # if abs(fwtheta_c - e_theta2goal) <= (np.pi/2):
        status = "Go Clockwise"
        # print(fwtheta_c) 
        vel = 0.1
        giro = fwtheta_c
        # else:
        #   #  abs(fwtheta_c - e_theta2goal) > (np.pi/2):
        #   status = "Go Counter Clockwise"
        #   fwtheta_cc = atheta + np.pi
        #   fwtheta_cc = np.arctan2(np.sin(fwtheta_cc),np.cos(fwtheta_cc))
        #   vel = 0.1
        #   giro = 10*(fwtheta_cc)
        print("Following Wall: " + status)
      #* Demasiado Cerca Detener el Robot
      else:
        print('STOP')
        vel = 0
        giro = 0

      # if min_range < 1.5:
      #   # if dtao == 0:
      #   #     dtao = e_d2goal
      #   #     print(dtao)
      #   # # Comportamiento de Follow Wall
      #   # if (e_d2goal < abs(dtao - 0.3)) and abs(atheta - e_theta2goal) < (np.pi/2):
      #   #     print("Clear Shot")
      #   #     if abs(e_d2goal) > 0.4: 
      #   #         vel = e_d2goal
      #   #         giro = e_theta2goal
      #   #     else:
      #   #         vel = 0
      #   #         giro = 0
      #   # else:
      #   if True:
      print("==================")
      vel, giro = self.limit_vel(vel,giro)
      vel_msg.linear.x = vel
      vel_msg.angular.z = giro
      self.pub_vel.publish(vel_msg)
      r.sleep()
  
  def wr_cb(self, num): self.wr = num.data
  def wl_cb(self, num): self.wl = num.data
  def laser_cb(self, laser_msg): 
    #cosa = LaserScan()
    self.ranges_list = laser_msg.ranges
    self.angle_increment = laser_msg.angle_increment
    self.min_angle = laser_msg.angle_min

  def cal_avoidTheta(self,main_ranges,angle_increm,start_angle, min_range):
    if min_range != 1000:
      min_index = main_ranges.index(min_range)
      theta = (angle_increm*min_index) + start_angle
      theta_def = np.arctan2(np.sin(theta), np.cos(theta))
      theta_avoid = theta_def + np.pi
      theta_avoid = np.arctan2(np.sin(theta_avoid), np.cos(theta_avoid))
      return theta_avoid
    else:
      return 0

  def cal_error2goal(self,main_goal,act_pos,theta):
    e_theta = (np.arctan2(main_goal[1] - act_pos[1], main_goal[0] - act_pos[0])) - theta
    e_d = np.sqrt(((main_goal[0] - act_pos[0])**2)+((main_goal[1] - act_pos[1])**2))
    return  e_theta, e_d

  def coord_maker(self, angle, dist):
    x = np.cos(angle)*dist
    y = np.sin(angle)*dist
    return x, y
  
  def pool_maker(self,x,y):
    theta = np.arctan2(y,x) 
    dist = np.sqrt(x**2 + y**2)
    return theta, dist

  def cal_vel(self):
    wRad = 0.05 # Wheel radius in Meters
    L = 0.19 # Wheel separation in Meters
    # Sacamos los objetivos de un arreglo ya definido
    v = (wRad*(self.wr + self.wl))/2
    w = (wRad*(self.wr - self.wl))/L # All in radians
    return v, w

  def cal_pos(self,dx,dy,theta,dt):
    '''
    Fuction to calculate the position in the space of the robot
    '''
    v, w = self.cal_vel()
    
    theta = (w*dt)+theta
    if theta > np.pi : theta = theta - (2*np.pi)
    elif theta < (-1*np.pi) : theta = theta + (2*np.pi)
    
    dx = dx + (v*(np.cos(theta))*dt)
    dy = dy + (v*(np.sin(theta))*dt)
    return dx, dy, theta
  
  def min_exclude(self, array):
    min_number = 1000
    for num in array:
      if num != 0 and num < min_number:
        min_number = num
    return min_number

  def limit_vel(self,vel,giro):
    '''
    Fuction that limits the velocities the robot can achieve
    '''
    if vel > 0.5: vel = 0.5
    if abs(giro) > 0.5: 
      if giro > 0: giro = 0.5
      else: giro = -0.5
    return vel, giro

  
  def go_stop(self):
    # Detener
    stop = Twist()
    self.pub_vel.publish(stop)
  
  def cleanup(self):
    self.go_stop()
    print("\n-----FINALIZADO-----\n")
############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
  rospy.init_node("follow_wall", anonymous=True) 
  try:
    follow_wall()
  except rospy.ROSInterruptException:
    print('\nEXECUTION COMPLETED SUCCESFULLY')