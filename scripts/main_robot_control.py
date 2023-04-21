#!/usr/bin/env python  
import rospy 
import os
from numpy import pi 
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class yisus_robot_control():  
  ''' Nodo para controlar un robot por medio de un topico de texto '''
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    #?#************ PUBLISHERS ************#  
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) 
    
    
    #?#************ SUBSCRIBERS ************#  
    rospy.Subscriber('string_command', String, self.get_command) 
    
    
    #?#************ CONSTANTS/VARIABLES ************#  
    #? Examplesperiod = 5.0
    self.control_command = ''
    self.command_flag = True
    periodo_ejecut = 5
    # Generar una bandera para cada actividad
    rate = rospy.Rate(50) # The rate of the while loop will be 50Hz 
    rospy.loginfo("Starting Message!")     
    
    #* Grab possible simulation Error
    while rospy.get_time() == 0: print("no simulated time has been received yet") 
    
    #?#************ PROGRAM BODY ************#  
    while not rospy.is_shutdown():
      myTwist = Twist() # Objeto vacio para el movimiento del robot
      
      
      #* Si no se recibe un commando no hacer nada 
      if not self.control_command:
        # os.system('clear')
        rospy.loginfo_once('Sin Comando. Estatus: Detenido')
        self.cmd_vel_pub.publish(myTwist)
        rate.sleep()
        continue
      
      
      #* Podemos recibir un nuevo comando ?
      if self.command_flag: 
        rospy.loginfo('Comando recibido: %s' %(self.control_command))
        start_time = rospy.get_time()  #Get the current time in float seconds 
        self.command_flag = False

      #* Tomar el tiempo de ejecucion
      ejecut_time = rospy.get_time() - start_time
      if ejecut_time >= periodo_ejecut and not self.command_flag:
        rospy.loginfo('Termino de comando: %s' %(self.control_command))  
        # Factor de tiempo
        self.command_flag = True
        self.control_command = ''
        periodo_ejecut = 5
      
      #* Avanzar recto 5 segundos. 1 m aprox
      if 'forward' in self.control_command: myTwist.linear.x = 0.2
      #* Va de reversa 5 segundos. 1 m aprox
      elif 'back' in self.control_command: myTwist.linear.x = -0.2
      #* Girar a la izquierda 5 segundos. 90 grados aprox
      elif 'left' in self.control_command: myTwist.angular.z = pi/10
      #* Girar a la derecha 5 segundos. 90 grados aprox
      elif 'right' in self.control_command: myTwist.angular.z = -pi/10
      #* Movimiento especial. Hacer un medio circulo para la izquierda y otro a la derecha
      elif 'special' in self.control_command:
        periodo_ejecut = 10
        sign_flag = 1 if ejecut_time < 5 else -1
        # Hace donas hacia la izquierda
        myTwist.linear.x = 0.1
        myTwist.angular.z = (pi/10) * sign_flag
      elif 'square' in self.control_command:
        periodo_ejecut = 4
        if ejecut_time <= 2:
          # Derecho por 2 seg
          myTwist.linear.x = 0.5
          myTwist.angular.z = 0.0
        else:
          # Girar 90 en 2 seg
          myTwist.angular.z = (pi/4.0)
          myTwist.linear.x = 0.0
      
      self.cmd_vel_pub.publish(myTwist)
      rate.sleep()
  
  # def move_forward(self, execution_time=5):
  #   """ Move the robot a forward for a determine time """
  #   rospy.loginfo("Moving forward!") 

  #?#************ CALLBACKS ************#  
  def get_command(self, command=String):
    """ Save string command from topic """
    if self.command_flag: self.control_command = command.data
    
  def cleanup(self):  
      '''This function is called just before finishing the node.'''
      stopTwist = Twist()
      self.cmd_vel_pub.publish(stopTwist)
      print("Finish Message!!!")  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('yisus_robot_control') # Node Name
    try: yisus_robot_control()  # Class Name
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")