#!/usr/bin/env python  
import rospy  
from fiducial_msgs.msg import FiducialTransformArray
import numpy as np
from geometry_msgs.msg import Point

class Fiducial():  
  '''
      Clase que obtiene trasformaciones y error de fiducial
  '''
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    ###******* INIT PUBLISHERS *******###  
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.get_fiducial)
    self.fiducial_pub = rospy.Publisher("info_fiducial", Point, queue_size=1)
    ###******* INIT CONSTANTS/VARIABLES *******###  
    
    self.fiducial_received = False
    self.fiducial_data = []
    self.POS_ARUCOS = {
       "701": (0.48,3.15), 
       "702": (2.29,2.85),
       "703": (1.04,4.65),
       "704": (1.43,2.45),
       "705": (1.20,0.98),
    }

    rate = rospy.Rate(50) # The rate of the while loop will be 50Hz 
    rospy.loginfo("Starting Message!")     
    
    ###******* PROGRAM BODY *******###  
    while not rospy.is_shutdown(): 
      if self.fiducial_received is True:
         dis = 3
         for fiducial in self.fiducial_data:
            aruco_diff = np.array([
              fiducial.transform.translation.z + 0.09, 
              - fiducial.transform.translation.x
            ])
            distancia_aruco = np.sqrt(aruco_diff[0]**2 + aruco_diff[1]**2)
            if dis > distancia_aruco:
               dis = distancia_aruco
               msg = Point()
               msg.x = fiducial.transform.translation.z + 0.09
               msg.y = -fiducial.transform.translation.x
               msg.z =  fiducial.object_error  # Ruido de la medicion de los arucos
               self.fiducial_pub.publish(msg)
      rate.sleep() 

  def get_fiducial(self, msg_array = FiducialTransformArray()):
     need_fiducial_array = []
     if self.fiducial_received is False:
        for fiducial in msg_array.transforms:
           if str(fiducial.fiducial_id) in self.POS_ARUCOS:
              self.fiducial_received = True
              need_fiducial_array.append(fiducial)
        self.fiducial_data = need_fiducial_array
    
  def cleanup(self):  
      '''This function is called just before finishing the node.'''
      print("Finish Message!!!")  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('Fiducial') # Node Name
    try: Fiducial()  # Class Name
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")