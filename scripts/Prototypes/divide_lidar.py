#!/usr/bin/env python  
import rospy  
from sensor_msgs.msg import LaserScan

class divide_lidar():  
  '''
    Tonteria de Template
  '''
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    ###******* INIT PUBLISHERS *******###  
    #? Exmaple: self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 
    rospy.Subscriber("base_scan", LaserScan, self.get_lidar_cb)

    ###******* INIT CONSTANTS/VARIABLES *******###  
    self.lidar_received = False

    rate = rospy.Rate(50) # The rate of the while loop will be 50Hz 
    rospy.loginfo("Starting Message!")     
    # Grab possible simulation Error
    # while rospy.get_time() == 0: 
    #   print("no simulated time has been received yet") 
    # start_time = rospy.get_time()  #Get the current time in float seconds 
    ###******* PROGRAM BODY *******###  
    while not rospy.is_shutdown(): 
      if self.lidar_received is True:
        #  print(self.lidar_msg.ranges[0])

         L,FL,F,FR,R = self.divide()

         min_L = min(L)
         min_FL = min(FL)
         min_F = min(F)
         min_FR = min(FR)
         min_R = min(R)

         if min_L <= 0.3:
            print("L")
         elif min_FL <= 0.3:
            print("FL")
         elif min_F <= 0.3:
            print("F")
         elif min_FR <= 0.3:
            print("FR")
         elif min_R <= 0.3:
            print("R")
         else:
            print("nadaaaaaaaaaa")
      rate.sleep() 

  def get_lidar_cb(self, msg):
     self.lidar_msg = msg
    #  print("lidar info received")
     self.lidar_received = True
  
  def divide(self):
     aux = self.lidar_msg.ranges
     R = aux[215:359]
     FR = aux[360:502]
     F = aux[503:645]
     FL = aux[646:788]
     L = aux[789:931]   

     return L, FL, F, FR, R

    
  def cleanup(self):  
      '''This function is called just before finishing the node.'''
      print("Finish Message!!!")  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('lidar_divide   ') # Node Name
    try: divide_lidar()  # Class Name
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")