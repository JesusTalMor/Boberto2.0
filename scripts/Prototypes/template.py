#!/usr/bin/env python  
import rospy  

class TemplateClass():  
  '''
    Tonteria de Template
  '''
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    ###******* INIT PUBLISHERS *******###  
    #? Exmaple: self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 
    ###******* INIT CONSTANTS/VARIABLES *******###  
    #? Examplesperiod = 5.0
    rate = rospy.Rate(50) # The rate of the while loop will be 50Hz 
    rospy.loginfo("Starting Message!")     
    # Grab possible simulation Error
    # while rospy.get_time() == 0: 
    #   print("no simulated time has been received yet") 
    # start_time = rospy.get_time()  #Get the current time in float seconds 
    ###******* PROGRAM BODY *******###  
    while not rospy.is_shutdown(): 
      # * Whole Body of the Program
      rate.sleep() 
    
  def cleanup(self):  
      '''This function is called just before finishing the node.'''
      print("Finish Message!!!")  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('move_forward_some_time') # Node Name
    try: TemplateClass()  # Class Name
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")