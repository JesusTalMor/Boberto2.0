#!/usr/bin/env python  
import rospy  
from sensor_msgs.msg import LaserScan
import numpy as np

class TransformLidar():  
  def __init__(self):  
    rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
    
    #?#********** PUBLICADORES Y SUBSCRIPTORES #?#**********
    self.trans_lidar_pub = rospy.Publisher('base_scan', LaserScan, queue_size=1)
    rospy.Subscriber("scan", LaserScan, self.get_lidar_cb) 
    
    #?#********** CONSTANTES / VARIABLES #?#**********
    self.lidar_data = LaserScan()
    self.recieved_lidar = False
    rate = rospy.Rate(10) # The rate of the while loop will be 10Hz 
    rospy.loginfo("Starting Message!")     
    
    #! Grab possible simulation Error
    while rospy.get_time() == 0: rospy.logwarn("No simulated time received !!") 
    
    #?#********** CUERPO DEL PROGRAMA #?#**********
    while not rospy.is_shutdown(): 
      #* Si no se reciben datos del lidar no hacer nada
      if not self.recieved_lidar:
        rate.sleep() 
        continue
      
      new_lidar_data = self.transform_lidar(self.lidar_data)
      self.trans_lidar_pub.publish(new_lidar_data)
      self.recieved_lidar = False
      rate.sleep() 

  def transform_lidar(self, original_lidar=LaserScan()):
    """ Original Lidar is the LaserScan we want to transform 
    __________
    
    @return : new LaserScan with transformed data
    """
    new_lidar = original_lidar
    len_ranges  = len(original_lidar.ranges)
    new_ranges = [np.inf]*len_ranges # Init the new array with inf
    new_ranges = np.roll(original_lidar.ranges, len_ranges/2)
    #print(new_ranges)
    print("len",len(new_ranges))
    new_lidar.ranges = new_ranges
    new_lidar.header.frame_id = "base_link"
    return new_lidar

  def get_lidar_cb(self, msg=LaserScan()):
    self.lidar_data = msg
    self.recieved_lidar = True
    
  def cleanup(self):  
      '''This function is called just before finishing the node.
      '''
      print("Finish Message!!!")  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('transform_lidar_data') # Node Name
    try: TransformLidar()  # Class Name
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPLETED SUCCESFULLY")
