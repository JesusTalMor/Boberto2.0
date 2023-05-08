#!/usr/bin/env python  
import rospy  
from sensor_msgs.msg import LaserScan
import numpy as np


class TansformLidar:
  def __init__(self):  
    #?#********** PUBLICADORES **********###  
    self.new_lidar_pub = rospy.Publisher("base_scan", LaserScan, queue_size=1) 

    #?#********** SUSCRIPTORES **********###  
    rospy.Subscriber("scan", LaserScan, self.lidar_cb ) 

    self.lidar = LaserScan()

    #? Esperar hasta que recibamos un tiempo de simulacion
    while rospy.get_time() == 0:  print("Sin tiempo de simulacion recibido") 
    print("Tiempo detectado. Iniciando Proceso") 
    init_time = rospy.get_time() # Tiempo inicio simulacion
    # Banderas para recibir informacion
    self.received_lidar = False
    rate = rospy.Rate(20) # The rate of the while loop 

    while not rospy.is_shutdown(): 
      if not self.received_lidar:
        rate.sleep() 
        continue

      new_lidar = self.transform_lidar(self.lidar)
      self.new_lidar_pub.publish(new_lidar)
      self.received_lidar = False
      rate.sleep() 

  def transform_lidar(sellf, original_lidar):
    #Original lidar is the lasercan we want to transorm
    # return new lidar (LaserScan) with the transformed data
    new_lidar = LaserScan()
    new_ranges = [np.inf]*len(original_lidar.ranges)
    new_ranges = np.roll(original_lidar.ranges, len(original_lidar.ranges)/2)

    new_lidar.angle_increment = original_lidar.angle_increment
    new_lidar.angle_max = original_lidar.angle_max
    new_lidar.angle_min =  original_lidar.angle_min
    new_lidar.header.frame_id = "base_link"
    new_lidar.ranges = new_ranges

    return new_lidar


  def lidar_cb(self, msg): 
      self.lidar = msg 
      self.received_lidar = 1 


############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node('localization')  
    try:TansformLidar()  
    except rospy.ROSInterruptException:
      rospy.logwarn("EXECUTION COMPLETED SUCCESFULLY")