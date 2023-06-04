#!/usr/bin/env python
import rospy
import numpy as np 
from geometry_msgs.msg import PoseStamped
from fiducial_msgs.msg import FiducialTransformArray 
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32 
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_euler 
from geometry_msgs.msg import Point


np.set_printoptions(suppress=True) 

np.set_printoptions(suppress=True) 
np.set_printoptions(formatter={'float': '{: 0.4f}'.format}) 

class KalmanClass():  
    def __init__(self):  
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1) 
        # self.miu_pub = rospy.Publisher("miu",Float32MultiArray, queue_size=1)
        self.position_pub = rospy.Publisher("position", Point, queue_size=1)    
        rospy.Subscriber("wl", Float32, self.wl_cb ) 
        rospy.Subscriber("wr", Float32, self.wr_cb )
        rospy.Subscriber("/fiducial_transforms",FiducialTransformArray,self.get_fiducial) 

        odom = Odometry() 
        #Robot constants  
        r=0.05 #[m] radius of the wheels 
        L=0.18 #[m] distance between wheels 

        #Robot state 
        self.wl = 0.0 #Robot left wheel angular speed 
        self.wr = 0.0 #Robot right wheel angular speed 
        self.x = 0.6 #Robot x-axis postion 
        self.y = 0.0 #Robot position 
        self.theta = np.pi/2 # Robot orientation 
        dt = 0.0 #time interval  
        v=0.0 #[m/s] Robot linear speed 
        w=0.0 #[rad/s] Robot angular speed 

        self.Sigma_pose = np.zeros([3,3]) #Creates the Covariance matrix (3x3) for x, y and theta 

        #Kalman variables
        self.aruco_x = 0.0
        self.aruco_y = 0.0
        self.aruco_noise = 0.0
        self.aruco_id = 0.0
        self.aruco_detected = False

        #Aruco coordinates
        #self.arucos_coordinates = {710:[4,-1],711:[4,7],712:[2,3],713:[0,7],714:[2,3],715:[3,0]}
        self.arucos_coordinates = {701:[0.48,3.15],702:[3.06,3.05],703:[1.91,5.44],704:[1.43,2.45],705:[2.10,0.6],706:[1.44,2.3],726:[5.0,0.0]}
        self.POS_ARUCOS = {
          "701": (0.48,3.15), 
          "702": (2.29,2.85),
          "703": (1.04,4.65),
          "704": (1.43,2.45),
          "705": (1.20,0.98),
        }

        while rospy.get_time() == 0: 
            print("no simulated time has been received yet") 
        print("Got time") 
        last_time = rospy.get_time() 

        self.received_wl = 0 
        self.received_wr = 0 
        rate = rospy.Rate(20) # The rate of the while loop 

        while not rospy.is_shutdown(): 

            if self.received_wl and self.received_wr: 
                #Define the sampling time  
                current_time = rospy.get_time() 
                dt=current_time-last_time 

                #Get a copy of wr an wl to get unexpected changes 
                wr = self.wr 
                wl = self.wl 

                #Compute the robot linear and angular speeds.  
                v= r*(wr+wl)/2.0 #Robot's linear speed 
                w = r*(wr-wl)/L #Robot's Angular Speed 

                # Calculate Covariance Matrix Sigma 
                
                self.predict(dt,v,w,r,L,wr,wl)

                if (self.aruco_detected):
                    self.update()


				#last_time =  current_time
                last_time = current_time

                # miu = [self.x,self.y,self.theta]
                posicion = [round(self.x,2), round(self.y,2), round((self.theta*180.0/np.pi),2)]
                rospy.loginfo("KALMAN POSITION: " + str(posicion))
                robot_pose = Point()
                robot_pose.x = self.x
                robot_pose.y = self.y
                robot_pose.z = self.theta

                # msg = Float32MultiArray()
                # msg.data = miu

                odom = self.fill_odom(v,w) 
                self.odom_pub.publish(odom)
                self.position_pub.publish(robot_pose)
                # self.miu_pub.publish(msg)
                rate.sleep() 

    def wl_cb(self, msg): 
        self.wl = msg.data 
        self.received_wl = 1 
 
    def wr_cb(self, msg): 
        self.wr = msg.data 
        self.received_wr = 1 
        #print(self.wr)

    def get_fiducial(self, msg_array=FiducialTransformArray()):
      need_fiducial_array = []
      # if self.aruco_detected is False:
      for fiducial in msg_array.transforms:
        if str(fiducial.fiducial_id) in self.POS_ARUCOS:
          self.aruco_x = fiducial.transform.translation.z + 0.09
          self.aruco_y = - fiducial.transform.translation.x
          self.aruco_noise = 0.0
          self.aruco_id = fiducial.fiducial_id
          self.aruco_detected = True
          break
            # self.fiducial_received = True
            # need_fiducial_array.append(fiducial)
        # self.fiducial_data = need_fiducial_array

    def aruco_cb(self,msg):
        print("Aruco detected")
        self.aruco_x = msg.data[0]
        self.aruco_y = msg.data[1]
        self.aruco_noise = msg.data[2]
        self.aruco_id = msg.data[3]
        self.aruco_detected = True
        #print(self.aruco_x,self.aruco_y,self.aruco_noise)

    def fill_odom(self,v, w): 
        # (x,y) -> robot position 
        # theta -> robot orientation 
        # Sigma_pose -> 3x3 pose covariance matrix 

        odom=Odometry() 
        odom.header.stamp =rospy.Time.now() 
        odom.header.frame_id = "odom" 
        odom.child_frame_id = "base_link" 
        odom.pose.pose.position.x = self.x 
        odom.pose.pose.position.y = self.y 
        odom.pose.pose.position.z = 0.0 
        quat=quaternion_from_euler(0.0, 0.0, self.theta) 

        odom.pose.pose.orientation.x = quat[0] 
        odom.pose.pose.orientation.y = quat[1] 
        odom.pose.pose.orientation.z = quat[2] 
        odom.pose.pose.orientation.w = quat[3] 
        odom.pose.covariance = [0.0]*36 

        # Fill the covariance matrix 
        odom.pose.covariance[0] = self.Sigma_pose[0,0] 
        odom.pose.covariance[1] = self.Sigma_pose[0,1] 
        odom.pose.covariance[5] = self.Sigma_pose[0,2] 
        odom.pose.covariance[6] = self.Sigma_pose[1,0] 
        odom.pose.covariance[7] = self.Sigma_pose[1,1] 
        odom.pose.covariance[11] = self.Sigma_pose[1,2] 
        odom.pose.covariance[30] = self.Sigma_pose[2,0] 
        odom.pose.covariance[31] = self.Sigma_pose[2,1] 
        odom.pose.covariance[35] = self.Sigma_pose[2,2] 

        odom.twist.twist.linear.x = v 
        odom.twist.twist.angular.z = w
        # print(self.x,self.y,self.theta) 
        return odom 

    def predict(self,delta_t,v,w,r,L,wr,wl):
        kr,kl = 0.1, 0.1

        sigma_w = np.array([[kr*abs(wr),0],
                            [0.0, kl*abs(wl)]
                           ])
        nabla_w = 1.0/2.0 * r * delta_t * np.array([[np.cos(self.theta), np.cos(self.theta)],
                                           [np.sin(self.theta), np.sin(self.theta)],
                                           [2.0/L, -2.0/L]])

        #print(nabla_w)

        q = np.array(nabla_w.dot(sigma_w).dot(nabla_w.T))

        # Algoritmo
        
        H = np.array([[1.0, 0.0, -delta_t * v * np.sin(self.theta)],
                    [0.0, 1.0, delta_t * v * np.cos(self.theta)],
                    [0.0, 0.0, 1.0]])

        self.Sigma_pose = np.array(H.dot(self.Sigma_pose).dot(H.T)+q)
        self.x = self.x + delta_t * v * np.cos(self.theta)
        self.y = self.y + delta_t * v * np.sin(self.theta)
        self.theta = self.theta + delta_t * w
        #Crop theta from -pi to pi 
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta)) #Make theta from -pi to pi 
	#print('Predict',self.x,self.y,self.theta)


    def update(self):
        Mk = np.array([[self.x],[self.y],[self.theta]])

        aruco_distance = np.sqrt(self.aruco_x**2 + self.aruco_y**2)
        aruco_angle = np.arctan2(self.aruco_y,self.aruco_x)
        z_aruco =  np.array([[aruco_distance],[aruco_angle]]) #sensor measure
        
        coordenada = self.arucos_coordinates[self.aruco_id]
        x_odom = coordenada[0]-self.x
        y_odom = coordenada[1]-self.y
	
                                        
        odom_distance = np.sqrt(x_odom**2 + y_odom**2)
        odom_angle = np.arctan2(y_odom,x_odom) - self.theta
        odom_limited = np.arctan2(np.sin(odom_angle),np.cos(odom_angle))
        p = x_odom**2 + y_odom**2
        z_odom = np.array([[odom_distance],[odom_limited]]) #odometry measure

        Gk = np.array([[-x_odom/np.sqrt(p),   -y_odom/np.sqrt(p),     0],
                        [y_odom/p,               -x_odom/p,               -1]])

        #Rk = np.array([[self.aruco_noise,0],[0,self.aruco_noise]])
        Rk = np.array([[0.1,0],[0,0.02]])

        Zk = Gk.dot(self.Sigma_pose).dot(Gk.T) + Rk

        Kk = self.Sigma_pose.dot(Gk.T).dot(np.linalg.inv(Zk))


        Mk = Mk + Kk.dot(z_aruco - z_odom)


        
        self.x=Mk[0][0]
        self.y=Mk[1][0] 
        self.theta = Mk[2][0] 
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta)) #Make theta from -pi to pi

        self.Sigma_pose = (np.eye(3) - Kk.dot(Gk)).dot(self.Sigma_pose)
	#print('Update',self.x,self.y,self.theta)
        self.aruco_detected = False


############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  
    # first thing, init a node! 
    rospy.init_node('odometry_kalman')  
    KalmanClass()