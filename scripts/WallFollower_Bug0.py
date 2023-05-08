#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist, PoseStamped 
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan   #Lidar 
import numpy as np 

class Robot(): 
    #This class implements the differential drive model of the robot 
    def __init__(self): 
        ############ ROBOT CONSTANTS ################  
        self.r=0.05 #wheel radius [m] 
        self.L = 0.18 #wheel separation [m] 
        ############ Variables ############### 
        self.x = 0.0 #x position of the robot [m] 
        self.y = 0.0 #y position of the robot [m] 
        self.theta = 0.0 #angle of the robot [rad] 

    def update_state(self, wr, wl, delta_t): 
        #This function returns the robot's state 
        #This functions receives the wheel speeds wr and wl in [rad/sec]  
        # and returns the robot's state 
        v=self.r*(wr+wl)/2 
        w=self.r*(wr-wl)/self.L 
        self.theta=self.theta + w*delta_t 
        #Crop theta_r from -pi to pi 
        self.theta=np.arctan2(np.sin(self.theta),np.cos(self.theta)) 
        vx=v*np.cos(self.theta) 
        vy=v*np.sin(self.theta) 

        self.x=self.x+vx*delta_t  
        self.y=self.y+vy*delta_t 

 

#This class will make the puzzlebot move to a given goal 
class GoToGoal():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 

        self.robot=Robot() #create an object of the Robot class 

        ############ Variables ############### 
        self.x_target= 1.0 #x position of the goal 
        self.y_target= 5.5 #y position of the goal 
        self.goal_received=1 #flag to indicate if the goal has been received 
        self.lidar_received = False #flag to indicate if the laser scan has been received 
        self.target_position_tolerance=0.20 #target position tolerance [m] 

        fw_distance = 0.45 # distance to activate the following walls behavior [m] 
        progress = 0.3 #If the robot is this close to the goal with respect to when it started following walls it will stop following walls 
        v_msg=Twist() #Robot's desired speed  

        self.wr=0 #right wheel speed [rad/s] 
        self.wl=0 #left wheel speed [rad/s] 

        self.current_state = 'GoToGoal' #Robot's current state 

        rospy.on_shutdown(self.cleanup)  

        ###******* INIT PUBLISHERS *******###  
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)  
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_cb) 
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb) 

        #********** INIT NODE **********###  
        freq=10
        rate = rospy.Rate(freq) #freq Hz  
        Dt =1.0/float(freq) #Dt is the time between one calculation and the next one 
        print("Node initialized") 
        print("Please send a Goal from rviz using the button: 2D Nav Goal") 
        print("You can also publish the goal to the (move_base_simple/goal) topic.") 

        ################ MAIN LOOP ################  
        while not rospy.is_shutdown():  
            self.robot.update_state(self.wr, self.wl, Dt) #update the robot's state 
            if self.lidar_received: 

                closest_range, closest_angle = self.get_closest_object(self.lidar_msg) #get the closest object range and angle 
                thetaAO = self.get_theta_ao(closest_angle) 
                thetaGTG =self.get_theta_gtg(self.x_target, self.y_target, self.robot.x, self.robot.y, self.robot.theta) 
                d_t=np.sqrt((self.x_target-self.robot.x)**2+(self.y_target-self.robot.y)**2) # Current distance from the robot's position to the goal

                # IF THE ROBOT IS AT GOAL....
                if self.at_goal():  
                    print("Goal reached") 
                    self.current_state = 'Stop' 
                    #print("Stop") 
                    v_msg.linear.x = 0 
                    v_msg.angular.z = 0 

                # IF THE ROBOT IS NOT AT GOAL
                elif self.current_state == 'GoToGoal':                  
                    if closest_range <= fw_distance: # if we detect an object 
                        # Implement the following walls behavior 
                        print(" change to following walls") 
                        thetaFWC = self.get_theta_fw(thetaAO, True) # Theta clockwise
                        if abs(thetaFWC-thetaGTG) <= np.pi/2.0:
                            self.current_state = "Clockwise"   
                            print("Going Clockwise")   
                        else:
                            self.current_state = "CounterClockwise" 
                            print("Going CounterClockwise")
                        D_Fw = d_t # YOU HAVE TO SAVE THE DISTANCE TO THE GOAL 

                    else: # if we didn't detect an object
                        print("Moving to the Goal") 
                        v_gtg, w_gtg = self.compute_gtg_control(self.x_target, self.y_target, self.robot.x, self.robot.y, self.robot.theta) 
                        v_msg.linear.x = v_gtg 
                        v_msg.angular.z = w_gtg 


                elif self.current_state == 'Clockwise': 
                    if d_t < (D_Fw - progress) and abs(thetaAO-thetaGTG) < np.pi/2.0: #ADD output condition#: CONDICION PARA QUE SALGA DEL COMPORTAMIENTO DE FOLLOWING WALLS
                        self.current_state = 'GoToGoal' 
                        print("Change to Go to goal") 

                    else: 
                        thetaFWC=self.get_theta_fw(thetaAO, True) #If it True is passed return clockwise, else counterclockwise 
                        vFWC, wFWC = self.compute_fw_control(thetaFWC) 
                        v_msg.linear.x = vFWC 
                        v_msg.angular.z = wFWC 


                elif self.current_state == 'CounterClockwise':
                    if d_t < (D_Fw-0.5) and (thetaAO-thetaGTG) < np.pi/2.0: #Clear Shot
                        self.current_state = 'GoToGoal'
                        print("Change to Go to goal")
                    else:
                        thetaFWC = self.get_theta_fw(thetaAO, False)
                        vFWCC, wFWCC = self.compute_fw_control(thetaFWC)
                        print ('counterclock')
                        v_msg.linear.x = vFWCC 
                        v_msg.angular.z = wFWCC

                
                elif self.current_state == 'Stop': 
                    print("Stop") 
                    v_msg.linear.x = 0 
                    v_msg.angular.z = 0 

                         
            # PUBLISH VELOCITY
            self.pub_cmd_vel.publish(v_msg)  
            rate.sleep()  

     

    def at_goal(self):  # condicion para terminar el proceso
        #This function returns true if the robot is close enough to the goal 
        #This functions receives the goal's position and returns a boolean 
        #This functions returns a boolean 
        return np.sqrt((self.x_target-self.robot.x)**2+(self.y_target-self.robot.y)**2)<self.target_position_tolerance 

 

    def get_closest_object(self, lidar_msg): 
        #This function returns the closest object to the robot 
        #This functions receives a ROS LaserScan message and returns the distance and direction to the closest object 
        #returns  closest_range [m], closest_angle [rad], 
        min_idx = np.argmin(lidar_msg.ranges) 
        closest_range = lidar_msg.ranges[min_idx] 
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment 
        # limit the angle to [-pi, pi] 
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle)) 
        return closest_range, closest_angle 

 

     

    def get_theta_gtg(self, x_target, y_target, x_robot, y_robot, theta_robot): 
        #This function returns the angle to the goal 
        theta_target=np.arctan2(y_target-y_robot,x_target-x_robot) 
        e_theta=theta_target-theta_robot 
        #limit e_theta from -pi to pi 
        #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi 
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 
        return e_theta 

 

    def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot): 
        #This function returns the linear and angular speed to reach a given goal 
        #This functions receives the goal's position (x_target, y_target) [m] 
        #  and robot's position (x_robot, y_robot, theta_robot) [m, rad] 
        #This functions returns the robot's speed (v, w) [m/s] and [rad/s] 
        kvmax = 0.25 #linear speed maximum gain  
        kwmax=0.8 #angular angular speed maximum gain 
        #kw=0.5 
        av = 2.0 #Constant to adjust the exponential's growth rate   
        aw = 2.0 #Constant to adjust the exponential's growth rate 
        ed=np.sqrt((x_target-x_robot)**2+(y_target-y_robot)**2) 
        #Compute angle to the target position 
        theta_target=np.arctan2(y_target-y_robot,x_target-x_robot) 
        e_theta=theta_target-theta_robot 
 
        #limit e_theta from -pi to pi 
        #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi 
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 

        #Compute the robot's angular speed 
        kw=kwmax*(1-np.exp(-aw*e_theta**2))/abs(e_theta) #Constant to change the speed  
        w=kw*e_theta 
        if abs(e_theta) > np.pi/8: 
            #we first turn to the goal 
            v=0 #linear speed  
        else: 
            # Make the linear speed gain proportional to the distance to the target position 
            kv=kvmax*(1-np.exp(-av*ed**2))/abs(ed) #Constant to change the speed  
            v=kv*ed #linear speed  
        return v,w 

 

    def get_theta_ao(self, theta_closest): 
        ##This function returns the angle for the Avoid obstacle behavior  
        # theta_closest is the angle to the closest object [rad] 
        #This functions returns the angle for the Avoid obstacle behavior [rad] 
        ############################################################ 
        thetaAO=theta_closest-np.pi 
        #limit the angle to [-pi,pi] 
        thetaAO = np.arctan2(np.sin(thetaAO),np.cos(thetaAO)) 

        return thetaAO 

 

  

    def get_theta_fw(self, thetaAO, clockwise): 
        ## This function computes the linear and angular speeds for the robot 
        # It receives thetaAO [rad] and clockwise [bool] 
        # Clockwise true 
        # CounterClockwise False
        if clockwise is True:
            theta_fw = thetaAO - np.pi/2.0
        else:
            theta_fw = thetaAO + np.pi/2.0
        
        theta_fw = np.arctan2(np.sin(theta_fw),np.cos(theta_fw)) # Limit to -pi to pi

        return theta_fw

     

    def compute_fw_control(self, thetaFW): 
        ## This function computes the linear and angular speeds for the robot 
        # It receives thetaFW [rad]    
        #Compute linear and angular speeds 
        kw = 1.5 #angular vel gain
        v = 0.14 #lineal vel is constant [m/s]
        w = kw *thetaFW

        return v, w


     

    def get_angle(self, idx, angle_min, angle_increment):  
        ## This function returns the angle for a given element of the object in the lidar's frame  
        angle= angle_min + idx * angle_increment  
        # Limit the angle to [-pi,pi]  
        angle = np.arctan2(np.sin(angle),np.cos(angle))  
        return angle  

     

    def polar_to_cartesian(self,r,theta):  
        ## This function converts polar coordinates to cartesian coordinates  
        x = r*np.cos(theta)  
        y = r*np.sin(theta)  

        return (x,y)  

     

    def laser_cb(self, msg):   
        ## This function receives a message of type LaserScan   
        self.lidar_msg = msg  
        self.lidar_received = True  
 


    def wl_cb(self, wl):  
        ## This function receives a the left wheel speed [rad/s] 
        self.wl = wl.data 

         

    def wr_cb(self, wr):  
        ## This function receives a the right wheel speed.  
        self.wr = wr.data  

     

    def goal_cb(self, goal):  
        ## This function receives a the goal from rviz.  
        print("Goal received I'm moving to x= "+str(goal.pose.position.x)+" y= "+str(goal.pose.position.y)) 
        self.current_state = "GoToGoal" 
        # assign the goal position 
        self.x_target = goal.pose.position.x 
        self.y_target = goal.pose.position.y 
        self.goal_received=1 

         

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 

 

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("bug_0", anonymous=True)  
    GoToGoal()  