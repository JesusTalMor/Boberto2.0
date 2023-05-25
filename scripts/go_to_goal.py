#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32 
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

class GoToGoal:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        self.robot = Robot()

        self.x_target= 3.0 #x position of the goal 
        self.y_target= 0.0 #y position of the goal
        self.target_position_tolerance=0.20 #target position tolerance [m] 

        v_msg=Twist() #Robot's desired speed  

        self.wr=0 #right wheel speed [rad/s] 
        self.wl=0 #left wheel speed [rad/s] 

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  

        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb) 

        freq=10
        rate = rospy.Rate(freq) #freq Hz  
        Dt =1.0/float(freq) #Dt is the time between one calculation and the next one 

        while not rospy.is_shutdown():
            self.robot.update_state(self.wr, self.wl, Dt)

            if self.at_goal():  
                    print("Goal reached")
                    v_msg.linear.x = 0 
                    v_msg.angular.z = 0 
            else:
                print("Moving to the Goal") 
                v_gtg, w_gtg = self.compute_gtg_control(self.x_target, self.y_target, self.robot.x, self.robot.y, self.robot.theta) 
                v_msg.linear.x = v_gtg 
                v_msg.angular.z = w_gtg 
            
            self.pub_cmd_vel.publish(v_msg)  
            rate.sleep()

    def at_goal(self):  # condicion para terminar el proceso
        #This function returns true if the robot is close enough to the goal 
        #This functions receives the goal's position and returns a boolean 
        #This functions returns a boolean 
        return np.sqrt((self.x_target-self.robot.x)**2+(self.y_target-self.robot.y)**2)<self.target_position_tolerance 

    
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

    
    def wl_cb(self, wl):  
        ## This function receives a the left wheel speed [rad/s] 
        self.wl = wl.data 

    def wr_cb(self, wr):  
        ## This function receives a the right wheel speed.  
        self.wr = wr.data 

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg)

############################## MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("GTG", anonymous=True)
    try: GoToGoal()  
    except rospy.ROSInterruptException:
        rospy.logwarn("EXECUTION COMPLETED SUCCESFULLY")