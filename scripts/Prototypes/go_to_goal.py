#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32 
import numpy as np 


class Robot(): 
    #This class implements the differential drive model of the robot 
    def __init__(self): 
        ############ ROBOT CONSTANTS ################  
        self.r = 0.05 #wheel radius [m] 
        self.L = 0.18 #wheel separation [m] 
        ############ Variables ############### 
        self.x = 0.0 #x position of the robot [m] 
        self.y = 0.0 #y position of the robot [m] 
        self.theta = 0.0 #angle of the robot [rad] 

    def update_state(self, wr, wl, delta_t): 
        #This function returns the robot's state 
        #This functions receives the wheel speeds wr and wl in [rad/sec]  
        # and returns the robot's state 
        v = self.r * (wr+wl)/2.0 
        w = self.r * (wr-wl)/self.L 
        self.theta = self.theta + w * delta_t 
        #Crop theta_r from -pi to pi 
        self.theta= np.arctan2(np.sin(self.theta),np.cos(self.theta)) 
        vx=v*np.cos(self.theta) 
        vy=v*np.sin(self.theta) 

        self.x= self.x + vx * delta_t  
        self.y= self.y + vy * delta_t

class GoToGoal:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        self.robot = Robot()

        self.target = Point()
        self.goal_received = False
        self.target_position_tolerance=0.1 #target position tolerance [m] 

        v_msg=Twist() #Robot's desired speed  

        self.wr = 0.0 #right wheel speed [rad/s] 
        self.wl = 0.0 #left wheel speed [rad/s] 
        self.received_wr = False
        self.received_wl = False

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  

        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb) 
        rospy.Subscriber('GOAL',Point, self.get_goal)

        freq = 10
        rate = rospy.Rate(freq) #freq Hz  
        Dt = 1.0/float(freq) #Dt is the time between one calculation and the next one 

        while not rospy.is_shutdown():
            if self.goal_received is False:
                rospy.loginfo("ESPERANDO GOAL")
                self.cleanup()
                rate.sleep()
                continue

            if self.received_wr is False or self.received_wl is False:
                print("No jalan las ruedas")
                rate.sleep()
                continue
            
            # * Actualizar posicion del robot
            self.robot.update_state(self.wr, self.wl, Dt)
            # d_t = self.distance_to_goal(self.target, self.robot)
            
            if self.at_goal():  
                print("Goal reached")
                v_msg.linear.x = 0.0 
                v_msg.angular.z = 0.0
                self.goal_received = False
            else:
                print("Moving to the Goal") 
                v_gtg, w_gtg = self.compute_gtg_control(self.target.x, self.target.y, self.robot.x, self.robot.y, self.robot.theta) 
                v_msg.linear.x = v_gtg 
                v_msg.angular.z = w_gtg 
            
            self.received_wl = False
            self.received_wr = False
            self.pub_cmd_vel.publish(v_msg)  
            rate.sleep()

    def at_goal(self):  # condicion para terminar el proceso
        #This function returns true if the robot is close enough to the goal 
        #This functions receives the goal's position and returns a boolean 
        #This functions returns a boolean 
        dToGoal = np.sqrt((self.target.x-self.robot.x)**2+(self.target.y-self.robot.y)**2)
        print("Distancia al Goal: ", round(dToGoal,2))
        return dToGoal < self.target_position_tolerance 

    # def distance_to_target(self, target=Point(), robot_pos=Robot()):

    def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot): 
        #This function returns the linear and angular speed to reach a given goal 
        #This functions receives the goal's position (x_target, y_target) [m] 
        #  and robot's position (x_robot, y_robot, theta_robot) [m, rad] 
        #This functions returns the robot's speed (v, w) [m/s] and [rad/s] 
        kvmax = 0.4 #linear speed maximum gain  
        kwmax = 0.6 #angular angular speed maximum gain 
        #kw=0.5 
        av = 2.0 #Constant to adjust the exponential's growth rate   
        aw = 2.0 #Constant to adjust the exponential's growth rate 
        ed = np.sqrt((x_target-x_robot)**2+(y_target-y_robot)**2) 
        #Compute angle to the target position 
        theta_target = np.arctan2(y_target-y_robot,x_target-x_robot) 
        e_theta= theta_target - theta_robot 
        #limit e_theta from -pi to pi 
        #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi 
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 

        #Compute the robot's angular speed 
        kw = kwmax*(1-np.exp(-aw*e_theta**2))/abs(e_theta) #Constant to change the speed  
        w=kw*e_theta 
        if abs(e_theta) > np.pi/8.0: 
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
        self.received_wl = True

    def wr_cb(self, wr):  
        ## This function receives a the right wheel speed.  
        self.wr = wr.data
        self.received_wr = True

    def get_goal(self, msg=Point()):
        self.target = msg
        self.goal_received = True
 
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