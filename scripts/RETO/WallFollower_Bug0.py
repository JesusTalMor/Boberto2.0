#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist, Point 
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan   #Lidar 
import numpy as np 

#This class will make the puzzlebot move to a given goal 
class GoToGoal():
    '''
    This class will make the puzzlebot move to a given goal
    '''
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 

        ############ Variables ############### 
        self.x_target= 0.0 #x position of the goal 
        self.y_target= 0.0 #y position of the goal 
        self.goal_received=0 #flag to indicate if the goal has been received 
        self.lidar_received = False #flag to indicate if the laser scan has been received 
        self.target_position_tolerance=0.2 #target position tolerance [m] 

        fw_distance = 0.35 # distance to activate the following walls behavior [m] 
        progress = 0.3 #If the robot is this close to the goal with respect to when it started following walls it will stop following walls 
        v_msg=Twist() #Robot's desired speed  

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.wr=0 #right wheel speed [rad/s] 
        self.wl=0 #left wheel speed [rad/s] 

        self.current_state = 'Stop' #Robot's current state 

        rospy.on_shutdown(self.cleanup)  

        ###******* INIT PUBLISHERS *******###  
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)  
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb) 
        rospy.Subscriber("position", Point, self.get_robot_pos)
        rospy.SubscribeLr("GOAL", Point, self.get_goal)


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
            if self.lidar_received and self.goal_received == 1: 

                closest_range, closest_angle = self.get_closest_object(self.lidar_msg) #get the closest object range and angle 
                thetaAO = self.get_theta_ao(closest_angle) 
                thetaGTG =self.get_theta_gtg(self.x_target, self.y_target, self.robot_x, self.robot_y, self.robot_theta) 
                d_t=np.sqrt((self.x_target-self.robot_x)**2+(self.y_target-self.robot_y)**2) # Current distance from the robot's position to the goal
                print(closest_range)
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
                        v_gtg, w_gtg = self.compute_gtg_control(self.x_target, self.y_target, self.robot_x, self.robot_y, self.robot_theta) 
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
                    if d_t < (D_Fw- progress) and abs(thetaAO-thetaGTG) < np.pi/2.0: #Clear Shot
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
        '''
        This function returns true if the robot is close enough to the goal 
        
        Parameters
        ----------
        This functions receives the goal's position and current robot position
        
        Return
        ------
        Bool
        '''
        return np.sqrt((self.x_target-self.robot_x)**2+(self.y_target-self.robot_y)**2)<self.target_position_tolerance 


    def get_closest_object(self, lidar_msg):
        '''
        Calculate the closest object to the robot

        Parameters
        ---------
        lidar_msg: ROS LaserScan message

        
        Return
        ------
        closest_range: float
            distance to the closest object [m]
        closest_angle: float
            distance to the closest_angle  [rad]
        '''
        min_idx = np.argmin(lidar_msg.ranges) 
        closest_range = lidar_msg.ranges[min_idx] 
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment 
        # Crop the angle to [-pi, pi] 
        closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle)) 
        return closest_range, closest_angle 


    def get_theta_gtg(self, x_target, y_target, x_robot, y_robot, theta_robot): 
        '''
        Calculate the angle to the goal

        Parameters
        ---------
        x_target:    float
            "x" coordinate of the goal
        y_target:    float
            "y" coordinate of the goal
        x_robot:     float
            "x" coordinate of robot's current position
        y_robot:     float
            "y" coordinate of robot's current position
        theta_robot: float
            Robot's current angle

        Return
        ------
        e_theta: float
            Update the angle value between the robot and the goal
        
        ''' 
        theta_target=np.arctan2(y_target-y_robot,x_target-x_robot) 
        e_theta=theta_target-theta_robot 
        #limit e_theta from -pi to pi 
        #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi 
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) 
        return e_theta 


    def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot):
        '''
        Calculate the linear and angular speed to reach a given goal

        Parameters
        ----------
        x_target: float
            Goal's position (x-component)  [m]
        y_target: float
            Goal's position (y-component)  [m]
        x_robot: float
            Robot's position (x-component) [m]
        y_robot: float
            Robot's position (y-component) [m]
        theta_robot: float
            Robot's position (angle component) [rad]
        
        Return
        ------
        v: float
            linear speed robot [m/s]
        w: float
            angular speed robot [rad/s]
        '''
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
        '''
        Calculate the angle for the Avoid obstacle behavior  

        Parameters
        ----------
        theta_closest: float
            Angle to the closest object [rad]
        
        Return
        ------
        thetaAO: float
            Angle for the Avoid Obstacle behavior [rad] 
        '''
        thetaAO=theta_closest-np.pi 
        #limit the angle to [-pi,pi] 
        thetaAO = np.arctan2(np.sin(thetaAO),np.cos(thetaAO)) 
        return thetaAO 



    def get_theta_fw(self, thetaAO, clockwise):
        '''
        Computes the angle for Following Walls behavior

        Parameters
        ----------
        thetaA0: float
        clockwise: bool
            True = Clockwise
            False = CounterClockwise

        Return
        ------
        theta_fw: float
            angle for following walls
        '''
        if clockwise is True:
            theta_fw = thetaAO - np.pi/2.0
        else:
            theta_fw = thetaAO + np.pi/2.0
        
        theta_fw = np.arctan2(np.sin(theta_fw),np.cos(theta_fw)) # Limit to -pi to pi

        return theta_fw


    def compute_fw_control(self, thetaFW): 
        '''
        Computes the linear and angular speeds for the robot
        in follwing wall behavior 

        Parameters
        ----------
        thetaFW: float
            Following theta angle [rad]   

        Return
        ------
        v: float
            linear speed robot [m/s]
        w: float
            angular speed robot [rad/s]
        '''
        kw = 1.2
        if abs(thetaFW) > np.pi/5.0:
            v = 0.0
            w = kw *thetaFW
        else:
            v = 0.1
            w = kw *thetaFW
        return v, w

    def laser_cb(self, msg):   
        '''Receives a message of type LaserScan'''
        self.lidar_msg = msg  
        self.lidar_received = True  


    def wl_cb(self, wl):  
        '''Receives a the left wheel speed [rad/s]'''
        self.wl = wl.data 


    def wr_cb(self, wr):  
        '''Receives a the right wheel speed [rad/s]''' 
        self.wr = wr.data  


    def get_goal(self, msg = Point()):
        self.x_target = msg.x
        self.y_target = msg.y
        self.goal_received = 1
        self.current_state = "GoToGoal"

    def get_robot_pos(self, msg = Point()):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_theta = msg.z


    def cleanup(self): 
        '''
        This function is called just before finishing the node  
        You can use it to clean things up before leaving  
        Example: stop the robot before finishing a node.
        '''
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 


############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":
    rospy.init_node("bug_0", anonymous=True)  
    try: GoToGoal()  
    except rospy.ROSInterruptException:
        rospy.logwarn("EXECUTION COMPLETED SUCCESFULLY")