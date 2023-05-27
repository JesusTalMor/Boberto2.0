#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist, PoseStamped 
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan   #Lidar 
import numpy as np 
# Pruebas
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
        v = self.r * (wr+wl)/2 
        w = self.r * (wr-wl)/self.L 
        self.theta = self.theta + w * delta_t 
        #Crop theta_r from -pi to pi 
        self.theta = np.arctan2(np.sin(self.theta),np.cos(self.theta)) 
        vx = v * np.cos(self.theta) 
        vy = v * np.sin(self.theta) 

        self.x=self.x+vx*delta_t  
        self.y=self.y+vy*delta_t 


class GoToGoal():
    '''This class will make the puzzlebot move to a given goal'''
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 

        self.robot=Robot() #create an object of the Robot class 

        #?########### Variables ############### 
        self.x_target= 0.01 #x position of the goal 
        self.y_target= 4.0 #y position of the goal 
        self.goal_received = True   #flag to indicate if the goal has been received 
        self.lidar_received = False #flag to indicate if the laser scan has been received 
        target_position_tolerance = 0.25 #target position tolerance [m] 

        fw_distance = 0.25 # distance to activate the following walls behavior [m] 
        tolerance = 0.05 #If the robot is this close to the line with respect to when it started following walls it will stop following walls 
        progress = 0.1
        v_msg=Twist() #Robot's desired speed  

        self.wr = 0.0 #right wheel speed [rad/s] 
        self.wl = 0.0 #left wheel speed [rad/s] 

        self.current_state = 'GoToGoal' #Robot's current state 

        #? ######## Point to Line variables#######
        A = 0.0
        B = 0.0
        C = 0.0
        d_p2line = 0.0 # distance point to line
        self.calculate_line = True
  

        #?#******* INIT PUBLISHERS *******###  
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("wl", Float32, self.wl_cb)  
        rospy.Subscriber("wr", Float32, self.wr_cb)  
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_cb) 
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb) 

        #?#********** INIT NODE **********###  
        freq = 50
        rate = rospy.Rate(freq) #freq Hz  
        Dt = 1.0/float(freq) #Dt is the time between one calculation and the next one 
        print("Node initialized") 
        print("Please send a Goal from rviz using the button: 2D Nav Goal") 
        print("You can also publish the goal to the (move_base_simple/goal) topic.") 

        #?############### MAIN LOOP ################  
        while not rospy.is_shutdown():  
            self.robot.update_state(self.wr, self.wl, Dt) #update the robot's state 
            if self.lidar_received:
                # Calcular una nueva recta ? 
                if self.calculate_line is True:
                    A,B,C = self.get_eq_values(self.robot.x,self.robot.y,self.x_target,self.y_target)

                # Rango mas cercano al robot, angulo y distancia
                closest_range, closest_angle = self.get_closest_object(self.lidar_msg) #get the closest object range and angle 
                # Calcula el angulo para evitar obstaculo
                thetaAO = self.get_theta_ao(closest_angle)
                # Calcula el angulo para rodear obstaculo 
                thetaGTG =self.get_theta_gtg(self.x_target, self.y_target, self.robot.x, self.robot.y, self.robot.theta) 
                # Distancia del robot al Goal
                d_t=np.sqrt((self.x_target-self.robot.x)**2+(self.y_target-self.robot.y)**2) 
                # Distancia a la recta, generada
                d_p2line = self.distance2line(A,B,C,self.robot.x,self.robot.y)

                print("Distancia Mas Cercana: ",closest_range)

                # IF THE ROBOT IS AT GOAL....
                if self.at_goal(d_t, target_position_tolerance):  
                    print("Goal reached") 
                    self.current_state = 'Stop' 
                    self.calculate_line = True
                    v_msg.linear.x = 0 
                    v_msg.angular.z = 0 

                # IF THE ROBOT IS NOT AT GOAL
                elif self.current_state == 'GoToGoal':                  
                    # * Detectamos un objeto muy cercano
                    if closest_range <= fw_distance: 
                        # Implement the following walls behavior 
                        print("Change to following walls") 
                        thetaFWC = self.get_theta_fw(thetaAO, True) # Theta clockwise
                        if abs(thetaFWC-thetaGTG) <= np.pi/2.0:
                            self.current_state = "Clockwise"   
                            print("Going Clockwise")   
                        else:
                            self.current_state = "CounterClockwise" 
                            print("Going CounterClockwise") 
                        D_Fw = d_t

                    else: # if we didn't detect an object
                        print("Moving to the Goal") 
                        v_gtg, w_gtg = self.compute_gtg_control(d_t, thetaGTG) 
                        v_msg.linear.x = v_gtg 
                        v_msg.angular.z = w_gtg 

                elif self.current_state == 'Clockwise': 
                    # Revisar si tenemos un Clear Shot
                    print("Distancia de Progreso: ", round(d_t,2), round((D_Fw - progress),2))
                    print("Angulo Clear Shot: ", round(abs(thetaAO-thetaGTG),2))
                    print("Distancia a la recta: ", round(d_p2line),2)
                    # * Calcular angulos
                    theta_clear_shot = abs(self.limit_angle(thetaAO - thetaGTG))
                    #   Ya avanzo una distancia ?   Tiene obstaculos a la vista ?   Esta cerca de la ruta ?
                    if d_t < (D_Fw - progress) and theta_clear_shot < np.pi/2.0 and d_p2line < tolerance: 
                        self.current_state = 'GoToGoal' 
                        print("Change to Go to goal") 

                    else: 
                        thetaFWC = self.get_theta_fw(thetaAO, True) #If it True is passed return clockwise, else counterclockwise 
                        vFWC, wFWC = self.compute_fw_control(thetaFWC,closest_range,closest_angle) 
                        print("Clockwise")
                        v_msg.linear.x = vFWC 
                        v_msg.angular.z = wFWC

                elif self.current_state == 'CounterClockwise':
                    print("Distancia de Progreso: ", round(d_t,2), round((D_Fw - progress),2))
                    print("Angulo Clear Shot: ", round(abs(thetaAO-thetaGTG),2))
                    print("Distancia a la recta: ", round(d_p2line),2)
                    # * Calcular angulos
                    theta_clear_shot = abs(self.limit_angle(thetaAO - thetaGTG))
                    #   Ya avanzo una distancia ?   Tiene obstaculos a la vista ?   Esta cerca de la ruta ?
                    if d_t < (D_Fw - progress) and theta_clear_shot < np.pi/2.0 and d_p2line < tolerance: 
                        self.current_state = 'GoToGoal'
                        print("Change to Go to goal")
                    else:
                        thetaFWC = self.get_theta_fw(thetaAO, False)
                        vFWCC, wFWCC = self.compute_fw_control(thetaFWC,closest_range,closest_angle)
                        print ('counterclock')
                        v_msg.linear.x = vFWCC 
                        v_msg.angular.z = wFWCC

                elif self.current_state == 'Stop': 
                    print("Stop") 
                    v_msg.linear.x = 0 
                    v_msg.angular.z = 0 


            # Funcion para limitar las velocidades lineal y angular
            v_msg.linear.x, v_msg.angular.z = self.limit_vel(v_msg.linear.x,v_msg.angular.z)

            
            # PUBLISH VELOCITY
            self.pub_cmd_vel.publish(v_msg)  
            rate.sleep()  

    #?#********** LIMITADORES #?#**********
    def limit_vel(self, v , w):
        sign = 1 if w > 0 else -1
        if abs(w) > 0.4:
            w = sign * 0.4

        sign = 1 if v > 0 else -1
        if abs(v) > 0.4:
            v = sign * 0.4

        sign = 1 if v > 0 else -1
        if abs(v) < 0.1 and v != 0.0:
            v = sign * 0.1

        return v,w

    def limit_angle(self, angle):
        """Funcion para limitar de -PI a PI cualquier angulo de entrada"""
        return np.arctan2(np.sin(angle), np.cos(angle))

    #?#********** CONDICIONALES #?#**********
    def at_goal(self, distancia, tolerancia):
        #This function returns true if the robot is close enough to the goal 
        #This functions receives the goal's position and returns a boolean 
        #This functions returns a boolean 
        return distancia < tolerancia

    #?#********** GETTERS #?#**********
    def get_closest_object(self, lidar_msg):
        """ This function returns the closest object to the robot
        This functions receives a ROS LaserScan message and returns the distance and direction to the closest object
        returns  closest_range [m], closest_angle [rad], 
        """ 
        min_idx = np.argmin(lidar_msg.ranges) 
        closest_range = lidar_msg.ranges[min_idx]
        closest_angle = lidar_msg.angle_min + min_idx * lidar_msg.angle_increment 
        # limit the angle to [-pi, pi] 
        closest_angle = self.limit_angle(closest_angle) 
        return closest_range, closest_angle 

    def get_theta_gtg(self, x_target, y_target, x_robot, y_robot, theta_robot): 
        #This function returns the angle to the goal 
        theta_target=np.arctan2(y_target - y_robot, x_target - x_robot) 
        e_theta= theta_target - theta_robot 
        #limit e_theta from -pi to pi 
        #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi 
        e_theta = self.limit_angle(e_theta)
        return e_theta 

    def get_theta_ao(self, theta_closest): 
        ##This function returns the angle for the Avoid obstacle behavior  
        # theta_closest is the angle to the closest object [rad] 
        #This functions returns the angle for the Avoid obstacle behavior [rad] 
        ############################################################ 
        thetaAO=theta_closest-np.pi 
        #limit the angle to [-pi,pi] 
        thetaAO = self.limit_angle(thetaAO)

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
        
        theta_fw = self.limit_angle(theta_fw)

        return theta_fw
    
    #?#********** COMPORTAMIENTOS #?#**********
    def compute_gtg_control(self, ed, e_theta): 
        #This function returns the linear and angular speed to reach a given goal 
        #This functions receives the goal's position (x_target, y_target) [m] 
        #  and robot's position (x_robot, y_robot, theta_robot) [m, rad] 
        #This functions returns the robot's speed (v, w) [m/s] and [rad/s] 
        kvmax = 0.25 #linear speed maximum gain  
        kwmax=0.8 #angular angular speed maximum gain 
        #kw=0.5 
        av = 2.0 #Constant to adjust the exponential's growth rate   
        aw = 2.0 #Constant to adjust the exponential's growth rate 

        #Compute the robot's angular speed 
        kw= kwmax*(1-np.exp(-aw*e_theta**2))/abs(e_theta) if e_theta != 0.0 else 0.0 #Constant to change the speed  
        w=kw*e_theta 
        if abs(e_theta) > np.pi/8: 
            #we first turn to the goal 
            v=0 #linear speed  
        else: 
            # Make the linear speed gain proportional to the distance to the target position 
            kv=kvmax*(1-np.exp(-av*ed**2))/abs(ed) #Constant to change the speed  
            v=kv*ed #linear speed  
        return v,w 

    def compute_fw_control(self, thetaFW, closest_range, closest_angle): 
        ## This function computes the linear and angular speeds for the robot 
        # It receives thetaFW [rad]    
        #Compute linear and angular speeds 
        kw = 1.2
        if abs(thetaFW) > np.pi/10:
            v = 0.0
            w = kw * thetaFW
        elif closest_range > 0.3:
            kw = 0.5
            w = kw*(thetaFW + closest_angle/2.0)
            v = 0.1 #lineal vel is constant [m/s]
        else:
            w = kw *thetaFW
            v = 0.1 #lineal vel is constant [m/s]

        return v, w

    #?#********** FUNCIONES PARA MANEJO DE RECTA #?#**********
    def get_eq_values(self,x1,y1,x2,y2): 
        '''
        This function calculate A,B,C that we gonna use to point to line function
        x1,y1 corresponds to x and y robot
        x2,y2 corresponds to x and y goal
        y-y1 = m(x-x1)
        0 = m*x-m*x1 + y1 - y

        A = m
        B = -1 
        C = -m*x1 + y1

        0 = Ax + By + C
        '''
        A = (y2 - y1) / (x2 - x1)
        B = -1
        C = -A * x1 + y1
        self.calculate_line = False
        return A, B, C

    def distance2line(self,A,B,C,x1,y1):
        print("Valores de la recta: ", A, B, C)
        d = abs((A*x1)+(B*y1)+C)/np.sqrt((A**2)+(B**2))
        return d

    #?#********** CALLBACKS #?#**********
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
    rospy.init_node("bug_2", anonymous=True)
    try: GoToGoal()  
    except rospy.ROSInterruptException:
        rospy.logwarn("EXECUTION COMPLETED SUCCESFULLY")
