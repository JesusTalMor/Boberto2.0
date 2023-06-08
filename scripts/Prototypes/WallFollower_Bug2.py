#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist, PoseStamped 
from std_msgs.msg import Float32 
from sensor_msgs.msg import LaserScan   #Lidar 
from geometry_msgs.msg import Point
import numpy as np 

class Bug2():
    '''This class will make the puzzlebot move to a given goal'''
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) 
        #?#******* INIT SUBSCRIBERS *******###
        rospy.Subscriber("base_scan", LaserScan, self.get_lidar_cb)
        rospy.Subscriber('position', Point, self.get_odom)
        rospy.Subscriber('GOAL',Point, self.get_goal)
        #?#******* INIT PUBLISHERS *******###  
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)  

        #?########### Variables ############### 
        self.goal = Point()
        self.goal_received = False   #flag to indicate if the goal has been received 
        
        self.lidar_received = False #flag to indicate if the laser scan has been received 

        target_position_tolerance = 0.01 #target position tolerance [m] 
        fw_distance = 0.25 # distance to activate the following walls behavior [m] 
        tolerance = 0.05 #If the robot is this close to the line with respect to when it started following walls it will stop following walls 
        progress = 0.1
        v_msg=Twist() #Robot's desired speed  

        self.robot_pos = Point()
        self.odom_received = False
        self.wr = 0.0 #right wheel speed [rad/s] 
        self.wl = 0.0 #left wheel speed [rad/s] 

        self.current_state = 'GoToGoal' #Robot's current state 

        #? ######## Point to Line variables#######
        A = 0.0
        B = 0.0
        C = 0.0
        d_p2line = 0.0 # distance point to line
        self.calculate_line = True

        #?#********** INIT NODE **********###  
        freq = 50
        rate = rospy.Rate(freq) #freq Hz  
        rospy.logwarn("BUG 2 NODE INITIALIZE") 
        # print("Please send a Goal from rviz using the button: 2D Nav Goal") 
        # print("You can also publish the goal to the (move_base_simple/goal) topic.") 

        #?############### MAIN LOOP ################  
        while not rospy.is_shutdown():  
            if self.goal_received is False:
                rospy.logwarn("WAIT GOAL")
                self.current_state = 'GoToGoal' #Robot's current state 
                self.stop_robot()
                rate.sleep()
                continue

            if self.odom_received is False:
                rospy.logwarn("NO ODOM")
                rate.sleep()
                continue

            if self.lidar_received is False:
                rospy.logwarn("NO LIDAR DATA")
                rate.sleep()
                continue
            
            # Calcular una nueva recta ? 
            if self.calculate_line is True:
                A,B,C = self.get_eq_values(self.robot_pos, self.goal)

            # Distancia a la recta, generada
            dist_to_line = self.distance2line(A,B,C, self.robot_pos.x, self.robot_pos.y)
            
            # Rango mas cercano al robot, angulo y distancia
            closest_range, closest_angle = self.get_closest_object(self.lidar_msg) #get the closest object range and angle 
            
            # Calcula el angulo para evitar obstaculo
            thetaAO = self.get_theta_ao(closest_angle)
            
            # Calcula el angulo para rodear obstaculo 
            thetaGTG = self.get_theta_gtg(self.goal, self.robot_pos) 
            
            # Distancia del robot al Goal
            dist_to_goal = self.get_distance_to_goal(self.goal, self.robot_pos)

            # Calcular Angulo de Clear SHOT
            theta_clear_shot = np.abs(self.limit_angle(thetaAO-thetaGTG))

            FW_progress = self.D_Fw - self.d_t

            #?#********** MAQUINA DE ESTADOS **********#?#
            rospy.logerr("RUNNING BUG2")
            rospy.logerr("CURRENT STATE: " + str(self.current_state))
            rospy.loginfo("----------------------------------------------")
            rospy.loginfo("DISTANCE TO GOAL: " + str(round(self.d_t, 2)))
            rospy.loginfo("FW PROGRESS: " + str(round(FW_progress,2)) + " | " + str(FW_progress >= progress))
            rospy.loginfo("CLEASHOT ANGLE: " + str(self.RadToDeg(theta_clear_shot)) + " | " + str(theta_clear_shot < np.pi/2.0))
            rospy.loginfo("DISTANCE LINE: " + str(round(dist_to_goal,2)) + " | " + str(dist_to_goal < tolerance))
            rospy.loginfo("----------------------------------------------\n\n")

            # IF THE ROBOT IS AT GOAL....
            if self.at_goal(dist_to_goal, target_position_tolerance):  
                rospy.logwarn("GOAL REACHED - CHANGE TO WAIT GOAL") 
                self.calculate_line = True
                self.goal_received = False

            # IF THE ROBOT IS NOT AT GOAL
            elif self.current_state == 'GoToGoal':                  
                # * Detectamos un objeto muy cercano
                if closest_range <= fw_distance: 
                    # Implement the following walls behavior 
                    rospy.logwarn("WALL DETECTED - CHANGE TO FW") 
                    thetaFWC = self.get_theta_fw(thetaAO, True) # Theta clockwise
                    if abs(thetaFWC-thetaGTG) <= np.pi/2.0:
                        self.current_state = "Clockwise"   
                        rospy.logwarn("GO CLOCKWISE")   
                    else:
                        self.current_state = "CounterClockwise" 
                        rospy.logwarn("GO COUNTERCLOCKWISE") 
                    D_Fw = dist_to_goal

                else: # if we didn't detect an object
                    rospy.loginfo("REACHING GOAL!!!") 
                    v_gtg, w_gtg = self.compute_gtg_control(dist_to_goal, thetaGTG) 
                    v_msg.linear.x = v_gtg 
                    v_msg.angular.z = w_gtg 

            elif self.current_state == 'Clockwise': 
                # * Calcular angulos
                theta_clear_shot = abs(self.limit_angle(thetaAO - thetaGTG))
                #   Ya avanzo una distancia ?   Tiene obstaculos a la vista ?   Esta cerca de la ruta ?
                if FW_progress >= progress and theta_clear_shot < np.pi/2.0 and dist_to_line < tolerance:
                    self.current_state = 'GoToGoal' 
                    rospy.loginfo("CLEARSHOT DETECTED - CHANGE TO GTG") 

                else: 
                    thetaFWC = self.get_theta_fw(thetaAO, True) #If it True is passed return clockwise, else counterclockwise 
                    vFWC, wFWC = self.compute_fw_control(thetaFWC,closest_range,closest_angle) 
                    v_msg.linear.x = vFWC 
                    v_msg.angular.z = wFWC

            elif self.current_state == 'CounterClockwise':
                # * Calcular angulos
                theta_clear_shot = abs(self.limit_angle(thetaAO - thetaGTG))
                #   Ya avanzo una distancia ?   Tiene obstaculos a la vista ?   Esta cerca de la ruta ?
                if FW_progress >= progress and theta_clear_shot < np.pi/2.0 and dist_to_line < tolerance:
                    self.current_state = 'GoToGoal'
                    rospy.loginfo("CLEARSHOT DETECTED - CHANGE TO GTG") 
                else:
                    thetaFWC = self.get_theta_fw(thetaAO, False)
                    vFWCC, wFWCC = self.compute_fw_control(thetaFWC,closest_range,closest_angle)
                    v_msg.linear.x = vFWCC 
                    v_msg.angular.z = wFWCC

            # Funcion para limitar las velocidades lineal y angular
            v_msg.linear.x = self.limit_vel(v_msg.linear.x, 0.4)
            v_msg.angular.z = self.limit_vel(v_msg.angular.z, 0.4)
            
            # PUBLISH VELOCITY
            self.pub_cmd_vel.publish(v_msg)  
            rate.sleep()  

    #?#********** LIMITADORES #?#**********
    def limit_vel(self, v , limit):
        sign = 1 if v > 0.0 else -1
        v = sign * v if np.abs(v) < limit else limit * sign
        return v

    def limit_angle(self, angle):
        """Funcion para limitar de -PI a PI cualquier angulo de entrada"""
        return np.arctan2(np.sin(angle), np.cos(angle))

    def stop_robot(self):
        v_msg = Twist()
        v_msg.linear.x = 0 
        v_msg.angular.z = 0 
        self.pub_cmd_vel.publish(v_msg)
    #?#********** CONDICIONALES #?#**********
    def at_goal(self, distancia, tolerancia):
        #This function returns true if the robot is close enough to the goal 
        #This functions receives the goal's position and returns a boolean 
        #This functions returns a boolean 
        return distancia <= tolerancia

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

    def get_theta_gtg(self, target = Point(), robot = Point()): 
        #This function returns the angle to the goal 
        theta_robot = robot.z
        theta_target=np.arctan2(target.y - robot.y, target.x - robot.x) 
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

    def get_distance_to_goal(target = Point(), robot = Point()):
        return np.sqrt((target.x - robot.x)**2+(target.y - robot.y)**2) 
    
    #?#********** COMPORTAMIENTOS #?#**********
    def compute_gtg_control(self, ed, e_theta): 
        kvmax = 1.0 #linear speed maximum gain  
        kwmax = 1.0 #angular angular speed maximum gain 
        #kw=0.5 
        av = 0.5 #Constant to adjust the exponential's growth rate   
        aw = 0.5 #Constant to adjust the exponential's growth rate 

        #Compute the robot's angular speed 
        kw = kwmax * (1-np.exp(-aw*e_theta**2))/np.abs(e_theta) if e_theta != 0.0 else 0.0 #Constant to change the speed  
        w = kw * 0.4 if e_theta > 0.0 else kw * -0.4 
        if abs(e_theta) > np.pi/8: 
            #we first turn to the goal 
            v = 0.0 #linear speed  
        else: 
            # Make the linear speed gain proportional to the distance to the target position 
            kv = kvmax*(1-np.exp(-av*ed**2))/abs(ed) #Constant to change the speed  
            v = kv * 0.4 #linear speed  
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
    def get_eq_values(self,init_point=Point(),end_point=Point()): 
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
        A = (end_point.y - init_point.y) / (end_point.x - init_point.x)
        B = -1
        C = -A * init_point.x + init_point.y
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
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        vel_msg = Twist() 
        self.pub_cmd_vel.publish(vel_msg) 

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node("BUG2", anonymous=True)
    try: Bug2()  
    except rospy.ROSInterruptException:
        rospy.logwarn("EXECUTION COMPLETED SUCCESFULLY")
