#!/usr/bin/env python  
import rospy  
import numpy as np
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class Bug0():  
    '''
    Un super twisting con los colagenos o que?
    Que sepas que me encanto
    '''
    def __init__(self):  
        rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
        ###******* INIT PUBLISHERS *******###  
        rospy.Subscriber("base_scan", LaserScan, self.get_lidar_cb)
        rospy.Subscriber('position', Point, self.get_odom)
        rospy.Subscriber('GOAL', Point, self.get_goal)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.gtg_topic = rospy.Publisher('gtg_topic', Bool, queue_size=1)
        self.fw_topic = rospy.Publisher('fw_topic', Bool, queue_size=1)  
        ###******* INIT CONSTANTS/VARIABLES *******###  
        # Posicion del Robot
        self.robot_pos = Point()
        self.odom_received = False

        # Definicion de punto inicial y goal    
        # GOAlS = [
        #     (0.51, 2.08),
        #     # (2.39, 4.77),
        #     # (1.79, 3.57),
        #     # (1.53, 1.30),
        #     # (1.42, 0.21),
        # ]
        # GOAL_INDEX = 0
        self.target = Point()
        # self.target.x, self.target.y = GOAlS[GOAL_INDEX]
        self.goal_recieved = False
    
        # Bandera para indicar que el lidar recibio datos 
        self.lidar_data = LaserScan()
        self.region_active = False 
        self.gtg_active = False
        self.fw_active = False

        # Definicion de estados
        self.current_state = "GTG"


        states = {
        "GTG" : "GO_TO_GOAL",
        "FW" : "WALL_FOLLOWER",
        "S" : "STOP"
        }

        progress = 0.1 # Para verificar si el robot avanzo esta distancia antes de cambiar de estado
        self.d_t = 0.0
        self.D_Fw = 0.0
        goal_tolance = 0.01


        rate = rospy.Rate(10) # The rate of the while loop will be 50Hz 
        rospy.loginfo("Starting Message!")     
        # Grab possible simulation Error
        # while rospy.get_time() == 0: 
        #   print("no simulated time has been received yet") 
        # start_time = rospy.get_time()  #Get the current time in float seconds 
        ###******* PROGRAM BODY *******###  
        while not rospy.is_shutdown(): 
            # * Whole Body of the Program
            if self.goal_recieved is False:
                rospy.logwarn("WAIT GOAL")
                self.gtg_topic.publish(False)
                self.fw_topic.publish(False)
                self.gtg_active = False
                self.fw_active = False
                self.stop_robot()
                rate.sleep()
                # if GOAL_INDEX + 1 < len(GOAlS):
                #     GOAL_INDEX += 1
                #     self.target.x, self.target.y = GOAlS[GOAL_INDEX]
                #     rospy.logwarn("GOING GOAL NUM. " + str(GOAL_INDEX))
                #     self.goal_recieved = True
                # else:
                #     self.goal_recieved = False
                continue

            if self.odom_received is False:
                # rospy.logwarn("NO ODOM")
                rate.sleep()
                continue

            if self.region_active is False:
                rate.sleep()
                continue

            # * Si ambos estados estan en Falso iniciar con GTG
            if self.gtg_active is False and self.fw_active is False:
                self.current_state = "GTG"
                self.gtg_topic.publish(True)
                self.fw_topic.publish(False)  
                self.gtg_active = True
                self.fw_active = False

            # Closest object 
            closest_dist, closest_angle = self.get_closest_object(self.lidar_data)

            # Avoid obstacle theta
            thetaAO = self.get_theta_ao(closest_angle) 

            # Go To Goal obstacle theta 
            thetaGTG = self.get_theta_gtg(self.target, self.robot_pos)
            
            # Distance to Goal
            self.d_t = self.get_distance_to_goal(self.target,self.robot_pos)

            # Calcular Angulo de Clear SHOT
            theta_clear_shot = np.abs(self.limit_angle(thetaAO-thetaGTG))

            FW_progress = self.D_Fw - self.d_t
            
            #?#********** MAQUINA DE ESTADOS **********#?#
            rospy.logerr("CURRENT GOAL: " + str(self.target.x) + "," + str(self.target.y))
            rospy.logerr("CURRENT STATE: " + str(self.current_state))
            rospy.loginfo("----------------------------------------------")
            rospy.loginfo("DISTANCE TO GOAL: " + str(round(self.d_t, 2)))
            rospy.loginfo("FW PROGRESS: " + str(round(FW_progress,2)) + " | " + str(FW_progress >= progress))
            rospy.loginfo("CLEASHOT ANGLE: " + str(self.RadToDeg(theta_clear_shot)) + " | " + str(theta_clear_shot < np.pi/2.0))
            rospy.loginfo("----------------------------------------------\n\n")

            if self.d_t < goal_tolance:
                rospy.loginfo("GOAL REACHED")
                self.change_state("STOP")

            elif self.current_state == "GTG":
                if closest_dist < 0.35:
                    print("WALL DETECTED - CHANGE TO FW")
                    self.change_state("FW")

            elif self.current_state == "FW":
                if FW_progress >= progress and theta_clear_shot < np.pi/2.0:
                    rospy.logwarn("CLEAR SHOT - CHANGE TO GTG")
                    self.change_state("GTG")
        
            self.odom_received = False
            self.region_active = False
            rate.sleep()

    #?#********** LIMITADORES #?#**********
    def limit_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))  

    def RadToDeg(self, angle):
        return round(angle*180.0/np.pi, 2)
    
    def change_state(self,state):
        self.current_state = state
        if self.current_state == "STOP":
            #* Apagar nodos de GTG y FW
            self.gtg_topic.publish(False)
            self.fw_topic.publish(False)
            self.gtg_active = False
            self.fw_active =  False
            self.stop_robot()
            self.goal_recieved =  False
            self.initial_pos = self.robot_pos

        elif self.current_state == "GTG":
            self.gtg_topic.publish(True)
            self.fw_topic.publish(False)
            self.gtg_active = True
            self.fw_active =  False
            rospy.sleep(3)

        elif self.current_state == "FW":
            self.D_Fw= self.d_t
            self.gtg_topic.publish(False)
            self.fw_topic.publish(True)
            self.gtg_active = False
            self.fw_active =  True
            #rospy.sleep(2)
    
    #?# ********** CALLBACKS #?#**********    
    def get_lidar_cb(self, msg=LaserScan()):
        if self.region_active is False:
            self.lidar_data = msg
            self.region_active = True

    def get_odom(self, msg=Point()):
        # position
        if self.odom_received is False:
            self.robot_pos = msg
            self.odom_received = True

    def get_goal(self, msg=Point()):
        if self.goal_recieved is False:
            self.target = msg
            self.goal_recieved = True
    
    #?# ********** GETTERS **********#?#
    def get_closest_object(self,lidar_data = LaserScan()):
        # Para delimitar region de al frente del robot
        idx_low = 360
        idx_high = 788
        front_data = lidar_data.ranges[idx_low:idx_high]

        Front = min(min(front_data), 10)

        # Para obtener closest angle
        min_idx = np.argmin(lidar_data.ranges)
        closest_angle = lidar_data.angle_min + min_idx * lidar_data.angle_increment 
        # limitar el angulo
        closest_angle = self.limit_angle(closest_angle) 
        return Front, closest_angle


    #?# ********** COMPORTAMIENTOS #?#**********
    def get_theta_ao(self, theta_closest): 
        ##This function returns the angle for the Avoid obstacle behavior  
        # theta_closest is the angle to the closest object [rad] 
        #This functions returns the angle for the Avoid obstacle behavior [rad] 
        ############################################################ 
        thetaAO=theta_closest-np.pi 
        #limit the angle to [-pi,pi] 
        thetaAO = self.limit_angle(thetaAO)
        return thetaAO

    def get_theta_gtg(self, target_pos = Point(), robot_pos = Point()): 
        #This function returns the angle to the goal 
        x_target, y_target = target_pos.x, target_pos.y
        x_robot, y_robot, theta_robot = robot_pos.x, robot_pos.y, robot_pos.z

        theta_target=np.arctan2(y_target - y_robot, x_target - x_robot) 
        e_theta= theta_target - theta_robot 
        #limit e_theta from -pi to pi 
        #This part is very important to avoid abrupt changes when error switches between 0 and +-2pi 
        e_theta = self.limit_angle(e_theta)
        return e_theta

    def get_distance_to_goal(self, target = Point(), robot_pos = Point()):
        delta_x = target.x - robot_pos.x
        delta_y = target.y - robot_pos.y
        distance = np.sqrt((delta_x) ** 2 + (delta_y) ** 2)
        return distance

    #?# ********** CLEAN #?#**********
    def stop_robot(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(vel_msg)

    def cleanup(self):  
        self.gtg_topic.publish(False)
        self.fw_topic.publish(False)
        self.gtg_active = False
        self.fw_active = False
        self.stop_robot()
        '''This function is called just before finishing the node.'''
        print("Finish Message!!!")  
############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":   
    rospy.init_node('Bug0') # Node Name
    try: Bug0()  # Class Name
    except rospy.ROSInterruptException:
        rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")
