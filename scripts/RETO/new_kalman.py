#!/usr/bin/env python
# * Import necessary libraries
import rospy
import numpy as np
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker 

r = 0.05
L = 0.18

class KF_NODE():
    def __init__(self):
        #?#********** PUBLISHERS #?#**********
        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=1)
        self.pub_pose = rospy.Publisher("/position", Point, queue_size=1)
        # self.marker_pub = rospy.Publisher("perfect_marker", Marker, queue_size=1)
        # self.pub_marker = rospy.Publisher("/marker", # TODO Marker)
        #?#********** SUBSCRIBERS #?#**********
        rospy.Subscriber("wl", Float32, self.get_wl)
        rospy.Subscriber("wr", Float32, self.get_wr)
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.get_fiducial)
        
        # TODO Crear variables y constantes
        #?#********** VARIABLES #?#**********
        odom = Odometry()
        init_time = rospy.get_time() # Tiempo de inicio del filtro
        rate = rospy.Rate(20) # The rate of the while loop 
        
        self.wl = 0.0
        self.received_wl = False

        self.wr = 0.0
        self.received_wr = False

        dt = 0.0 # delta T
        v = 0.0 # Velocidad lineal [m/s]
        w = 0.0 # Velocidad Angular [m/s]

        #?#*********** POSITION #?#***********
        self.x = 1.2 # Posicion Inicial en X del Robot
        self.y = 0.0 # Posicion Inicial en Y del Robot
        self.theta = np.pi/2.0 # Orientacion Inicial en Z del Robot

        self.P = np.zeros([3,3]) # Matriz de Covarianza del Robot

        #?#*********** ARUCOS #?#***********
        self.aruco_x = 0.0
        self.aruco_y = 0.0
        self.aruco_noise = 0.0
        self.aruco_id = 0.0
        self.aruco_detected = False
        self.POS_ARUCOS = {
            701: (0.48,3.15), 
            702: (3.06,3.05),
            703: (1.91,5.44),
            704: (1.43,2.45),
            705: (2.1,0.6),
            706: (1.44,2.3),
        }

        rospy.loginfo("STARTING KF NODE")
        while not rospy.is_shutdown():
            # Revisar valores de las llantas
            if self.received_wl is True and self.received_wr is True:
                # Obtener un valor de delta_T
                current_time = rospy.get_time()
                dt = current_time - init_time

                # Copia de seguridad para las llantas
                wr = self.wr
                wl = self.wl

                # Calcular velocidad Angular y Linear
                v = r*(wr+wl)/2.0 #Robot's linear speed 
                w = r*(wr-wl)/L #Robot's Angular Speed

                self.predict(dt, [wr, wl], [0.087, 0.087], v, w) 

                if self.aruco_detected is True:
                    rospy.logwarn("ARUCO DETECTED UPDATE")
                    self.update()
                    self.aruco_detected = False

                # Actualizar tiempo
                init_time = current_time

                # * Imprimir posicion actual
                posicion = [round(self.x,2), round(self.y, 2), round(self.rad2deg(self.theta), 2)]
                rospy.loginfo("KALMAN POSITION: " + str(posicion))
                robot_pos = Point()
                robot_pos.x = self.x
                robot_pos.y = self.y
                robot_pos.z = self.theta

                # * Publicar valores
                odom = self.fill_odom(robot_pos, self.P, v, w)
                self.pub_odom.publish(odom)
                self.pub_pose.publish(robot_pos)
                rate.sleep()


    #?#********** CALLBACKS #?#********** 
    def get_wl(self, msg=Float32()):
        self.wl = msg.data
        self.received_wl = True

    def get_wr(self, msg=Float32()):
        self.wr = msg.data
        self.received_wr = True

    def get_fiducial(self, msg=FiducialTransformArray()):
        """ Obtiene los datos de un marcador aruco """
        for fiducial in msg.transforms:
            # fiducial = FiducialTransform()
            if fiducial.fiducial_id in self.POS_ARUCOS:
                self.aruco_x = fiducial.transform.translation.z + 0.09
                self.aruco_y = - fiducial.transform.translation.x
                self.aruco_noise = fiducial.object_error
                self.aruco_id = fiducial.fiducial_id
                self.aruco_detected = True
                break

    #?#********** FUNCIONES FILTRO DE KALMAN #?#********** 
    def predict(self, dt, w_llantas, e_llantas, v, w):
        """ Estimar nuevas posiciones en el sistema de coordenadas"""
        # * Desempaquetar variables
        kr = e_llantas[0]
        kl = e_llantas[1]
        wr = w_llantas[0]
        wl = w_llantas[1]

        #?#********** CALCULAR MATRIZ DE HACOBIANO #?#********** 
        comp_H1 = -dt * v * np.sin(self.theta)
        comp_H2 = dt * v * np.cos(self.theta)
        H = np.array([
            [1.0, 0.0, comp_H1  ],
            [0.0, 1.0, comp_H2  ],
            [0.0, 0.0,   1.0    ]
        ])

        #?#********** CALCULAR MATRIZ DE RUIDO #?#********** 
        S_ruido = np.array([
            [kr * abs(wr), 0.0],
            [0.0, kl * abs(wl)]
        ])
        H_ruido = np.array([
            [np.cos(self.theta), np.cos(self.theta)],
            [np.sin(self.theta), np.sin(self.theta)],
            [2.0/L,        -2.0/L]
        ])
        H_ruido_mult = 0.5 * r * dt * H_ruido
        Qk = np.array(H_ruido_mult.dot(S_ruido).dot(H_ruido_mult.T))

        #?#********** CALCULAR COVARIANZA #?#**********
        self.P = np.array(H.dot(self.P).dot(H.T) + Qk)

        #?#********** CALCULAR NUEVAS POSICIONES #?#**********
        self.x = self.x + dt * v * np.cos(self.theta)
        self.y = self.y + dt * v * np.sin(self.theta)
        self.theta = self.theta + dt * w
        self.theta = self.crop_angle(self.theta)
    

    def update(self):
        NR_pos = np.array([
            [self.x],
            [self.y],
            [self.theta]
        ])

        #?#********** CALCULAR LOS DELTAS #?#**********
        coord_aruco = self.POS_ARUCOS[self.aruco_id]
        delta_x = coord_aruco[0] - self.x
        delta_y = coord_aruco[1] - self.y
        rho = delta_x**2 + delta_y**2

        #?#********** OBSERVACION ESTIMADA #?#**********
        z_rho_hat = np.sqrt(rho)
        z_theta_hat = np.arctan2(delta_y, delta_x) - self.theta
        z_theta_hat = self.crop_angle(z_theta_hat)
        z_hat = np.array([
            [z_rho_hat],
            [z_theta_hat]
        ])
        #?#********** OBSERVACION ARUCO #?#**********
        z_rho = np.sqrt(self.aruco_x**2 + self.aruco_y**2)
        z_theta = np.arctan2(self.aruco_y, self.aruco_x)
        z_aruco = np.array([
            [z_rho],
            [z_theta]
        ])

        #?#********** HACOBIANO UPDATE #?#**********
        H = np.array([
            [(-delta_x)/np.sqrt(rho), (-delta_y)/np.sqrt(rho), 0.0],
            [       delta_y/rho,            (-delta_x)/rho  , -1.0],
        ])

        #?#********** RUIDO ARUCO #?#**********
        Rk = np.array([
            [self.aruco_noise, 0.0],
            [0.0, self.aruco_noise]
        ])

        #?#********** ZETA DE UPDATE #?#**********
        Z = H.dot(self.P).dot(H.T) + Rk

        #?#********** GANANCIA DE KALMAN #?#**********
        K = self.P.dot(H.T).dot(np.linalg.inv(Z))

        #?#********** CORREGIR POSICIONES #?#**********
        NR_pos = NR_pos + K.dot(z_aruco - z_hat)
        self.x = NR_pos[0][0]
        self.y = NR_pos[1][0]
        self.theta = NR_pos[2][0]
        self.theta = self.crop_angle(self.theta)

        self.P = (np.eye(3) - K.dot(H)).dot(self.P)


    #?#********** HELPERS #?#**********
    def crop_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle)) 
    def rad2deg(self, angle):
        return angle*180.0/np.pi
    def fill_odom(self, position, covarianza, v, w):
        odom=Odometry() 
        odom.header.stamp =rospy.Time.now() 
        odom.header.frame_id = "odom" 
        odom.child_frame_id = "base_link" 
        odom.pose.pose.position.x = position.x 
        odom.pose.pose.position.y = position.y 
        odom.pose.pose.position.z = 0.0 
        quat=quaternion_from_euler(0.0, 0.0, position.z) 

        odom.pose.pose.orientation.x = quat[0] 
        odom.pose.pose.orientation.y = quat[1] 
        odom.pose.pose.orientation.z = quat[2] 
        odom.pose.pose.orientation.w = quat[3] 
        odom.pose.covariance = [0.0]*36 

        # Fill the covariance matrix 
        odom.pose.covariance[0] = covarianza[0,0] 
        odom.pose.covariance[1] = covarianza[0,1] 
        odom.pose.covariance[5] = covarianza[0,2] 
        odom.pose.covariance[6] = covarianza[1,0] 
        odom.pose.covariance[7] = covarianza[1,1] 
        odom.pose.covariance[11] = covarianza[1,2] 
        odom.pose.covariance[30] = covarianza[2,0] 
        odom.pose.covariance[31] = covarianza[2,1] 
        odom.pose.covariance[35] = covarianza[2,2] 

        odom.twist.twist.linear.x = v 
        odom.twist.twist.angular.z = w
        # print(self.x,self.y,self.theta) 
        return odom 
    
    def fill_marker_perfect(self, odom = Odometry()):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()

        marker.type = 10
        marker.id = 1

        marker.mesh_resource = "package://puzzlebot_sim/descriptions/MCR2_1000_0_Puzzlebot.stl" 

        # Set the scale of the marker 
        marker.scale.x = 1 
        marker.scale.y = 1 
        marker.scale.z = 1 

        # Set the color 
        marker.color.r = 1.0 
        marker.color.g = 0.0 
        marker.color.b = 1.0 
        marker.color.a = 1.0     

            # Set the pose of the marker 
        marker.pose.position.x = odom.pose.pose.position.x
        marker.pose.position.y = odom.pose.pose.position.y
        marker.pose.position.z = 0.0 

        # Set the marker orientation 
        #quat=quaternion_from_euler(0.0,0.0,theta) 
        marker.pose.orientation.x = odom.pose.pose.orientation.x
        marker.pose.orientation.y = odom.pose.pose.orientation.y
        marker.pose.orientation.z = odom.pose.pose.orientation.z
        marker.pose.orientation.w = odom.pose.pose.orientation.w

        return marker


if __name__ == "__main__":
    rospy.init_node('KALMANFILTER') # Node Name
    try: KF_NODE()  # Class Name
    except rospy.ROSInterruptException:
        rospy.logwarn("EXECUTION COMPELTED SUCCESFULLY")
