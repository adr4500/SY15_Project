#!/usr/bin/python3

import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class StateEstimation:
    def __init__(self):
        rospy.init_node("state_estimation", log_level=rospy.INFO)

        rospy.loginfo("Démarrage du nœud d'estimation d'état")
        
        self.estimate_publisher = rospy.Publisher("estimation", PoseWithCovarianceStamped, queue_size=1)

        
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.receive_odom)
        self.imu_subscriber = rospy.Subscriber("/imu", Imu, self.receive_imu)

        # Matrices
        self.P = np.eye(5) * 0.00001 # Matrice de covariance de l'état
        print("INIT")
        print(self.P)
        self.X = np.zeros((5,1)) # Vecteur d'état
        self.F = np.eye(5) # Matrice de transition d'état
        self.C = np.zeros((2,5)) # Matrice de transition de mesure
        self.Q = np.eye(5) * 0.000001 # Matrice de covariance du bruit de processus
        self.R = np.eye(2) # Matrice de covariance du bruit de mesure

        self.R[0,0] = 0.001
        self.R[1,1] = 0.00001
        # Initialisation des matrices
        self.C[0,3] = 1
        self.C[1,4] = 1

        self.K = self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.R) # Matrice de gain de Kalman

        self.last_time = rospy.Time.now().to_sec()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.predict)
        
    def predict(self,event_timer):

        new_time = rospy.Time.now().to_sec()

        # Calcul de la matrice F
        self.F[0,2] = -self.X[3] * math.sin(self.X[2]) * (new_time - self.last_time)
        self.F[0,3] = math.cos(self.X[2]) * (new_time - self.last_time)
        self.F[1,2] = self.X[3] * math.cos(self.X[2]) * (new_time - self.last_time)
        self.F[1,3] = math.sin(self.X[2]) * (new_time - self.last_time)
        self.F[2,4] = (new_time - self.last_time)

        # Mise a jour du vecteur d'état
        self.X[0] = self.X[0] + self.X[3] * math.cos(self.X[2]) * (new_time - self.last_time)
        self.X[1] = self.X[1] + self.X[3] * math.sin(self.X[2]) * (new_time - self.last_time)
        self.X[2] = self.X[2] + self.X[4] * (new_time - self.last_time)

        self.last_time = new_time
        
        # Calcul de la matrice P
        
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        self.publish_estimate()
        
        
    def receive_odom(self, odom_msg:Odometry):
        
        linear_velocity = odom_msg.twist.twist.linear.x
        angular_velocity = odom_msg.twist.twist.angular.z
        if abs(angular_velocity) > 2 :
            return
        
        self.X = self.X + self.K @ (np.array([[linear_velocity], [angular_velocity]]) - self.C @ self.X)
        self.P = (np.eye(5) - self.K @ self.C) @ self.P
        self.K = self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.R)
        
        
        
    def receive_imu(self, imu_msg:Imu):
        linear_acceleration = imu_msg.linear_acceleration # Attention, contient .x .y et .z
        angular_velocity = imu_msg.angular_velocity.z
        
        # Non utilisé
        
        
    
    # À adapter
    def publish_estimate(self):
        
        x = self.X[0]
        y = self.X[1]
        theta = self.X[2]
        
        covariance = self.P[0:3,0:3]
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        
        msg.pose.pose.orientation.x = 0
        msg.pose.pose.orientation.y = 0
        msg.pose.pose.orientation.z = math.sin(theta / 2)
        msg.pose.pose.orientation.w = math.cos(theta / 2)

        msg.pose.covariance[ 0] = covariance[0][0]
        msg.pose.covariance[ 1] = covariance[0][1]
        msg.pose.covariance[ 5] = covariance[0][2]
        msg.pose.covariance[ 6] = covariance[1][0]
        msg.pose.covariance[ 7] = covariance[1][1]
        msg.pose.covariance[11] = covariance[1][2]
        msg.pose.covariance[30] = covariance[2][0]
        msg.pose.covariance[31] = covariance[2][1]
        msg.pose.covariance[35] = covariance[2][2]
        
        self.estimate_publisher.publish(msg)
    
    
    def run(self):
        rospy.spin()
        
        
if __name__ == "__main__":
    node = StateEstimation()
    node.run()
