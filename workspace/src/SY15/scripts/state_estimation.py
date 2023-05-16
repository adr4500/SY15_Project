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
        
        self.command_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.receive_input)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.receive_odom)
        self.imu_subscriber = rospy.Subscriber("/imu", Imu, self.receive_imu)
        
        
    def receive_input(self, twist_msg:Twist):
        linear_velocity = twist_msg.linear.x
        angular_velocity = twist_msg.angular.z
        
        # ÉTAPE DE PRÉDICTION
        
        self.publish_estimate()
        
        
    def receive_odom(self, odom_msg:Odometry):
        linear_velocity = odom_msg.twist.twist.linear.x
        angular_velocity = odom_msg.twist.twist.angular.z
        
        # ÉTAPE DE CORRECTION
        

        
    def receive_imu(self, imu_msg:Imu):
        linear_acceleration = imu_msg.linear_acceleration # Attention, contient .x .y et .z
        angular_velocity = imu_msg.angular_velocity.z
        
        # ÉTAPE DE CORRECTION
        
        
    
    # À adapter
    def publish_estimate(self):
        
        x = 0
        y = 0
        theta = 0
        
        covariance = np.zeros((3,3)) # Il ne faut que la partie de la covariance concernant x, y et theta
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_footprint"
        
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