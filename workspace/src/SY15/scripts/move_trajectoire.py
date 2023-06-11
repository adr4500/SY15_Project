#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist,PoseWithCovarianceStamped
import numpy as np
import math

class TrajectoryControllerNode:
    def __init__(self):
        rospy.init_node('trajectory_controller_node', anonymous=True)
        self.suscriber_position = rospy.Subscriber("/estimation", PoseWithCovarianceStamped, self.pose_callback)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.desired_trajectory = [(1.1,0),(1.3,-0.15),(1.1,-0.4),(0.9,-0.5),(0.7,-0.7),(1.1,-0.8),(2,-0.8),(-1,-1)]  # Trajectory to follow
        self.point_trajectoire = 0

    def pose_callback(self, pose:PoseWithCovarianceStamped):
        # Calculate control command based on current pose and desired trajectory
        # Implement your control algorithm here
        # Example: proportional control based on position difference
        # You might need to adjust the gains (kp_linear, kp_angular) for your system
        kp_linear = 0.5
        kp_angular = 0.5  

        # Get current robot pose
        current_pose = pose.pose.pose

        # Get desired pose from trajectory 
        #A MODIFIER avec l'ensemble de points
        #marche qu'avec des coord positive ?
        desired_pose_coord_x = self.desired_trajectory[self.point_trajectoire][0]
        desired_pose_coord_y = self.desired_trajectory[self.point_trajectoire][1]

        print("Point visé : ("+str(desired_pose_coord_x)+","+str(desired_pose_coord_y)+")")

        # Calculate position difference
        dx = desired_pose_coord_x - current_pose.position.x
        dy = desired_pose_coord_y - current_pose.position.y

        # Calculate heading difference
        desired_heading = math.atan2(dy, dx)
        current_heading = 2 * math.atan2(current_pose.orientation.z, current_pose.orientation.w)

        # Calculate linear and angular velocity commands
        diff_angle = desired_heading - current_heading + math.pi
        diff_angle = diff_angle % (2*math.pi)
        diff_angle = diff_angle - math.pi

        linear_velocity = kp_linear * math.sqrt(dx**2 + dy**2)
        angular_velocity = kp_angular * (diff_angle)

        # Publish Twist message with control commands
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
        print("Les differences x et y : "+str(dx)+","+str(dy))
        if abs(dx)<0.02 and abs(dy)<0.02:
            self.point_trajectoire += 1
            if self.desired_trajectory[self.point_trajectoire][0] == -1:
                print("Fin")
                return

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = TrajectoryControllerNode()
        controller.run()
    except rospy.ROSInterruptException:
        pass
