#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist,PoseWithCovarianceStamped, Pose
from std_msgs.msg import Bool
import numpy as np
import math
import matplotlib.pyplot as plt
from time import sleep

class TrajectoryControllerNode:
    def __init__(self):
        rospy.init_node('trajectory_controller_node', anonymous=True)
        self.suscriber_position = rospy.Subscriber("/estimation", PoseWithCovarianceStamped, self.pose_callback)
        self.suscriber_target = rospy.Subscriber("/target", Pose, self.set_target)
        self.publisher_target_check = rospy.Publisher("/state_check",Bool,queue_size=1)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.desired_trajectory = []  # Trajectory to follow
        self.point_trajectoire = 0
        self.reglage_angulaire = False
        self.reglage_lineaire = False
        self.est_marche_arriere = False

        self.first_passage = True

        self.axe_deplacement = 'x'


    def set_target(self, pose:Pose):
        self.desired_trajectory = []
        self.desired_trajectory.append((pose.position.x,pose.position.y))
        self.first_passage = True

    def pose_callback(self, pose:PoseWithCovarianceStamped):
        # Calculate control command based on current pose and desired trajectory
        # Implement your control algorithm here
        # Example: proportional control based on position difference
        # You might need to adjust the gains (kp_linear, kp_angular) for your system
        
        if not self.desired_trajectory:
            # print("attente de nouveau point !")
            return
        
        kp_linear = 0.3
        kp_angular = 0.5 


        # Get current robot pose
        current_pose = pose.pose.pose

        # Get desired pose from trajectory 
        #A MODIFIER avec l'ensemble de points
        #marche qu'avec des coord positive ?
        desired_pose_coord_x = self.desired_trajectory[self.point_trajectoire][0]
        desired_pose_coord_y = self.desired_trajectory[self.point_trajectoire][1]

        # print("Point visé : ("+str(desired_pose_coord_x)+","+str(desired_pose_coord_y)+")")
        # print("Point actuelle : ("+str(current_pose.position.x)+","+str(current_pose.position.y)+")")

        # Calculate position difference
        dx = desired_pose_coord_x - current_pose.position.x
        dy = desired_pose_coord_y - current_pose.position.y


        # Calculate heading difference
        desired_heading = math.atan2(dy, dx)
        #self.vect_desired.append(desired_heading)

        #print("Angle desiré : "+str(desired_heading))
        current_heading = 2 * math.atan2(current_pose.orientation.z, current_pose.orientation.w)
        #self.vect_current.append(current_heading)
        #print("Angle courant : "+str(current_heading))
        # Calculate linear and angular velocity commands
        diff_angle = desired_heading - current_heading + math.pi
        diff_angle = diff_angle % (2*math.pi)
        diff_angle = diff_angle - math.pi
        
        #self.vect_diff.append(diff_angle)
        #print("Angle difference : "+str(diff_angle))
        #tant que la diff d'angle est sup à 5 degre
        if self.first_passage == True :
            # print("Premier passage")
            #on est deja passé dans le premier passage
            self.first_passage = False
            #on l'axe de déplacement
            if abs(dy) > abs(dx) :
                if dy > 0:
                    self.axe_deplacement = '+y'
                else : 
                    self.axe_deplacement = '-y'
    
            else :
                if dx > 0:
                    self.axe_deplacement = '+x'
                else : 
                    self.axe_deplacement = '-x'
            #si on a un angle qui vaut plus de 120° on a un demi-tour
            #if abs(diff_angle) > (2/3 * 3.1415):
                #self.est_marche_arriere = True
            

        if(abs(diff_angle) > 0.08 and self.reglage_angulaire == False and self.reglage_lineaire == False and self.est_marche_arriere == False):
            # print("Reglage angulaire 1")
            linear_velocity = 0
            angular_velocity = kp_angular * (diff_angle)
        #on passe va ensuite au point
        else :
            self.reglage_angulaire = True
        
        if self.reglage_angulaire == True and self.reglage_lineaire == False:
            # print("Reglage lineaire")
            if(self.est_marche_arriere == False):
                linear_velocity = kp_linear * math.sqrt(dx**2 + dy**2)
                angular_velocity = kp_angular * (diff_angle)
            else:
                linear_velocity =  - kp_linear * math.sqrt(dx**2 + dy**2)
                angular_velocity = - kp_angular * (diff_angle)
            #on garde la composante angulaire au cas ou des ajustement à faire
            

        if abs(dx)<0.01 and abs(dy)<0.01:
            self.reglage_lineaire = True

        if self.reglage_lineaire == True  : 
            
            #deuxieme réglage angulaire
            #suivant l'axe de déplacement on vise angulairement un point plus loin sur le même axe
            if self.est_marche_arriere == True : 
                if self.axe_deplacement == '+x':
                    self.axe_deplacement == '-x'
                elif self.axe_deplacement == '-x':
                    self.axe_deplacement == '+x'
                elif self.axe_deplacement == '+y':
                    self.axe_deplacement == '-y'
                elif self.axe_deplacement == '-y':
                    self.axe_deplacement == '+y'

            if self.axe_deplacement == '+x':
                desired_pose_coord_x = self.desired_trajectory[self.point_trajectoire][0] + 1000
                desired_pose_coord_y = self.desired_trajectory[self.point_trajectoire][1]
            elif self.axe_deplacement == '-x':
                desired_pose_coord_x = self.desired_trajectory[self.point_trajectoire][0] - 1000
                desired_pose_coord_y = self.desired_trajectory[self.point_trajectoire][1]
            elif self.axe_deplacement == '+y':
                desired_pose_coord_x = self.desired_trajectory[self.point_trajectoire][0]
                desired_pose_coord_y = self.desired_trajectory[self.point_trajectoire][1] + 1000
            elif self.axe_deplacement == '-y':
                desired_pose_coord_x = self.desired_trajectory[self.point_trajectoire][0]
                desired_pose_coord_y = self.desired_trajectory[self.point_trajectoire][1] - 1000

            

            # print("Nouveau Point visé : ("+str(desired_pose_coord_x)+","+str(desired_pose_coord_y)+")")
            #on refait les calculs
            dx = desired_pose_coord_x - current_pose.position.x
            dy = desired_pose_coord_y - current_pose.position.y
            # Calculate heading difference
            desired_heading = math.atan2(dy, dx)
            #self.vect_desired.append(desired_heading)

            # print("Angle desiré : "+str(desired_heading))
            current_heading = 2 * math.atan2(current_pose.orientation.z, current_pose.orientation.w)
            #self.vect_current.append(current_heading)
            # print("Angle courant : "+str(current_heading))
            # Calculate linear and angular velocity commands
            diff_angle = desired_heading - current_heading + math.pi
            diff_angle = diff_angle % (2*math.pi)
            diff_angle = diff_angle - math.pi
            # print("Angle diff : "+str(diff_angle))
            # print("Reglage angulaire 2")
            linear_velocity = 0
            angular_velocity = kp_angular * (diff_angle)

            if abs(diff_angle) < 0.08:
                #on arrête
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0
                cmd_vel_msg.angular.z = 0
                self.cmd_vel_pub.publish(cmd_vel_msg)
                cmd_state_check = Bool()
                cmd_state_check.data = True
                self.publisher_target_check.publish(cmd_state_check)
                self.desired_trajectory = []
                self.reglage_angulaire = False
                self.reglage_lineaire = False
                # print("Fin")
        
                return

        # Publish Twist message with control commands
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
        cmd_state_check = Bool()
        cmd_state_check.data = False
        self.publisher_target_check.publish(cmd_state_check)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = TrajectoryControllerNode()
        controller.run()
    except rospy.ROSInterruptException:
        pass
