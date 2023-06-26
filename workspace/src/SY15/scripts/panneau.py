#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist,PoseWithCovarianceStamped, Quaternion, Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
import numpy as np
import math
import matplotlib.pyplot as plt
from time import sleep
from sensor_msgs.point_cloud2 import read_points
from numpy import linalg as LA

class VectorPanneau:
    def __init__(self):
        rospy.init_node('vector_panneau', anonymous=True)
        self.suscriber_position = rospy.Subscriber("/cluster_panneau", PointCloud2, self.callback)
        self.publisher_target_check = rospy.Subscriber("/panneau_check",Bool,self.can_sen_coord)
        self.suscriber_position = rospy.Subscriber("/estimation", PoseWithCovarianceStamped, self.pose_callback)
        self.target_publisher = rospy.Publisher('/target',Pose,queue_size=10)
        self.send_coord = True
        self.vectors = []
        self.x = []
        self.y = []
        self.robot_coord = np.array([0,0])
        

    def pose_callback(self, pose : PoseWithCovarianceStamped):
        self.robot_coord[0] = pose.pose.pose.position.x
        self.robot_coord[1] = pose.pose.pose.position.y

    def can_sen_coord(self, check:Bool):
        self.send_coord = True
        self.vectors = []
        self.x = []
        self.y = []


    def send_coord_move(self):
        #si on a 10 valeurs on fait tout les calculs pour publish les coord
        if len(self.vectors) > 10 :
            #calcul vecteur moyen
            vx = []
            vy = []
            for i in range(len(self.vectors)):
                vx.append(self.vectors[i][0])
                vy.append(self.vectors[i][1])
            
            #calcul centre de gravité
            center = np.array([np.mean(self.x),np.mean(self.y)])
            print("Le centre de gravité est : "+str(center))
            vector_1 = 0.22*np.array([np.mean(vx),np.mean(vy)])
            vector_2 = -0.22*np.array([np.mean(vx),np.mean(vy)])
            vector_1 += center
            vector_2 += center
            print("Le nouveau point 1 est : "+str(vector_1))
            print("Le nouveau point 2 est : "+str(vector_2))
            dist_1 = np.sqrt(vector_1[0]**2+vector_1[1]**2)
            dist_2 = np.sqrt(vector_2[0]**2+vector_2[1]**2)
            if dist_1 < dist_2 : 
                vector = vector_1
            else : 
                vector = vector_2
            #publier
            print("Le nouveau point repere robot est : "+str(vector))
            vector += self.robot_coord
            print("Le nouveau point repere global est : "+str(vector))
            
            self.target_publisher.publish(Pose(Point(vector[0],vector[1],0),Quaternion()))
            self.send_coord = False
        else : 
            return

    def callback(self, msg : PointCloud2):
        
        if not self.send_coord:
            # print("attente de recherche de panneau !")
            return
        
        standardized_data = []
        for x,y,z,c in read_points(msg):
            standardized_data.append([x,y])
            self.x.append(x)
            self.y.append(y)
        
        if standardized_data != []:
            covariance_matrix = np.cov(standardized_data, ddof = 0, rowvar = False)
            eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)

            index = np.argmin(eigenvalues)
            vector = eigenvectors[index]
            self.vectors.append(vector)
        
        self.send_coord_move()

        
        


    def run(self):
        rospy.spin()
        

if __name__ == '__main__':
    try:
        controller = VectorPanneau()
        controller.run()
    except rospy.ROSInterruptException:
        pass
