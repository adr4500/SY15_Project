#!/usr/bin/env python3

import numpy as np
import rospy

# Type of input and output messages
from sensor_msgs.point_cloud2 import create_cloud, read_points
from sensor_msgs.msg import PointCloud2, PointField

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]

def callback(msg):
    points = np.array(list(read_points(msg)))[:,:2]
    intensities = np.array(list(read_points(msg)))[:,3]
    groups = np.zeros(points.shape[0], dtype=int)

    # ToDo: Determine k and D values
    k = 8
    D = 0.2
    
    # ToDo: Clustering algorithm
    for i in range(k, points.shape[0]):
        d=[D+1 for i in range(k)]
        for j in range(1,k) :
            d[j-1]=np.sqrt((points[i,0]-points[i-j,0])**2+(points[i,1]-points[i-j,1])**2)
        dmin=min(d)
        jmin=d.index(dmin)+1
        
        if dmin<D :
            if groups[i-jmin]==0 :
                groups[i-jmin]=max(groups)+1
            groups[i]=groups[i-jmin]
    P=[]
    C=[]
    I=[]
    for i in range(points.shape[0]) :
        #intensity_max = 0
        #group_max = 0
        #if intensities[i] > intensity_max :
        #        intensity_max = intensities[i]
        #        group_max = groups[i]
        if groups[i]!=0 and np.count_nonzero(groups==groups[i])>3:
            
            P.append(points[i])
            C.append(groups[i])
            I.append(intensities[i])


    moy_intensities = 0
    nb_intensities = 0
    tab_moy_intensities = []
    clusters = []

    for i in range(len(C)):
        #cas fin de liste
        if i == len(C) - 1 :
            cluster_current = 0
        else :
            cluster_current = C[i]
        if i != 0 : 
            cluster_precedent = C[i-1]
        else : 
            cluster_precedent = C[i]

        if cluster_current == cluster_precedent :
            moy_intensities += I[i]
            nb_intensities += 1
        else : 
            if nb_intensities!= 0:
                tab_moy_intensities.append(moy_intensities/nb_intensities)
                clusters.append(cluster_precedent)
                moy_intensities = 0
                nb_intensities = 0

    tab_moy_intensities = np.array(tab_moy_intensities)
    clusters = np.array(clusters)

    #print(str(tab_moy_intensities))
    #print(str(clusters))

    indice_max = np.argmax(tab_moy_intensities)
    group_max = clusters[indice_max]

    P_panneau = []
    C_panneau = []
    for i in range(len(C)):
        if C[i] == group_max:
            P_panneau.append(P[i])
            C_panneau.append(C[i])

    clust_msg = create_cloud(msg.header, PC2FIELDS, [[P[i][0],P[i][1],0,c] for i,c in enumerate(C)])
    clust_msg_panneau = create_cloud(msg.header, PC2FIELDS, [[P_panneau[i][0],P_panneau[i][1],0,c] for i,c in enumerate(C_panneau)])
    pub_clusters.publish(clust_msg)
    pub_panneau.publish(clust_msg_panneau)

if __name__ == '__main__':
    rospy.init_node('clusterer')
    pub_clusters = rospy.Publisher('/lidar/clusters_docking', PointCloud2, queue_size=10)
    pub_panneau = rospy.Publisher('/cluster_panneau', PointCloud2, queue_size=10)
    rospy.Subscriber('/lidar/points_docking', PointCloud2, callback)
    rospy.spin()