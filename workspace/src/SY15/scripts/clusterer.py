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
    for i in range(points.shape[0]) :
        if groups[i]!=0 and np.count_nonzero(groups==groups[i])>3:
            P.append(points[i])
            C.append(groups[i])

    clust_msg = create_cloud(msg.header, PC2FIELDS, [[P[i][0],P[i][1],0,c] for i,c in enumerate(C)])
    pub_clusters.publish(clust_msg)

if __name__ == '__main__':
    rospy.init_node('clusterer')
    pub_clusters = rospy.Publisher('/lidar/clusters', PointCloud2, queue_size=10)
    rospy.Subscriber('/lidar/points', PointCloud2, callback)
    rospy.spin()
