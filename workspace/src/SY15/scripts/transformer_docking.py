#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float32

# Type of input and output messages
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]

sight = 10

def callback(msg):
    global sight
    coords = []
    rangemin=msg.range_min
    rangemax=msg.range_max

    
    for i, theta in enumerate(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)):
        r=msg.ranges[i]
        intensity = msg.intensities[i]
        if(0.1<r<sight) and (rangemin<r<rangemax) :
            coords.append((r*np.cos(theta),r*np.sin(theta),intensity))

    # Create a PointCloud2 message from coordinates
    pc2 = create_cloud(msg.header, PC2FIELDS, [[x,y,0,int(inten)] for x,y,inten in coords])
    #print(intensity)
    pub_pc2.publish(pc2)

def update_sight(msg):
    global sight
    sight = msg.data

if __name__ == '__main__':
    rospy.init_node('transformer')
    pub_pc2 = rospy.Publisher('/lidar/points_docking', PointCloud2, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.Subscriber('/sight',Float32,update_sight)
    rospy.spin()
