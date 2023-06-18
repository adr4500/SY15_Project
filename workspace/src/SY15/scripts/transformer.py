#!/usr/bin/env python3

import numpy as np
import rospy

# Type of input and output messages
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]

def callback(msg):
    coords = []
    rangemin=msg.range_min
    rangemax=msg.range_max

    for i, theta in enumerate(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)):
        r=msg.ranges[i]
        if(0.1<r<0.5) and (rangemin<r<rangemax) :
            coords.append((r*np.cos(theta),r*np.sin(theta)))
    # Create a PointCloud2 message from coordinates
    pc2 = create_cloud(msg.header, PC2FIELDS, [[x,y,0,0] for x,y in coords])
    pub_pc2.publish(pc2)

if __name__ == '__main__':
    rospy.init_node('transformer')
    pub_pc2 = rospy.Publisher('/lidar/points', PointCloud2, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()
