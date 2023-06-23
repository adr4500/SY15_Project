#!/usr/bin/env python3

import numpy as np
import rospy

# Type of input and output messages
from sensor_msgs.point_cloud2 import read_points
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

PC2FIELDS = [PointField('x', 0, PointField.FLOAT32, 1),
             PointField('y', 4, PointField.FLOAT32, 1),
             PointField('z', 8, PointField.FLOAT32, 1),
             PointField('c', 12, PointField.INT16, 1)
]

# Computes the distance from point pM to segment [p1,p2]
def dist_seg(pM, p1, p2):
    vec = p2 - p1
    vec /= np.linalg.norm(vec)
    vecT = np.array([-vec[1], vec[0]])
    return np.abs(np.sum((pM-p1)*vecT))

def dist(point,segment) :
    vectSegment=[segment[1][0]-segment[0][0],segment[1][1]-segment[0][1]]
    vectHyp=[point[0]-segment[0][0],point[1]-segment[0][1]]
    hypothenuse=np.sqrt(vectHyp[0]**2+vectHyp[1]**2)
    cosalpha=(vectSegment[0]*vectHyp[0]+vectSegment[1]*vectHyp[1])/(np.sqrt(vectSegment[0]**2+vectSegment[1]**2)*hypothenuse)
    sinalpha=np.sqrt(1-cosalpha**2)
    return sinalpha*hypothenuse

def rdp(points, eps):
    d=[0 for i in range(len(points))]
    dmax=0
    jmax=0
    for i in range(1,len(points)-1) :
        d[i]=dist(points[i],[points[0],points[len(points)-1]])
    dmax=max(d)
    jmax=d.index(dmax)
    if dmax>eps :
        Pres1=rdp([points[i] for i in range(0,jmax)],eps)
        Pres2=rdp([points[i] for i in range(jmax,len(points))],eps)
        Pres=np.concatenate((Pres1,Pres2))
    else :
        Pres=[points[0],points[len(points)-1]]

    return Pres

def callback(msg, eps):
    clusters = {}
    polylines = MarkerArray()
    for x,y,z,c in read_points(msg):
        if c not in clusters.keys(): clusters[c] = []
        clusters[c].append([x,y])

    for c, points in clusters.items():
        poly = rdp(np.array(points), eps)

        polyline = Marker()
        polyline.header = msg.header
        polyline.id = c
        polyline.type = Marker.LINE_STRIP
        polyline.action = Marker.ADD
        polyline.pose.orientation.w = 1
        polyline.scale.x, polyline.scale.y, polyline.scale.z = eps*2, eps*2, 0.3
        polyline.color.r, polyline.color.g, polyline.color.b, polyline.color.a = 1, 0, 0, 0.5
        polyline.lifetime = rospy.Duration(0.2)
        for x,y in poly:
            point = Point()
            point.x, point.y, point.z = x, y, 1
            polyline.points.append(point)
        polylines.markers.append(polyline)

    pub_polylines.publish(polylines)

if __name__ == '__main__':
    rospy.init_node('shaper_polyline')
    eps = rospy.get_param('~eps', 0.15)
    pub_polylines = rospy.Publisher('/lidar/polylines', MarkerArray, queue_size=10)
    rospy.Subscriber('/lidar/clusters', PointCloud2, callback, eps)
    rospy.spin()
