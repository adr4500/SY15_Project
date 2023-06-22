#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion, Point
from visualization_msgs.msg import MarkerArray, Marker
from enum import Enum
from std_msgs.msg import Bool

# Enum of directions
Direction = Enum('Direction',['UP','RIGHT','DOWN','LEFT'])

State = Enum('State',['MOVING_TO_EDGE','WAITING','MOVING_TO_CENTER','MAPPING'])

class Tile :
    '''This class represents one cell of the labyrinth, the labyrinth is defined recursively as a Tile tree'''
    def __init__(self,parent=None):
        self.corners=[None,None,None,None] # [top_left,top_right,bottom_left,bottom_right]
        self.parent = parent
        self.parent_direction = None
        self.children = [None,None,None,None] # [up,right,down,left]
        self.visited = False
    
    def get_center(self):
        '''Returns the center of the cell'''
        if None in self.corners :
            raise ValueError("The corners of the cell are not defined")
        else :
            return np.mean(self.corners,axis=0)
    
    def get_corners(self):
        '''Returns the corners of the cell'''
        return self.corners
    
    def set_corner(self,index,corner):
        '''Sets the corner of the cell at the given index'''
        self.corners[index]=corner

    def create_child(self,direction):
        '''Creates a child cell in the given direction'''
        if self.children[direction] is not None :
            raise ValueError("The child cell already exists")
        else :
            self.children[direction] = Tile(self)

            # Connect the tiles
            if direction == Direction.UP :
                self.children[direction].set_corner(2,self.corners[0])
                self.children[direction].set_corner(3,self.corners[1])
                self.children[direction].parent_direction = Direction.DOWN
            elif direction == Direction.RIGHT :
                self.children[direction].set_corner(0,self.corners[1])
                self.children[direction].set_corner(2,self.corners[3])
                self.children[direction].parent_direction = Direction.LEFT
            elif direction == Direction.DOWN :
                self.children[direction].set_corner(0,self.corners[2])
                self.children[direction].set_corner(1,self.corners[3])
                self.children[direction].parent_direction = Direction.UP
            elif direction == Direction.LEFT :
                self.children[direction].set_corner(1,self.corners[0])
                self.children[direction].set_corner(3,self.corners[2])
                self.children[direction].parent_direction = Direction.RIGHT
        
    def extrapolate_corners(self):
        # If two diagonal corners are defined, extrapolate orthogonaly the two others
        if self.corners[0] is not None and self.corners[3] is not None :
            if self.corners[1] is None :
                self.corners[1][0] = self.corners[0][0]
                self.corners[1][1] = self.corners[3][1]
            if self.corners[2] is None :
                self.corners[2][0] = self.corners[3][0]
                self.corners[2][1] = self.corners[0][1]
        elif self.corners[1] is not None and self.corners[2] is not None :
            if self.corners[0] is None :
                self.corners[0][0] = self.corners[1][0]
                self.corners[0][1] = self.corners[2][1]
            if self.corners[3] is None :
                self.corners[3][0] = self.corners[2][0]
                self.corners[3][1] = self.corners[1][1]
    
    def get_parent_edge(self):
        '''Returns the entrance of the parent cell'''
        if self.parent is None :
            raise ValueError("The cell has no parent")
        else :
            if self.parent_direction == Direction.UP :
                return np.mean([self.corners[2],self.corners[3]],axis=0)
            elif self.parent_direction == Direction.RIGHT :
                return np.mean([self.corners[0],self.corners[2]],axis=0)
            elif self.parent_direction == Direction.DOWN :
                return np.mean([self.corners[0],self.corners[1]],axis=0)
            elif self.parent_direction == Direction.LEFT :
                return np.mean([self.corners[1],self.corners[3]],axis=0)

    
class Labyrinth_Solver:
    '''Ros node used to map and solve'''
    def __init__(self):
        rospy.init_node('lab_solver')
        
        self.first_tile = None
        self.current_tile = None
        self.tile_size = None

        self.robot_pos = None

        self.state = None
        
        self.lidar_subscriber = rospy.Subscriber('/lidar/polylines',MarkerArray,self.lidar_callback)
        self.target_publisher = rospy.Publisher('/target',Pose,queue_size=10)
        self.pos_subscriber = rospy.Subscriber("/estimation", PoseWithCovarianceStamped, self.pos_callback)
        self.state_subscription = rospy.Subscriber("/state_check",Bool,self.state_callback)

        rospy.spin()
    
    def state_callback(self,data):
        '''Callback function for the state'''
        if not data.data :
            # Next State
            if self.state == State.MOVING_TO_EDGE :
                self.state = State.WAITING
            elif self.state == State.MOVING_TO_CENTER :
                self.state = State.MAPPING

    
    def lidar_callback(self,data):
        '''Callback function for the lidar'''
        # If the robot is moving, wait for it to stop
        if self.state == State.MOVING_TO_EDGE or self.state == State.MOVING_TO_CENTER or self.robot_pos is None :
            return
        
        # Initialization of the first tile
        if self.first_tile is None :
            self.first_tile = Tile()
            # Detect entrance : two alligned segments with a gap between them
            if len(data.markers) != 2 :
                print("WARNING : {} Markers detected")
                raise ValueError("The labyrinth entrance is not detected")
            else :
                # Take the closest point to the robot of each segment
                first_segment = [data.markers[0].points[0],data.markers[0].points[-1]]
                second_segment = [data.markers[1].points[0],data.markers[1].points[-1]]
                first_segment_distances = [np.linalg.norm(np.array([self.robot_pos.pose.position.x,self.robot_pos.pose.position.y])-np.array([point.x,point.y])) for point in first_segment]
                second_segment_distances = [np.linalg.norm(np.array([self.robot_pos.pose.position.x,self.robot_pos.pose.position.y])-np.array([point.x,point.y])) for point in second_segment]

                entrance_points = []
                if first_segment_distances[0] < first_segment_distances[1] :
                    entrance_points.append(np.array([first_segment[0].x,first_segment[0].y]))
                else :
                    entrance_points.append(np.array([first_segment[1].x,first_segment[1].y]))

                if second_segment_distances[0] < second_segment_distances[1] :
                    entrance_points.append(np.array([second_segment[0].x,second_segment[0].y]))
                else :
                    entrance_points.append(np.array([second_segment[1].x,second_segment[1].y]))
                
                # Sort entrance points by x coordinate
                entrance_points.sort(key=lambda point: point[1])

                # Set the corners of the first tile
                self.first_tile.set_corner(2,[entrance_points[0][0],entrance_points[0][1]])
                self.first_tile.set_corner(3,[entrance_points[1][0],entrance_points[1][1]])
                self.first_tile.parent_direction = Direction.DOWN

                # Initialize tile size from the entrance
                self.tile_size = np.linalg.norm(entrance_points[0]-entrance_points[1])

                # Go to entrance
                target_coord = np.mean(entrance_points,axis=0)
                target = Pose(Point(target_coord[0],target_coord[1],0),Quaternion())
                print(target)
                self.target_publisher.publish(target)
                self.current_tile = self.first_tile
                self.state = State.MOVING_TO_EDGE
                self.current_tile.visited = True
                return
        
        else :
            if self.state == State.WAITING :
                # Bring the robot to the center of the tile
                direction = self.current_tile.parent_direction
                distance = self.tile_size/2
                pos = np.array([self.robot_pos.pose.position.x,self.robot_pos.pose.position.y])
                if direction == Direction.UP :
                    target_coord = pos + np.array([0,distance])
                elif direction == Direction.RIGHT :
                    target_coord = pos + np.array([distance,0])
                elif direction == Direction.DOWN :
                    target_coord = pos + np.array([0,-distance])
                elif direction == Direction.LEFT :
                    target_coord = pos + np.array([-distance,0])
                target = Pose(Point(target_coord[0],target_coord[1],0),Quaternion())
                self.target_publisher.publish(target)
                self.state = State.MOVING_TO_CENTER
                return
            elif self.state == State.MAPPING :
                # Get the closest points in each quadrant
                quadrants = [[],[],[],[]] # [top_left,top_right,bottom_left,bottom_right]
                walls = [False,False,False,False]
                for marker in data.markers :
                    points_in_quad = [False,False,False,False]
                    for point in marker.points :
                        # Change base from robot coords to general coords
                        point = np.array([point.x,point.y])
                        # Rotate point
                        point = np.dot(np.array([[np.cos(self.robot_pos.pose.orientation.z),-np.sin(self.robot_pos.pose.orientation.z)],[np.sin(self.robot_pos.pose.orientation.z),np.cos(self.robot_pos.pose.orientation.z)]]),point)
                        point = point + np.array([self.robot_pos.pose.position.x,self.robot_pos.pose.position.y])
                        if point[0] > self.robot_pos.pose.position.x :
                            if point[1] > self.robot_pos.pose.position.y :
                                quadrants[0].append(point)
                                points_in_quad[0] = True
                            else :
                                quadrants[3].append(point)
                                points_in_quad[3] = True
                        else :
                            if point[1] > self.robot_pos.pose.position.y :
                                quadrants[1].append(point)
                                points_in_quad[1] = True
                            else :
                                quadrants[2].append(point)
                                points_in_quad[2] = True
                    if points_in_quad[0] and points_in_quad[1] :
                        walls[Direction.UP] = True
                    if points_in_quad[1] and points_in_quad[3] :
                        walls[Direction.RIGHT] = True
                    if points_in_quad[2] and points_in_quad[3] :
                        walls[Direction.DOWN] = True
                    if points_in_quad[0] and points_in_quad[2] :
                        walls[Direction.LEFT] = True

                
                # Sort points by distance to the robot
                for quadrant in quadrants :
                    quadrant.sort(key=lambda point: np.linalg.norm(np.array([self.robot_pos.pose.position.x,self.robot_pos.pose.position.y])-np.array([point.x,point.y])))
                
                # Get the closest point in each quadrant
                closest_points = []
                for quadrant in quadrants :
                    closest_points.append(quadrant[0])
                
                # Delete points that are too far
                for i in range(len(closest_points)):
                    if np.linalg.norm(np.array([self.robot_pos.pose.position.x,self.robot_pos.pose.position.y])-np.array([closest_points[i].x,closest_points[i].y])) > self.tile_size/2 :
                        closest_points[i] = None
                
                # Update the corners of the current tile
                for i in range(len(closest_points)):
                    if closest_points[i] is not None :
                        self.current_tile.set_corner(i,closest_points[i])
                self.current_tile.extrapolate_corners()

                # If all corners are defined, create the children
                if None not in self.current_tile.get_corners() :
                    for direction in Direction :
                        if not walls[direction]:
                            self.current_tile.create_child(direction)
                
                # Decide to move :
                # If tile is mapped, go to child
                if None not in self.current_tile.get_corners() :
                    for child in self.current_tile.children :
                        if child is not None :
                            if not child.visited :
                                self.current_tile = child
                                self.state = State.MOVING_TO_EDGE
                                target_coord = self.current_tile.get_parent_edge()
                                target = Pose(Point(target_coord[0],target_coord[1],0),Quaternion())
                                self.target_publisher.publish(target)
                                return
                    # If all children are visited, go to parent
                    self.current_tile = self.current_tile.parent
                    self.state = State.MOVING_TO_CENTER
                    target_coord = self.current_tile.get_center()
                    target = Pose(Point(target_coord[0],target_coord[1],0),Quaternion())
                    self.target_publisher.publish(target)
                    return
                else :
                    pass

        


    def pos_callback(self,data):
        '''Callback function for the robot position'''
        self.robot_pos = data.pose
if __name__ == "__main__":
    node=Labyrinth_Solver()
