#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion, Point
from visualization_msgs.msg import MarkerArray, Marker
from enum import Enum
from std_msgs.msg import Bool,Float32
from random import shuffle
import math

FINISH_POINT = np.array([0.0,0.0])

# Enum of directions
Direction = Enum('Direction',['UP','RIGHT','DOWN','LEFT'])

State = Enum('State',['MOVING_TO_EDGE','WAITING','MOVING_TO_CENTER','MAPPING','FINISHED','MOVING_TO_FINISH','OFF'])

class Tile : 
    '''This class represents one cell of the labyrinth, the labyrinth is defined recursively as a Tile tree'''
    def __init__(self,parent=None):
        self.corners=[None,None,None,None] # [top_left,top_right,bottom_left,bottom_right]
        self.parent = parent
        self.parent_direction = None
        self.children = [None,None,None,None] # [up,right,down,left]
        self.visited = False
        self.walls = [False,False,False,False] # [up,right,down,left]
    
    def get_center(self):
        '''Returns the center of the cell'''
        if np.any(self.corners == None) : # TO FIX
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
        if self.children[direction.value-1] is not None :
            raise ValueError("The child cell already exists")
        else :
            self.children[direction.value-1] = Tile(self)

            # Connect the tiles
            if direction == Direction.UP :
                self.children[direction.value-1].set_corner(2,self.corners[0])
                self.children[direction.value-1].set_corner(3,self.corners[1])
                self.children[direction.value-1].parent_direction = Direction.DOWN
            elif direction == Direction.RIGHT :
                self.children[direction.value-1].set_corner(0,self.corners[1])
                self.children[direction.value-1].set_corner(2,self.corners[3])
                self.children[direction.value-1].parent_direction = Direction.LEFT
            elif direction == Direction.DOWN :
                self.children[direction.value-1].set_corner(0,self.corners[2])
                self.children[direction.value-1].set_corner(1,self.corners[3])
                self.children[direction.value-1].parent_direction = Direction.UP
            elif direction == Direction.LEFT :
                self.children[direction.value-1].set_corner(1,self.corners[0])
                self.children[direction.value-1].set_corner(3,self.corners[2])
                self.children[direction.value-1].parent_direction = Direction.RIGHT
            
            print(f"Created child in direction {direction}, coords are {self.children[direction.value-1].get_corners()}")
        
    def extrapolate_corners(self):
        # If two diagonal corners are defined, extrapolate orthogonaly the two others
        if self.corners[0] is not None and self.corners[3] is not None :
            if self.corners[1] is None :
                extrapolated_point = [self.corners[0][0], self.corners[3][1]]
                self.corners[1] = np.array(extrapolated_point)
            if self.corners[2] is None :
                extrapolated_point = [self.corners[3][0], self.corners[0][1]]
                self.corners[2] = np.array(extrapolated_point)
        elif self.corners[1] is not None and self.corners[2] is not None :
            if self.corners[0] is None :
                extrapolated_point = [self.corners[1][0], self.corners[2][1]]
                self.corners[0] = np.array(extrapolated_point)
            if self.corners[3] is None :
                extrapolated_point = [self.corners[2][0], self.corners[1][1]]
                self.corners[3] = np.array(extrapolated_point)
    
    def get_parent_edge(self):
        '''Returns the entrance of the parent cell'''
        if self.parent is None :
            raise ValueError("The cell has no parent")
        else :
            if self.parent_direction == Direction.UP :
                return np.mean([self.corners[0],self.corners[1]],axis=0)
            elif self.parent_direction == Direction.RIGHT :
                return np.mean([self.corners[1],self.corners[3]],axis=0)
            elif self.parent_direction == Direction.DOWN :
                return np.mean([self.corners[2],self.corners[3]],axis=0)
            elif self.parent_direction == Direction.LEFT :
                return np.mean([self.corners[0],self.corners[2]],axis=0)
    
    def update_walls(self,walls):
        '''Updates the walls of the cell'''
        # Locical or on each wall
        self.walls = [self.walls[i] or walls[i] for i in range(len(self.walls))]

    
class Labyrinth_Solver:
    '''Ros node used to map and solve'''
    def __init__(self):
        rospy.init_node('lab_solver')
        self.is_init=False
        self.first_tile = None
        self.current_tile = None
        self.tile_size = 0.3

        self.robot_pos = None
        self.number_of_moves = 0

        self.state = State.MAPPING
        
        self.lidar_subscriber = rospy.Subscriber('/lidar/polylines',MarkerArray,self.lidar_callback)
        self.target_publisher = rospy.Publisher('/target',Pose,queue_size=10)
        self.pos_subscriber = rospy.Subscriber("/estimation", PoseWithCovarianceStamped, self.pos_callback)
        self.state_subscription = rospy.Subscriber("/state_check",Bool,self.state_callback)
        self.sight_publisher = rospy.Publisher('/sight',Float32,queue_size=10)
        self.finish_publisher = rospy.Publisher('/panneau_check',Bool,queue_size=10)

        self.is_init=True

        rospy.spin()
    
    def state_callback(self,data):
        '''Callback function for the state'''
        if data.data :
            # Next State
            if self.state == State.MOVING_TO_EDGE :
                self.state = State.WAITING
            elif self.state == State.MOVING_TO_CENTER :
                self.state = State.MAPPING
            elif self.state == State.MOVING_TO_FINISH :
                self.state = State.OFF
                print("Finished")
                output = Bool()
                output.data = True
                self.finish_publisher.publish(output)


    
    def lidar_callback(self,data):
        '''Callback function for the lidar'''
        # If the robot is moving, wait for it to stop
        if not self.is_init :
            return
        if self.state == State.MOVING_TO_EDGE or self.state == State.MOVING_TO_CENTER or self.state == State.OFF or self.robot_pos is None :
            return

        if self.state == State.FINISHED :
            self.state = MOVING_TO_FINISH
            target = Pose(Point(FINISH_POINT[0],FINISH_POINT[1],0),Quaternion())
            self.target_publisher.publish(target)
            return
        
        if self.state == State.WAITING :
            # Bring the robot to the center of the tile
            direction = self.current_tile.parent_direction
            distance = self.tile_size*0.4
            pos = np.array([self.robot_pos.pose.position.x,self.robot_pos.pose.position.y])
            if direction == Direction.UP :
                target_coord = pos + np.array([-distance,0])
            elif direction == Direction.RIGHT :
                target_coord = pos + np.array([0,distance])
            elif direction == Direction.DOWN :
                target_coord = pos + np.array([distance,0])
            elif direction == Direction.LEFT :
                target_coord = pos + np.array([0,-distance])
            target = Pose(Point(target_coord[0],target_coord[1],0),Quaternion())
            self.target_publisher.publish(target)
            self.state = State.MOVING_TO_CENTER
            print("Moving to center")
            return
        elif self.state == State.MAPPING :
            # Get the closest points in each quadrant
            quadrants = [[],[],[],[]] # [top_left,top_right,bottom_left,bottom_right]
            walls = [False,False,False,False]
            # Get orientation from quaternion
            robot_orientation = 2 * math.atan2(self.robot_pos.pose.orientation.z, self.robot_pos.pose.orientation.w)
            print(f"Robot position : {self.robot_pos.pose.position.x,self.robot_pos.pose.position.y}, angle : {robot_orientation}")
            for marker in data.markers :
                print("Marker found")
                points_in_quad = [False,False,False,False]
                for point in marker.points :
                    # Change base from robot coords to general coords
                    point = np.array([point.x,point.y])
                    # Rotate point
                    point = np.dot(np.array([[np.cos(robot_orientation),-np.sin(robot_orientation)],[np.sin(robot_orientation),np.cos(robot_orientation)]]),point)
                    point = point + np.array([self.robot_pos.pose.position.x,self.robot_pos.pose.position.y])
                    if point[0] > self.robot_pos.pose.position.x :
                        if point[1] > self.robot_pos.pose.position.y :
                            quadrants[0].append(point)
                            points_in_quad[0] = True
                            print(f"\tPoint in quad 0 : {point}")
                        else :
                            quadrants[1].append(point)
                            points_in_quad[1] = True
                            print(f"\tPoint in quad 1 : {point}")
                    else :
                        if point[1] > self.robot_pos.pose.position.y :
                            quadrants[2].append(point)
                            points_in_quad[2] = True
                            print(f"\tPoint in quad 2 : {point}")
                        else :
                            quadrants[3].append(point)
                            points_in_quad[3] = True
                            print(f"\tPoint in quad 3 : {point}")
                if points_in_quad[0] and points_in_quad[1] :
                    walls[Direction.UP.value-1] = True
                if points_in_quad[1] and points_in_quad[3] :
                    walls[Direction.RIGHT.value-1] = True
                if points_in_quad[2] and points_in_quad[3] :
                    walls[Direction.DOWN.value-1] = True
                if points_in_quad[0] and points_in_quad[2] :
                    walls[Direction.LEFT.value-1] = True
            
            
            # Sort points by distance to the robot
            for quadrant in quadrants :
                quadrant.sort(key=lambda point: np.linalg.norm(np.array([self.robot_pos.pose.position.x,self.robot_pos.pose.position.y])-np.array([point[0],point[1]])))
            
            # Get the closest point in each quadrant
            closest_points = []
            for quadrant in quadrants :
                if quadrant != [] :
                    closest_points.append(quadrant[0])
                else :
                    closest_points.append(None)
            
            print(f"Points found : {closest_points}")
            
            # Delete points that are too far
            for i in range(len(closest_points)):
                if closest_points[i] is not None and np.linalg.norm(np.array([self.robot_pos.pose.position.x,self.robot_pos.pose.position.y])-np.array([closest_points[i][0],closest_points[i][1]])) > self.tile_size*0.9 :
                    closest_points[i] = None

            print(f"Points after deletion : {closest_points}")

            if self.current_tile is None :
                self.current_tile = Tile()
                self.current_tile.update_walls(walls)
                self.current_tile.walls[Direction.UP.value-1]=False
                self.first_tile = self.current_tile
                # The mean of the distances between the points of each quadrant
                distances = []
                if (closest_points[0] is not None and closest_points[1] is not None) :
                    distances.append(np.linalg.norm(np.array([closest_points[0][0],closest_points[0][1]])-np.array([closest_points[1][0],closest_points[1][1]])))
                if (closest_points[2] is not None and closest_points[3] is not None) :
                    distances.append(np.linalg.norm(np.array([closest_points[2][0],closest_points[2][1]])-np.array([closest_points[3][0],closest_points[3][1]])))
                if (closest_points[0] is not None and closest_points[2] is not None) :
                    distances.append(np.linalg.norm(np.array([closest_points[0][0],closest_points[0][1]])-np.array([closest_points[2][0],closest_points[2][1]])))
                if (closest_points[1] is not None and closest_points[3] is not None) :
                    distances.append(np.linalg.norm(np.array([closest_points[1][0],closest_points[1][1]])-np.array([closest_points[3][0],closest_points[3][1]])))
                self.tile_size = np.mean(distances)
                self.sight_publisher.publish(Float32(self.tile_size))
                print(f"Taille d'une tuile : {self.tile_size}")
            else :
                self.current_tile.update_walls(walls)
                if self.current_tile == self.first_tile :
                    self.current_tile.walls[Direction.UP.value-1] = False

            # Update the corners of the current tile
            for i in range(len(closest_points)):
                if closest_points[i] is not None :
                    self.current_tile.set_corner(i,closest_points[i])
            print(f"Tile corners before extrapolation : {self.current_tile.get_corners()}")
            self.current_tile.extrapolate_corners()

            print(f"Tile corners after extrapolation : {self.current_tile.get_corners()}")

            # If all corners are defined, create the children
            cor = self.current_tile.get_corners()
            is_defined = type(cor[0]) != type(None) and type(cor[1]) != type(None) and type(cor[2]) != type(None) and type(cor[3]) != type(None)
            if is_defined and self.current_tile.children == [None,None,None,None] :
                for direction in Direction :
                    if not self.current_tile.walls[direction.value-1] and self.current_tile.parent_direction != direction:
                        self.current_tile.create_child(direction)
            
            # Check if the robot is out of the labyrinth
            if self.current_tile.walls == [None,None,None,None] and not is_defined and self.number_of_moves > 1 :
                self.state = State.FINISHED
                return
            
            # Decide to move :
            # If tile is mapped, go to child
            if is_defined :
                self.number_of_moves = 0
                children = [child for child in self.current_tile.children]
                shuffle(children)
                for child in children :
                    if child is not None :
                        if not child.visited :
                            child.visited = True
                            self.current_tile = child
                            self.state = State.MOVING_TO_EDGE
                            target_coord = self.current_tile.get_parent_edge()
                            target = Pose(Point(target_coord[0],target_coord[1],0),Quaternion())
                            self.target_publisher.publish(target)
                            print ("Moving to child edge")
                            return
                # If all children are visited, go to parent
                self.current_tile = self.current_tile.parent
                self.state = State.MOVING_TO_CENTER
                target_coord = self.current_tile.get_center()
                target = Pose(Point(target_coord[0],target_coord[1],0),Quaternion())
                self.target_publisher.publish(target)
                print ("Moving to parent center")
                return
            else :
                distance = self.tile_size*0.4
                # TO IMPROVE, DEPLACER EN FONCTION DU BORD DE LA TUILE
                if type(self.current_tile.get_corners()[0]) == type(None) and type(self.current_tile.get_corners()[1]) == type(None) :
                    direction = Direction.UP
                elif type(self.current_tile.get_corners()[1]) == type(None) and type(self.current_tile.get_corners()[3]) == type(None) :
                    direction = Direction.RIGHT
                elif type(self.current_tile.get_corners()[2]) == type(None) and type(self.current_tile.get_corners()[3]) == type(None) :
                    direction = Direction.DOWN
                elif type(self.current_tile.get_corners()[0]) == type(None) and type(self.current_tile.get_corners()[2]) == type(None) :
                    direction = Direction.LEFT
                
                pos = np.array([self.robot_pos.pose.position.x,self.robot_pos.pose.position.y])
                if direction == Direction.UP :
                    target_coord = pos + np.array([distance,0])
                elif direction == Direction.RIGHT :
                    target_coord = pos + np.array([0,-distance])
                elif direction == Direction.DOWN :
                    target_coord = pos + np.array([-distance,0])
                elif direction == Direction.LEFT :
                    target_coord = pos + np.array([0,distance])
                target = Pose(Point(target_coord[0],target_coord[1],0),Quaternion())
                self.target_publisher.publish(target)
                self.state = State.MOVING_TO_CENTER
                self.number_of_moves += 1
                print("Could not find tile edge : continuing within the tile")
                print("Current mapping state :")
                print(f"\tWalls : {self.current_tile.walls} # [up,right,down,left]")
                print(f"\tCorners : {self.current_tile.get_corners()} # [top_left,top_right,bottom_left,bottom_right]")
                return

        


    def pos_callback(self,data):
        '''Callback function for the robot position'''
        self.robot_pos = data.pose
if __name__ == "__main__":
    node=Labyrinth_Solver()
