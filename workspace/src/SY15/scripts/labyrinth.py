import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker

SIZE = 20 # Size of the labyrinth

# Enum of directions
class Direction:
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3

class Tile :
    '''This class represents one cell of the labyrinth, the labyrinth is defined recursively as a Tile tree'''
    def __init__(self,parent=None):
        self.corners=[None,None,None,None] # [top_left,top_right,bottom_left,bottom_right]
        self.parent = parent
        self.parent_direction = None
        self.children = [None,None,None,None] # [up,right,down,left]
    
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

    
class Labyrinth_Solver:
    '''Ros node used to map and solve'''
    def __init__(self):
        rospy.init_node('lab_solver')
        self.lidar_subscriber = rospy.Subscriber('/poly_cartesien',MarkerArray,self.lidar_callback)
        self.target_publisher = rospy.Publisher('/target',PoseStamped,queue_size=10)
        self.pos_subscriber = rospy.Subscriber('/...',PoseStamped,self.pos_callback)

        self.first_tile = None
        self.current_tile = None

        self.robot_pos = None

        self.moving = False

        rospy.spin()
    
    def lidar_callback(self,data):
        '''Callback function for the lidar'''

        # If the robot is moving, wait for it to stop
        if self.moving :
            return
        
        # Initialization of the first tile
        if self.first_tile is None :
            self.first_tile = Tile()
            # Detect entrance : two alligned segments with a gap between them
            if len(data.markers) != 2 :
                raise ValueError("The labyrinth entrance is not detected")
            else :
                # Take the closest point to the robot of each segment
                first_segment = [data.markers[0].points[0],data.markers[0].points[-1]]
                second_segment = [data.markers[1].points[0],data.markers[1].points[-1]]
                first_segment_distances = [np.linalg.norm(np.array(self.robot_pos.pose.position)-np.array(point)) for point in first_segment]
                second_segment_distances = [np.linalg.norm(np.array(self.robot_pos.pose.position)-np.array(point)) for point in second_segment]

                entrance_points = []
                if first_segment_distances[0] < first_segment_distances[1] :
                    entrance_points.append(first_segment[0])
                else :
                    entrance_points.append(first_segment[1])

                if second_segment_distances[0] < second_segment_distances[1] :
                    entrance_points.append(second_segment[0])
                else :
                    entrance_points.append(second_segment[1])
                
                # Sort entrance points by x coordinate
                entrance_points.sort(key=lambda point: point.x)

                # Set the corners of the first tile
                self.first_tile.set_corner(2,[entrance_points[0].x,entrance_points[0].y])
                self.first_tile.set_corner(3,[entrance_points[1].x,entrance_points[1].y])
                self.first_tile.parent_direction = Direction.DOWN

                # Go to entrance
                self.target_publisher.publish(np.mean(entrance_points,axis=0))
                self.current_tile = self.first_tile
                self.moving = True
                return
        
        else :
            # Map the tile the robot just entered
            
        


    def pos_callback(self,data):
        '''Callback function for the robot position'''
        self.robot_pos = data