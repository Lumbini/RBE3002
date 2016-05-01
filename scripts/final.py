#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
from Node import Node
import tf
import numpy
import math 
import rospy, tf, numpy, math
import copy
from OurPoint import OurPoint
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from action_nav import NavNode

wheel_rad = 3.5 / 100.0 #cm
wheel_base = 23.0 / 100.0 #cm

# reads in global map
def mapCallBack(data):
    print("map callback")
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    global smallerNodeDictCopy
    global nodeDict
    global smallerNodeDict
    global smallerWidth
    global frontiers

    ## store data about the map
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y

    ## dictionary to store points and nodes
    nodeDict = {}
    frontiers = []

    ## parses the map into a dictionary
    for i in range(0, len(mapData)):
        ypos = int(math.floor(i / width)) ## shifts y downwards on real robot
        xpos = i - (ypos * width) - (width / 2) ## shifts x left on real robot
        ypos = ypos - (height / 2)
        point = OurPoint(xpos, ypos)
        node = Node(xpos, ypos, mapData[i], width)
        nodeDict[point] = node

    ## Find all possible frontiers in the given map. 
    for key in nodeDict:
        node = nodeDict[key]
        neighbors = node.getAllNeighbors(nodeDict)
        if node.data == -1:
            for neighbor in neighbors:
                if neighbor.data != -1 and neighbor.data != 100:
                    frontiers.append(neighbor)
                    break
   

## should make a list of places bordering unknown space
## this can become much more interesting if we want, using a labeling algorithm to identify spaces and then we can find the centroid of the space
def findFrontiers():
    global frontiers
    global failedList

    if len(frontiers) == 0:
        # raise Exception("frontiers list is empty, you are done")
        print "frontiers is empty 2"
    else:

        toreturn = frontiers[0]
        frontiers.pop(0)
        ##robot tried to get to places at least twice
        if toreturn not in failedList:
            frontiers.append(toreturn)
            failedList.append(toreturn)
        return toreturn

       
def publishTwist(lin_Vel, ang_Vel):
    """Send a movement (twist) message."""
    global pub_drive_startup
    msg = Twist()
    msg.linear.x = lin_Vel
    msg.angular.z = ang_Vel
    pub_drive_startup.publish(msg)


def spinWheels(u1, u2, time):
    """This function accepts two wheel velocities and a time interval."""
    print "Spinning"
    global pub

    r = wheel_rad
    b = wheel_base
    #compute wheel speeds
    w = (u1 - u2)/b #Determines the angular velocity of base on the wheels.
    if w == 0:
        u = u1
    else: 
        u = w * ((b/2) * ((u1 + u2)/(u1-u2)))
    start = rospy.Time().now().secs
    #create movement and stop messages
    move_msg = Twist() #creates a move_msg object inheriting type from the Twist() class
    move_msg.linear.x = u #sets linear velocity
    move_msg.angular.z = w #sets amgular velocity (Populates messages with data.)

    stop_msg = Twist()
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0
    #publish move message for desired time
    while(rospy.Time().now().secs - start < time and not rospy.is_shutdown()): # waits for said time and checks for ctrl C
        pub_drive_startup.publish(move_msg) #publishes the move_msg
    pub_drive_startup.publish(stop_msg)

def publishCells(nodes):
    global pub_frontiers
    global offsetY
    global offsetX

    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution

    for node in nodes:
        point=Point()
        point.x = (node.x * resolution) - offsetX + (0.30 / resolution) # added secondary offset 
        point.y = (node.y * resolution) + offsetY + (0.44 / resolution) # added secondary offset ... Magic ?
        point.z = 0
        cells.cells.append(point)

    pub_frontiers.publish(cells)

## timer callback function
## updates position, orientation and time of the system
def tCallback(event):
    
    global pose
    global theta

    now = rospy.Time.now()

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    pose.position.x = position[0]
    pose.position.y = position[1]

    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    #convert yaw to degrees
    pose.orientation.z = yaw
    theta = math.degrees(yaw)

#Main handler of the project
def run():
    global pub
    global smallerNodeDict
    global smallerNodeDictCopy
    global doneExploring
    global moving
    global moveError
    global pose
    global odom_list
    global pub_drive_startup
    global frontier
    global frontiers
    global failedList
    global pub_frontiers

    nav_node = NavNode()

    failedList = []

    #rospy.init_node('final_turtlebot')

    pose = Pose()

    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub_frontiers = rospy.Publisher("/frontiers", GridCells, queue_size=1)  
    pub_drive_startup = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, None, queue_size=10)


    odom_list = tf.TransformListener() #listener for robot location
    rospy.Timer(rospy.Duration(.01), tCallback) # timer callback for robot location


    rospy.sleep(3)

    print "after pubs and subs"

    doneExploring = False
    moving = False
    moveError = False

    ## make robot do startup spin maneuver here
    spinWheels(-0.2, 0.2, 5)
    print "after startup"

    rospy.sleep(4) ## waiting so we get the new mapData after the spin

    # nav_node.goto_point(1.0, 0.0)
    # nav_node.goto_point(0.0, 1.0)
    # nav_node.goto_point(1.0, 1.0)
    # nav_node.goto_point(-1.0, -1.0)
    # nav_node.goto_point(0.0, 0.0)
    while len(frontiers) > 0 and not (rospy.is_shutdown()):
        publishCells(frontiers)
        frontier = findFrontiers()
        nav_node.goto_point(float(frontier.x * resolution), float(frontier.y * resolution))

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
