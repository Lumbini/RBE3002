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

"""
Callback functions
--------------------------------------------------------------------------------------------
"""

def mapCallBack(data):

    """
    Map callback
    Triggers everytime something is published to the /map topic
    Updates our local map data in our dictionary
    Updates the list of frontiers
    """

    print("Updating map")

    global resolution
    global offsetX
    global offsetY
    global nodeDict
    global frontiers
    global extendedFrontiers

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
    extendedFrontiers = []

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
        if node.data == -1:
            neighbors = node.getAllNeighbors(nodeDict)
            for neighbor in neighbors:
                if neighbor.data != -1 and neighbor.data != 100 and (neighbor not in frontiers):
                    frontiers.append(neighbor)
                    break

    ## Finds extended driving points that are all 0 space
    for node in frontiers:
        if node.data != 0:
            robotPoint = OurPoint(pose.position.x, pose.position.y)
            frontiers.remove(node)
            if node.x > robotPoint.x:
                if node.y > robotPoint.y:
                    for i in range(robotPoint.x, node.x + 1):
                        if extendedFrontiersHelper(i, j, nodeDict):
                            extendedFrontiers.append(nodeDict[OurPoint(i, j)])
                            break
                    for j in range(robotPoint.y, node.y + 1):
                        if extendedFrontiersHelper(i, j, nodeDict):
                            extendedFrontiers.append(nodeDict[OurPoint(i, j)])
                            break
                else:
                    for i in range(robotPoint.x, node.x + 1):
                        if extendedFrontiersHelper(i, j, nodeDict):
                            extendedFrontiers.append(nodeDict[OurPoint(i, j)])
                            break
                    for j in range(node.y, robotPoint.y + 1):
                        if extendedFrontiersHelper(i, j, nodeDict):
                            extendedFrontiers.append(nodeDict[OurPoint(i, j)])
                            break
            else:
                if node.y > robotPoint.y:
                    for i in range(node.x, robotPoint.x + 1):
                        if extendedFrontiersHelper(i, j, nodeDict):
                            extendedFrontiers.append(nodeDict[OurPoint(i, j)])
                            break
                    for j in range(robotPoint.y, node.y + 1):
                        if extendedFrontiersHelper(i, j, nodeDict):
                            extendedFrontiers.append(nodeDict[OurPoint(i, j)])
                            break

                else:
                    for i in range(node.x, robotPoint.x + 1):
                        if extendedFrontiersHelper(i, j, nodeDict):
                            extendedFrontiers.append(nodeDict[OurPoint(i, j)])
                            break
                    for j in range(node.y, robotPoint.y + 1):
                        if extendedFrontiersHelper(i, j, nodeDict):
                            extendedFrontiers.append(nodeDict[OurPoint(i, j)])
                            break



def extendedFrontiersHelper(i, j, grid):
    thisNode = grid[OurPoint(i, j)]
    if thisNode.data == 0:
        return True
    else:
        return False


def timerCallback(event):
    
    """
    Timer Callback
    Executes every 1ms to update the position and orientation of the robot
    Uses odom and tf to do so
    """

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
   
"""
Frontier functions
--------------------------------------------------------------------------------------------
"""

def findFrontiers(frontierList):

    """
    Find Frontiers
    Returns one frontier for the robot to try to drive to
    Gets this frontier from the list of frontiers
    """

    # global failedList

    if len(frontierList) > 0:
        toreturn = frontierList[0]
        frontierList.pop(0)
        while toreturn in failedList and len(frontierList) > 0:
            toreturn = frontierList[0]
            frontierList.pop(0)
            failedList.append(toreturn)
            
        return toreturn
    else:
        print "Frontiers list is empty. You must be done searching!"

"""
Driving functions
--------------------------------------------------------------------------------------------
"""

def spinWheels(u1, u2, time, publisher):

    """
    Spin Wheels
    Accepts two wheel velocities and a time interval
    Publishes a movement message that executes that motion for that time
    """

    print "Spinning in spinWheels"

    r = 3.5 / 100.0
    b = 23.0 / 100.0
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
        publisher.publish(move_msg) #publishes the move_msg
    publisher.publish(stop_msg)

"""
RViz functions
--------------------------------------------------------------------------------------------
"""

def publishCells(nodes, publisher):

    """
    Publish Cells
    Publishes a list of nodes as GridCells to RViz
    Uses the given publisher
    """

    global offsetY
    global offsetX

    print "publishing frontiers"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution

    for node in nodes:
        point=Point()
        point.x = (node.x * resolution) + offsetX + (0.72 / resolution)
        point.y = (node.y * resolution) + offsetY + (0.643 / resolution)
        point.z = 0
        cells.cells.append(point)

    publisher.publish(cells)

"""
Main function
--------------------------------------------------------------------------------------------
"""

#Main handler of the project
def run():

    """
    Run
    Run is basically our main
    """

    global pose
    global odom_list
    global frontier
    global frontiers
    global failedList
    global extendedFrontiers

    failedList = []
    extendedFrontiers = []
    frontiers = []

    nav_node = NavNode()
    pose = Pose()

    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub_frontiers = rospy.Publisher("/frontiers", GridCells, queue_size=1)  
    pub_drive_startup = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, None, queue_size=10)

    print "Set up publisher and subscribers"

    odom_list = tf.TransformListener()
    rospy.Timer(rospy.Duration(.01), timerCallback)

    rospy.sleep(3)

    spinWheels(-0.15, 0.15, 5, pub_drive_startup)

    rospy.sleep(3)

    print "Entering startup\n"
    print "Driving to startup position 1"
    nav_node.goto_point(1.0, 0.0)
    print "Driving to startup position 2"
    nav_node.goto_point(1.0, -1.0)
    print "Driving to startup position 3"
    nav_node.goto_point(0.0, -1.0)
    print "Driving to startup position 4"
    nav_node.goto_point(-1.0, -1.0)
    print "Driving to startup position 5"
    nav_node.goto_point(-1.0, 0.0)
    print "Driving to startup position 6"
    nav_node.goto_point(-1.0, 1.0)
    print "Driving to startup position 7"
    nav_node.goto_point(0.0, 1.0)
    print "Driving to startup position 8"
    nav_node.goto_point(1.0, 1.0)

    print "\nAfter startup maneuver\n"

    while (len(extendedFrontiers) > 0 or len(frontiers) > 0) and not (rospy.is_shutdown()):
        while len(extendedFrontiers) > 0 and not (rospy.is_shutdown()):
            publishCells(frontiers, pub_frontiers)
            publishCells(extendedFrontiers, pub_frontiers)
            frontier = findFrontiers(extendedFrontiers)
            nav_node.goto_point(float(frontier.x * resolution), float(frontier.y * resolution))
            print "Remaining Extended Frontiers: ", len(extendedFrontiers)

        while len(frontiers) > 0 and not (rospy.is_shutdown()):
            publishCells(frontiers, pub_frontiers)
            publishCells(extendedFrontiers, pub_frontiers)
            frontier = findFrontiers(frontiers)
            nav_node.goto_point(float(frontier.x * resolution), float(frontier.y * resolution))
            print "Remaining Frontiers: ", len(frontiers)

        failedList = []

    print "Done exploring"
    print "Frontiers: ", frontiers
    print "Extended frontiers", extendedFrontiers

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
