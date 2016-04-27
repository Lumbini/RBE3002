#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
from Node import Node
import tf
import numpy
import math 
import rospy, tf, numpy, math
import copy
from OurPoint import OurPoint


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

    ## parses the map into a dictionary
    for i in range(0, len(mapData)):
        ypos = int(math.floor(i / width)) ## shifts y downwards on real robot
        xpos = i - (ypos * width) - (width / 2) ## shifts x left on real robot
        ypos = ypos - (height / 2)
        point = OurPoint(xpos, ypos)
        node = Node(xpos, ypos, mapData[i], width)
        nodeDict[point] = node

    smallerWidth = int(math.floor(width / 3))

    ## lets make the map lower resolution for speed sake
    smallerNodeDict = {}

    ## make a smaller dictionary with nodes all at 0
    for i in range(0, len(mapData) / 9):
        ypos = int(math.floor(i / smallerWidth))
        xpos = int(i - (ypos * smallerWidth)) - (smallerWidth / 2)
        ypos = ypos  - (height / 6)
        point = OurPoint(xpos, ypos)
        node = Node(xpos, ypos, 0, smallerWidth)
        print node
        smallerNodeDict[point] = node

    k = -1

    ## update this smaller dictionary with the actual values of the cells
    for key in nodeDict:
        
        node = nodeDict[key]
        
        #print node
        ##get ALL neighbors here
        #print neighbors
        if ((node.x - 1) % 3 == 0 and (node.y - 1) % 3 == 0):
            neighbors = node.getAllNeighbors(nodeDict)
            for neighbor in neighbors:
                if(neighbor.data == 100):

                    xpos2 = math.ceil(node.x / 3)
                    ypos2 = math.ceil(node.y / 3)
                    print neighbor.x, neighbor.y, smallerWidth, ypos2

                    point2 = OurPoint(xpos2, ypos2)
                    print point2
                    if point2 in smallerNodeDict:
                        print "exists", smallerNodeDict.get(point2)
                    else:
                        print "doesnt exist"
                    lowResNode = smallerNodeDict[point2]
                    lowResNode.data = 100
	
    smallerNodeDictCopy = copy.deepcopy(smallerNodeDict)

    ## obstacle expansion
    for key in smallerNodeDictCopy:
        node = smallerNodeDict[key]
        if node.data == 100:
            for neighbor in node.getAllNeighbors(smallerNodeDict):
                point = OurPoint(neighbor.x, neighbor.y)
                neighborNode = smallerNodeDictCopy[point]
                neighborNode.data = 100


## should make a list of places bordering unknown space
## this can become much more interesting if we want, using a labeling algorithm to identify spaces and then we can find the centroid of the space
def findFrontiers(grid):
    
    frontiers = []

    for key in grid:
        node = grid[key]
        neighbors = node.getAllNeighbors(grid)

        if node.data == -1:
            for neighbor in neighbors:
                if neighbor.data != -1 and neighbor.data != 100:
                    frontiers.append(node)
                    break

    if frontiers is None:
        raise Exception("frontiers list is empty, you are done")
    else:
        return frontiers
       
## publishes a message to drive to this node
def driveTo(node):
    global pub_drive
    msg = PoseStamped()

    ## set up header and position
    msg.header.frame_id = 'map'
    msg.pose.position.x = node.x
    msg.pose.position.y = node.y
    msg.pose.position.z = 0

    ## quaternion wizardry
    if theta is None:
        quat = (0, 0, 0, 1)
    else:
        try:
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, float(theta))
        except Exception, e:
            raise e

    ## set up the orientation based on the quaternion calculations
    qx, qy, qz, qw = quat
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw

    ## publish the message and set moving state
    pub_drive.publish(msg)
    moving = True

def publishTwist(lin_Vel, ang_Vel):
    """Send a movement (twist) message."""
    global pub_drive_startup
    msg = Twist()
    msg.linear.x = lin_Vel
    msg.angular.z = ang_Vel
    pub.publish(msg)

def startup():
    global pose
    ## TODO make this more interesting maybe?
    publishTwist(0.0, 0.2)
    rospy.sleep(2)
    

## attempts to recover the robot after it says it cannot go to a frontier
def attemptRecover():
    global frontiers
    global moveError
    ## do our startup maneuver again just in case
    startup()
    ## try to find a fontier and go there
    try:
        frontiers = findFrontiers()
        moveError = False
    except Exception, e: ## if we can't find a frontier
        raise e ## this exception will cause the robot to stop exploring, we must be done if we can't recover

    return

## keeps track of the robot state
def moveBaseResult(msg):
    global moving
    global moveError
    result = msg.status.status
    ## result tells us the state of the robot
    ## 0 doesn't mean anything important to us
    ## 1 means the robot is moving to its destination
    ## 2 doesn't mean anything important to us
    ## 3 means the robot has reached its destination
    ## 4 means the robot cannot reach the destination 
    if result == 1:
        moving = True 
        print "Robot is currently moving to its destination"
    elif result == 3:
        moving = False
        print "Robot has reached its destination"
    elif result == 4:
        moving = False
        moveError = True
        print "There is an error moving to the destination"
    else:
        print "There was some unexpected result that we don't care about right now"

## timer callback function
## updates position, orientation and time of the system
def tCallback(event):
    
    global pose
    global theta

    now = rospy.Time.now()

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
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
    global frontiers
    global pub_drive
    global pub_drive_startup
  
    rospy.init_node('final_turtlebot')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    base_result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, moveBaseResult, queue_size=1)
    pub_drive = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10) ## TODO check this type
    pub_drive_startup = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, None, queue_size=10)

    rospy.Timer(rospy.Duration(.01), tCallback) # timer callback for robot location
    
    odom_list = tf.TransformListener() #listener for robot location

    rospy.sleep(5)

## -----------------------------------------------------------------------------------------------------------
## new main for the final

    doneExploring = False
    moving = False
    moveError = False

    ## make robot do startup spin maneuver here
    startup()

    rospy.sleep(5) ## waiting so we get the new mapData after the spin
    ## TODO might also want to keep track of the frontiers that have failed
    try: 

        while (not doneExploring) and (not rospy.is_shutdown()):
            frontiers = findFrontiers(); ## find frontiers
            driveTo(frontiers[0]) ## TODO might need a better way of storing frontiers/iterating through them
                                  ## maybe keeping failed frontiers in a list and avoiding them until they're the only ones left

            ## wait until the robot is done moving or has a moveError
            while moving and (not moveError) and (not rospy.is_shutdown()): 
                rospy.sleep(0.1)

            ## if there is a move error, we should try to recover and find the next open frontier
            if moveError:
                try:
                    attemptRecover()
                ## if we can't recover, that must mean we're done searching
                except Exception, e:
                    doneExploring = True 

    except rospy.ROSInterruptException:
        pass



if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
