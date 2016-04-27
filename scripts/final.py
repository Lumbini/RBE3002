#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
from Node import Node
import Driving
import tf
import numpy
import math 
import rospy, tf, numpy, math
import AStar
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

    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y

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

    smallerNodeDict = {}


    for i in range(0, len(mapData) / 9):
        ypos = int(math.floor(i / smallerWidth))
        xpos = int(i - (ypos * smallerWidth)) - (smallerWidth / 2)
        ypos = ypos  - (height / 6)
        point = OurPoint(xpos, ypos)
        node = Node(xpos, ypos, 0, smallerWidth)
        print node
        smallerNodeDict[point] = node

    k = -1

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

    for key in smallerNodeDictCopy:
        node = smallerNodeDict[key]
        if node.data == 100:
            for neighbor in node.getAllNeighbors(smallerNodeDict):
                point = OurPoint(neighbor.x, neighbor.y)
                neighborNode = smallerNodeDictCopy[point]
                neighborNode.data = 100



def readGoal(goal):
    global goalX	
    global goalY
    print 'hi'
 
    goalX = int(goal.pose.position.x / resolution)
    goalY = int(goal.pose.position.y / resolution)
    smallerX = int(math.ceil(goalX / 3))
    smallerY = int(math.ceil(goalY / 3))
    indexGoal = int(math.floor(goalX + (goalY * width)))
    smallerIndex = smallerX + smallerY * smallerWidth

    goalPoint = OurPoint(smallerX, smallerY)

    #Convert the goal to a Node object
    goalNode = Node(smallerX, smallerY, smallerNodeDictCopy[goalPoint], smallerWidth)
    thisPath = AStar.AStar(startPosNode, goalNode, smallerNodeDictCopy)
    print thisPath
    waypoints = []
    #waypoints.append(startPosNode)
    waypoints.extend(reversed(AStar.getWaypoints(thisPath)))
    waypoints.append(goalNode)

    print "goal", goal.pose
    publishPath(thisPath, waypoints)
    for i in range(0, len(waypoints)):
        newPoseX = waypoints[i].x * resolution
        newPoseY = waypoints[i].y * resolution
        newPose = Pose()
        newPose.position.x = newPoseX
        newPose.position.y = newPoseY

def readStart(startPos):
    global startPosX
    global startPosY
    global startPosNode

    startPosX = int(startPos.pose.pose.position.x / resolution)
    startPosY = int(startPos.pose.pose.position.y / resolution)
    indexStart = int(math.floor(startPosX + (startPosY * width)))
    smallerX = int(math.ceil(startPosX / 3))
    smallerY = int(math.ceil(startPosY / 3))
    smallerIndex = smallerX + smallerY * smallerWidth

    startPoint = OurPoint(smallerX, smallerY)
    #Cconvert start node to a Node Object. 
    startPosNode = Node(smallerX, smallerY, smallerNodeDictCopy[startPoint].data, smallerWidth)
    print "start ", startPos.pose.pose


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
       
def driveTo(node):
    global pub_drive
    pose = PoseStamped()

    pose.header.frame_id = 'map'
    pose.pose.position.x = node.x
    pose.pose.position.y = node.y
    pose.pose.position.z = 0

    if theta is None:
        quat = (0, 0, 0, 1)
    else:
        try:
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, float(theta))
        except Exception, e:
            raise e

    qx, qy, qz, qw = quat
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    pub_drive.publish(pose)
    moving = True

def startup():
    ## this can be a rather complicated or simple maneuver
    ## we can choose whatever we want
    ## rotating is a complete circle, moving to a predefined set of points relatively close to the robot, etc.
    pass

def attemptRecover():
    global frontiers
    global moveError
    startup()

    try:
        frontiers = findFrontiers()
        moveError = False
    except Exception, e:
        pass

    return


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
    global pubpath
    global pubway
    global expand_pub
    global smallerNodeDict
    global smallerNodeDictCopy
    global doneExploring
    global moving
    global moveError
    global pose
    global odom_list
    global frontiers
  
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1)
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('goal_lab3', PoseStamped, readGoal, queue_size=1) #change topic for best results
    start_sub = rospy.Subscriber('initpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results
    expand_pub = rospy.Publisher('/expand', GridCells, queue_size=1)
    base_result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, moveBaseResult, queue_size=1)
    pub_drive = rospy.Publisher('/move_base_simple/goal')

    rospy.Timer(rospy.Duration(.01), tCallback) # timer callback for robot location
    
    odom_list = tf.TransformListener() #listener for robot location

    rospy.sleep(5)

## -----------------------------------------------------------------------------------------------------------
## new main for the final

    doneExploring = False
    moving = False
    moveError = False

    ## make robot do startup spin maneuver here

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
