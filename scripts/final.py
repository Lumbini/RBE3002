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
    #print "Frontiers: ", frontiers
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
        #print node
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
                    #print neighbor.x, neighbor.y, smallerWidth, ypos2

                    point2 = OurPoint(xpos2, ypos2)
                    #print point2
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
    

def publishCell(node):
    global pub
    global offsetY
    global offsetX

    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution*3
    cells.cell_height = resolution*3

    point=Point()
    point.x=(node.x*resolution*3)+offsetX + (0.5* resolution*3) # added secondary offset 
    point.y=(node.y*resolution*3)+offsetY - (-0.5 * resolution*3) # added secondary offset ... Magic ?
    point.z=0
    cells.cells.append(point)
        
    #print cells.cells
    pub.publish(cells)   

## should make a list of places bordering unknown space
## this can become much more interesting if we want, using a labeling algorithm to identify spaces and then we can find the centroid of the space
def findFrontiers():
    global frontiers

    if len(frontiers) == 0:
        # raise Exception("frontiers list is empty, you are done")
        print "frontiers is empty 2"
    else:
        toreturn = frontiers[0]
        publishCell(toreturn)
        frontiers.pop(0)
        return toreturn
       
## publishes a message to drive to this node
def driveTo(node):
    global pub_drive
    global theta
    global moving
    msg = PoseStamped()

    ## set up header and position
    msg.header.frame_id = 'map'
    msg.pose.position.x = node.x
    msg.pose.position.y = node.y
    #msg.pose.position.z = 0

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
    pub_drive_startup.publish(msg)

def startup():
    global pose
    print "vroom vroom rotating"
    ## TODO make this more interesting maybe?
    spinWheels(-0.04, 0.04, 3)
    rospy.sleep(2)

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

## attempts to recover the robot after it says it cannot go to a frontier
def attemptRecover():
    global frontiers
    global moveError
    global frontier
    ## do our startup maneuver again just in case
    startup()
    ## try to find a fontier and go there
    try:
        frontier = findFrontiers()
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
        print "Robot has set its destination"
    elif result == 2:
        print "result 2, who cares?"
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
    # global frontiers
    global pub_drive
    global pub_drive_startup
    global frontier

    rospy.init_node('final_turtlebot')

    
    pose = Pose()

    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    base_result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, moveBaseResult, queue_size=1) 
    pub_drive = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10) ## TODO check this type
    pub_drive_startup = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, None, queue_size=10)

    odom_list = tf.TransformListener() #listener for robot location
    rospy.Timer(rospy.Duration(.01), tCallback) # timer callback for robot location


    rospy.sleep(3)

    print "after pubs and subs"

## -----------------------------------------------------------------------------------------------------------
## new main for the final

    doneExploring = False
    moving = False
    moveError = False

    ## make robot do startup spin maneuver here
    spinWheels(-0.2, 0.2, 4)
    print "after startup"

    rospy.sleep(4) ## waiting so we get the new mapData after the spin
    startingPoint = OurPoint(pose.position.x, pose.position.y)
    
    # ## TODO might also want to keep track of the frontiers that have failed
    try: 

        while (not doneExploring) and (not rospy.is_shutdown()):
            frontier = findFrontiers(); ## find frontiers
            print "trying to drive to ", frontier
            driveTo(frontier) 
                                  
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
