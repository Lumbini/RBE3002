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
    #nodeGrid = []

    nodeDict = {}

    ## parses the map into a dictionary
    for i in range(0, len(mapData)):
        ypos = int(math.floor(i / width)) - (height / 2) ## shifts y downwards on real robot
        xpos = i - (ypos * width) - (width / 2) ## shifts x left on real robot
        point = OurPoint(xpos, ypos)
        node = Node(xpos, ypos, mapData[i], width)
        nodeDict[point] = node

    smallerWidth = int(math.floor(width / 3))

    smallerNodeDict = {}

    for i in range(0, len(mapData) / 9):
        ypos = int(math.floor(i / smallerWidth)) - (int(math.floor(i / smallerWidth)) / 2)
        xpos = int(i - ((math.floor(i/smallerWidth) * smallerWidth))) - (smallerWidth / 2)
        point = OurPoint(xpos, ypos)
        node = Node(xpos, ypos, 0, smallerWidth)
        smallerNodeDict[point] = node

    k = -1

    for i in range(0, len(nodeDict) - 4):
        ypos = int(math.floor(i / width)) - (height / 2) ## shifts y downwards on real robot
        xpos = i - (ypos * width) - (width / 2) ## shifts x left on real robot
        point = OurPoint(xpos, ypos)
        node = nodeDict[point]

        ##get ALL neighbors here
        neighbors = [] ## TODO fix this
        col = i % 9
        row = int(math.floor(i / width))
        if ((row - 1) % 3 == 0):
            k = k + 1
            for neighbor in neighbors:
                if(neighbor.data == 100):
                    ypos = int(math.floor(k / width)) - (height / 2) ## shifts y downwards on real robot
                    xpos = k - (ypos * width) - (width / 2) ## shifts x left on real robot
                    point = OurPoint(xpos, ypos)
                    lowResNode = smallerNodeDict[point]
                    lowResNode.data = 100
	
    smallerNodeDictCopy = copy.deepcopy(smallerNodeDict)

    for key in smallerNodeDictCopy:
        node = smallerNodeDictCopy[key]
        if node.data == 100:
            for neighbor in node.getNeighbors(smallerNodeDictCopy): ## TODO fix getNeighbors
                point = OurPoint(neighbor.x, neighbor.y)
                neighborNode = smallerNodeDictCopy[point]
                neighborNode.data = 100



def readGoal(goal):
    global goalX	
    global goalY
    print 'hi'
 
    goalX = int(goal.pose.position.x / resolution)
    goalY = int(goal.pose.position.y / resolution)
    smallerX = int(math.floor(goalX / 3))
    smallerY = int(math.floor(goalY / 3))
    indexGoal = int(math.floor(goalX + (goalY * width)))
    smallerIndex = smallerX + smallerY * smallerWidth

    #Convert the goal to a Node object
    goalNode = Node(smallerX, smallerY, nodeGridCopy[smallerIndex], smallerWidth)
    thisPath = AStar.AStar(startPosNode, goalNode, nodeGridCopy)
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
    smallerX = int(math.floor(startPosX / 3))
    smallerY = int(math.floor(startPosY / 3))
    smallerIndex = smallerX + smallerY * smallerWidth
    #Cconvert start node to a Node Object. 
    startPosNode = Node(smallerX, smallerY, nodeGridCopy[smallerIndex].data, smallerWidth)
    print "start ", startPos.pose.pose

def publishPath(path, waypoints):
    global pubpath
    global pubway

    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution * 3
    cells.cell_height = resolution * 3

    cells2 = GridCells()
    cells2.header.frame_id = 'map'
    cells2.cell_width = resolution * 3
    cells2.cell_height = resolution * 3

    for node in path:
        point = Point()
        point.x = (node.x * resolution*3) + (0.5 * resolution*3)#offsetX + (1.5 * resolution)
        point.y=(node.y * resolution*3) + (.5 * resolution*3) #offsetY - (.5 * resolution)
        point.z = 0
        cells.cells.append(point)

    for node in waypoints:
        point = Point()
        point.x = (node.x * resolution*3) + (0.5 * resolution*3)#offsetX + (1.5 * resolution)
        point.y=(node.y * resolution*3) + (.5 * resolution*3) #offsetY - (.5 * resolution)
        point.z = 0
        cells2.cells.append(point)

    pubpath.publish(cells)
    pubway.publish(cells2)



#publishes map to rviz using gridcells type
def publishCells(grid, nodes):
    global pub
    global smallerNodeGrid
    global offsetY
    global offsetX

    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution*3
    cells.cell_height = resolution*3

    cells2 = GridCells()
    cells2.header.frame_id = 'map'
    cells2.cell_width = resolution*3
    cells2.cell_height = resolution*3

    for key in grid:
        thisNode = grid[key]
        thatNode = nodes[key]

        if thisNode.data == 100:
            point=Point()
            point.x=(key.x*resolution*3)+offsetX + (0.5* resolution*3) # added secondary offset 
            point.y=(key.y*resolution*3)+offsetY - (-0.5 * resolution*3) # added secondary offset ... Magic ?
            point.z=0
            cells.cells.append(point)
        if(thatNode.data == 100):
            point=Point()
            point.x=(key.x*resolution*3)+offsetX + (.5 * resolution*3) # added secondary offset 
            point.y=(key.y*resolution*3)+offsetY - (-0.5 * resolution*3) # added secondary offset ... Magic ?
            point.z=0
            cells2.cells.append(point)

    #print cells.cells
    pub.publish(cells)
    expand_pub.publish(cells2)

                   

#Main handler of the project
def run():
    global pub
    global pubpath
    global pubway
    global expand_pub
  
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('goal_lab3', PoseStamped, readGoal, queue_size=1) #change topic for best results
    start_sub = rospy.Subscriber('initpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results
    expand_pub = rospy.Publisher('/expand', GridCells, queue_size=1)

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(4)

    while (1 and not rospy.is_shutdown()):
        publishCells(smallerNodeDict, smallerNodeDictCopy) #publishing map data every 4 seconds
        #rospy.sleep(6) 
        print("Complete")


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
