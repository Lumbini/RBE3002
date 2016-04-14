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
    global nodeGridCopy
    global nodeGrid
    global smallerNodeGrid
    global smallerWidth

    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    nodeGrid = []

    for i in range(0, len(mapData)):
        prob = mapData[i]
        ypos = math.floor(i / width)
        xpos = i - (ypos * width)
        node = Node(xpos, ypos, prob, width)
        nodeGrid.append(node)

    smallerWidth = int(math.floor(width/3))

    smallerNodeGrid = []
    for i in range(0, int(math.floor(len(mapData)/9))):
        y = int(math.floor(i / smallerWidth))
        x = int(i - ((math.floor(i/smallerWidth) * smallerWidth)))
        value = 0
        node = Node(x, y, value, smallerWidth)
        smallerNodeGrid.append(node)

    k = -1
    #print len(smallerNodeGrid)
    for i in range(0, len(nodeGrid) - 4):
        node = nodeGrid[i]
        #print (node.data == 100)
        neighbors = node.getAllNeighbors(nodeGrid)
        # print neighbors
        col = (i % 9)
        row = int(math.floor(i/width))
        if((row - 1)%3 == 0):
            if (i%3 == 1):
                k = k + 1
                for neighbor in neighbors:
                    if(neighbor.data == 100):
                        lowResNode = smallerNodeGrid[k]	
                        lowResNode.data = 100
        #print "data: %d" %(smallerNodeGrid[int(math.floor(i/3))].data)
	

    nodeGridCopy = copy.deepcopy(smallerNodeGrid)


    for i in range(0, len(nodeGridCopy)):
        #y = math.floor(i / smallerWidth)
        #x = i - (y * smallerWidth)
        node = smallerNodeGrid[i]
        if(node.data == 100):
            for neighbor in node.getNeighbors(smallerNodeGrid):
                index = int(math.floor(neighbor.x + neighbor.y * smallerWidth))
                neighborNode = nodeGridCopy[index]
                neighborNode.data = 100


    #print nodeGridCopy
    print data.info
    print k

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
    waypoints = AStar.getWaypoints(thisPath)
    waypoints.append(goalNode)
    waypoints.append(startPosNode)

    print "goal", goal.pose
    publishPath(thisPath, waypoints)
    for i in range(0, len(waypoints)):
        newPoseX = waypoints[i].x * resolution
        newPoseY = waypoints[i].y * resolution
        newPose = Pose()
        newPose.position.x = newPoseX
        newPose.position.y = newPoseY
        Driving.navToPose(newPose)

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

    for i in range(0,len(grid)): #height should be set to height of grid
        #print k # used for debugging
        thisNode = grid[i]
        thatNode = nodes[i]
        #print "data %d" % thisNode.data
        #print "x: %d y: %d" %(thisNode.x,thisNode.y)
        x = int(i % smallerWidth)
        y = int(math.floor(i / smallerWidth))
        if (thisNode.data == 100):
            point=Point()
            point.x=(x*resolution*3)+offsetX + (1.0* resolution*3) # added secondary offset 
            point.y=(y*resolution*3)+offsetY - (-0.5 * resolution*3) # added secondary offset ... Magic ?
            point.z=0
            cells.cells.append(point)
        if(thatNode.data == 100):
            point=Point()
            point.x=(x*resolution*3)+offsetX + (.5 * resolution*3) # added secondary offset 
            point.y=(y*resolution*3)+offsetY - (-0.5 * resolution*3) # added secondary offset ... Magic ?
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
        publishCells(smallerNodeGrid, nodeGridCopy) #publishing map data every 4 seconds
        rospy.sleep(4) 
        print("Complete")


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
