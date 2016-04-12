#!/usr/bin/env python

from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
from Node import Node
import Driving
import rospy, tf, numpy, math
import AStar
import copy


# reads in global map
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    global nodeGridCopy
    global nodeGrid
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

    nodeGridCopy = copy.deepcopy(nodeGrid)

    for i in range(0, len(nodeGrid)):
        y = math.floor(i / width)
        x = i - (y * width)
        node = nodeGrid[int(x + (y * width))]
        if(node.data == 100):
            for neighbor in node.getNeighbors(nodeGrid):
                index = int(math.floor(neighbor.x + neighbor.y * width))
                nodeGridCopy[index].data = 100

    print data.info

def readGoal(goal):
    global goalX
    global goalY
    global pose
    print 'hi'
 
    goalX = int(goal.pose.position.x / resolution)
    goalY = int(goal.pose.position.y / resolution)

    indexGoal = int(math.floor(goalX + (goalY*width)))

    #COnvert the goal to a Node object
    goalNode = Node(goalX, goalY, mapData[indexGoal], width)
    thisPath = AStar.AStar(startPosNode, goalNode, nodeGridCopy)
    waypoints = []
    waypoints.append(startPosNode)
    waypoints.extend(AStar.getWaypoints(thisPath))
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
    
    #Cconvert start node to a Node Object. 
    startPosNode = Node(startPosX, startPosY, mapData[indexStart], width)

def publishPath(path, waypoints):
    global pubpath
    global pubway

    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution

    cells2 = GridCells()
    cells2.header.frame_id = 'map'
    cells2.cell_width = resolution
    cells2.cell_height = resolution

    for node in path:
        point = Point()
        point.x = (node.x * resolution) + (0.5 * resolution)#offsetX + (1.5 * resolution)
        point.y=(node.y * resolution) + (.5 * resolution) #offsetY - (.5 * resolution)
        point.z = 0
        cells.cells.append(point)

    for node in waypoints:
        point = Point()
        point.x = (node.x * resolution) + (0.5 * resolution)#offsetX + (1.5 * resolution)
        point.y=(node.y * resolution) + (.5 * resolution) #offsetY - (.5 * resolution)
        point.z = 0
        cells2.cells.append(point)

    pubpath.publish(cells)
    pubway.publish(cells2)



#publishes map to rviz using gridcells type

def publishCells(grid, nodes):
    global pub
    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    cells2 = GridCells()
    cells2.header.frame_id = 'map'
    cells2.cell_width = resolution 
    cells2.cell_height = resolution

    for i in range(1,height): #height should be set to hieght of grid
        k=k+1
        for j in range(1,width): #width should be set to width of grid
            k=k+1
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=(j*resolution)+offsetX + (1.5 * resolution) # added secondary offset 
                point.y=(i*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
                point.z=0
                cells.cells.append(point)
            thisNode = nodes[j + (i * width)]
            if(thisNode.data == 100):
                point=Point()
                point.x=(j*resolution)+offsetX + (.5 * resolution) # added secondary offset 
                point.y=(i*resolution)+offsetY - (-0.5 * resolution) # added secondary offset ... Magic ?
                point.z=0
                cells2.cells.append(point)

    pub.publish(cells)
    expand_pub.publish(cells2)



#Main handler of the project
def run():
    global pub
    global pubpath
    global pubway
    global expand_pub
    global nodeGridCopy


    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('/goal_lab3', PoseStamped, readGoal, queue_size=1) #change topic for best results
    start_sub = rospy.Subscriber('/initpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results
    expand_pub = rospy.Publisher('/expand', GridCells, queue_size=1)
    
    
    # wait a second for publisher, subscribers, and TF
    rospy.sleep(3)

    while (1 and not rospy.is_shutdown()):
        publishCells(mapData, nodeGridCopy) #publishing map data every 2 seconds
        rospy.spin()
        rospy.sleep(2)  
        print("Complete")


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
