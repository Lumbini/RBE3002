#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
import copy


wheel_rad = 3.5 / 100.0 #cm
wheel_base = 23.0 / 100.0 #cm


def publishTwist(lin_Vel, ang_Vel):
    """Send a movement (twist) message."""
    global pubDrive
    msg = Twist()
    msg.linear.x = lin_Vel
    msg.angular.z = ang_Vel
    pubDrive.publish(msg)


def navToPose(nextPose):
    """Drive to a goal subscribed to from /move_base_simple/goal"""
    #compute angle required to make straight-line move to desired pose
    global xPosition
    global yPosition
    global theta
    global pose
    print "Naving to Pose"
    #initialTurn = 0;
    #capture desired x and y positions
    desiredY = nextPose.position.y
    desiredX = nextPose.position.x
    #capture desired angle
    # quat = nextPose.orientation
    # q = [quat.x, quat.y, quat.z, quat.w]

    # roll, pitch, yaw = euler_from_quaternion(q)
    # desiredT = yaw * (180.0/math.pi)

    # print q, roll, pitch, yaw, desiredT

    #compute distance to target
    differenceX = pose.position.x - desiredX
    differenceY = pose.position.y - desiredY
    distance = math.sqrt(math.pow(differenceX, 2) + math.pow(differenceY, 2))

    print "From: %d, %d To: %d, %d" % (pose.position.x, pose.position.y, nextPose.position.x, nextPose.position.y)

    #compute initial turn amount
    initialTurn = math.degrees(math.atan(differenceY/differenceX))


    #Mapping the initial turn on a scale of 0-360
    print differenceX, differenceY, initialTurn
    if(differenceX < 0) and (differenceY < 0):
        #Puts the initial Turn in the first quadrant
        #initial turn will be positive, leave it alone 0-90
        initialTurn = math.degrees(math.atan(differenceY/differenceX))

    elif (differenceX > 0) and (differenceY < 0):
        #Puts the initial turn in the 2nd quadrant
        #Initial turn will be negative so add 180 for angle between 90-180
        initialTurn += 180

    elif (differenceX > 0) and (differenceY > 0):
        #Puts initial turn in third quadrant 
        #Initial turn will be positive change to between 0-270
        initialTurn += 180
    elif (differenceX < 0) and (differenceY > 0):
        #Puts initial turn in the 4th quadrant
        #Initial turn will be negative change to between 270-360
        initialTurn += 360

    print "spin!" #turn to calculated angle
    print "initialTurn: %d" % initialTurn
    rotate(initialTurn)
    initialTurn = 0;
    print "move!" #move in straight line specified distance to new pose
    driveSmooth(0.5, distance)
    print "spin!" #spin to final angle 
    #rotate(desiredT)
    #print "finalTurn: %d" % desiredT
    print "done"



#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    """This function accepts two wheel velocities and a time interval."""
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
        pubDrive.publish(move_msg) #publishes the move_msg
    pubDrive.publish(stop_msg)

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    """This function accepts a speed and a distance for the robot to move in a straight line"""

    initialX = pose.position.x
    initialY = pose.position.y
    atDest = 0
    #Loop until the distance between the attached frame and the origin is equal to the
    #distance specified 
    while (not atTarget and not rospy.is_shutdown()):
        currentX = pose.position.x
        currentY = pose.position.y
        currentDistance = math.sqrt(math.pow((currentX - initialX), 2) + math.pow((currentY - initialY), 2))
     #Distance formula
        if (currentDistance >= distance):
            atDest = 1
            publishTwist(0, 0)
        else:
            publishTwist(speed, 0)
            rospy.sleep(0.15)


def driveSmooth(speed, distance):
    """This function accepts a speed and a distance for the robot to move in a smoothed straight line."""
    initX = pose.position.x
    initY = pose.position.y
    rampVel = 0.0
    atDest = 0
    percent = 0.5
    increment = speed / ((percent * (distance/speed))/ 0.03)
    while(atDest == 0 and not rospy.is_shutdown()):
        
        currentX = pose.position.x
        currentY = pose.position.y
        dist = math.sqrt(math.pow((currentX - initX), 2) + math.pow((currentY - initY), 2))
        if (rampVel < 0):
            rampVel = 0
            publishTwist(rampVel, 0)
            print "Pub1: "+ str(rampVel)
        if (dist >= distance):
            publishTwist(0, 0)
            print "Pub2: "+ str(rampVel)
            atDest = 1
        else:
            if((distance - dist) <= distance * percent and rampVel >= 0):
                rampVel -= increment
                publishTwist(rampVel, 0)
                print "Pub3: "+ str(rampVel)
            elif((distance - dist) >= distance * (1.0 - percent) and rampVel <= speed):
                rampVel += increment
                publishTwist(rampVel, 0)
                print "Pub4: " + str(rampVel)
            else:
                publishTwist(speed, 0)
                print "Pub5: " + str(rampVel)
	rospy.sleep(0.03)

def rotateJake(angle):
    global odom_list
    global pose
    if (angle > 180 or angle < -180):
        print "angle is too large or small"
        if(angle >= 180):
            angle = angle -360
        elif (angle <= -180):
            angle = angle + 360
    vel = Twist()   
    done = True

    # set rotation direction
    error = angle-math.degrees(pose.orientation.z)

    if error >= 0:
        turn = "left"
    else:
        turn = "right"

    while ((abs(error) >= 6) and not rospy.is_shutdown()):
        print "To angle: %d" % angle    
        print "theta: %d" % math.degrees(pose.orientation.z) ## prints for debugging
        if turn == "right":
            spinWheels(-.02, .02, .1) 
        else:
            spinWheels(.02, -.02, .1) 
        currentAngle = math.degrees(pose.orientation.z)
        error = angle - currentAngle
    vel.angular.z = 0.0
    pubDrive.publish(vel)

def rotate(angle):
    global odom_list
    global pose

    vel = Twist();   
    done = True

    # set rotation direction

    error = angle-theta
    #determine which way to turn based on the angle
    if angle < 90 and theta > 180:
        if theta - angle < 360 - theta + angle:
            turn = -1
        else:
            turn = 1
    elif angle < 180 and theta > 270:
        if theta - angle < 360 - theta + angle:
            turn = -1
        else:
            turn = 1
    elif theta < 180 and angle > 270:
        if theta + 360 - angle < angle - theta:
            turn = -1
        else:
            turn = 1
    elif theta < 90 and angle > 180:
        if theta + 360 - angle < angle - theta:
            turn = -1
        else:
            turn = 1
    elif error < 0:
        turn = -1
    else:
        turn = 1
    

    while ((abs(error) >= 2) and not rospy.is_shutdown()):
        #Use this while loop to start the robots motion and determine if you are at the right angle.    
        #print "theta: %d" % math.degrees(pose.orientation.z)
        print "Error: %d, Rotate Pose %d, To angle: %d" % (error, theta, angle)
       
        vel.angular.z = 0.3*turn
        pubDrive.publish(vel)
        error = angle - (theta)
    vel.angular.z = 0.0
    pubDrive.publish(vel)

def waypointsCallback(gridCells):
    global pose     #Provides acess to the current pose
    global finalGoal
    wayPoses = []   #List to which the gridcells is parsed as poses

    # Iterates through the cells in gridcells and appends to wayposes
    # to be passed into navToPose
    for index in range(0, len(gridCells.cells)):
        nextPose = Pose()
        nextPose.position.x = gridCells.cells[index].x
        nextPose.position.y = gridCells.cells[index].y
        nextPose.orientation.z = pose.orientation.z

        wayPoses.append(nextPose)
    
    wayPoseCopy = copy.deepcopy(wayPoses)
    # wayPoseCopy = reversed(wayPoses)
    for nextPose in wayPoseCopy:
        navToPose(nextPose)

    publishTwist(0., 0.)

def findGoal(goal):
    global pose     #Provides access to the current Pose
    global finalGoal    # Global Final Goal

    # Converting the Poststamped to a pose.
    finalGoal = Pose()
    finalGoal = goal.pose.position.x
    finalGoal = goal.pose.position.y
    finalGoal = goal.pose.orientation.z


#keeps track of current location and orientation
def tCallback(event):
    
    global pose
    global xPosition
    global yPosition
    global theta

    now = rospy.Time.now() 

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    pose.position.x=position[0]
    pose.position.y=position[1]
    # the previous 2 lines and next 2 lines are repetative. Joes bad
    xPosition=position[0]
    yPosition=position[1]

    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)
    #convert yaw to degrees
    pose.orientation.z = yaw
    theta = math.degrees(yaw)
    if (theta < 0):
        theta = 360 + theta

if __name__ == '__main__':
    global pubDrive
    global odom_list
    global odom_tf
    global pose

    pose = Pose()
    #Open a new node for driving
    rospy.init_node('Driving_AStar')

    #Driving publisher
    pubDrive = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    goal_sub_Drive = rospy.Subscriber('/goal_lab3', PoseStamped, findGoal, queue_size=1) #Subscribe to the set goal
    waypoints_sub = rospy.Subscriber('/waypoints', GridCells, waypointsCallback, queue_size=1) #Subscribe to the set waypoints
    rospy.Timer(rospy.Duration(.01), tCallback) # timer callback for robot location
    odom_list = tf.TransformListener() #listner for robot location

    rospy.sleep(2)

    while not rospy.is_shutdown():
        rospy.spin()