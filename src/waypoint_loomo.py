#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt, pow
import os
import yaml

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.transforms[0].transform.translation.x
    y = msg.transforms[0].transform.translation.y
    
    # print(msg.transforms[0].header.frame_id)

    rot_q = msg.transforms[0].transform.rotation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def distance(actual, goal):
    return sqrt(pow(actual[0]-goal.x, 2)+pow(actual[1]-goal.y, 2))

def loadCoordinates(filePath):
    print(f"Opening: {filePath}...")
    with open(filePath) as f:
        data = yaml.load(f, Loader=yaml.SafeLoader)
    print(data)
    return data
        

if __name__ == '__main__':

    yamlFile = "goal_points.yaml"
    scriptDir = os.path.dirname(os.path.realpath(__file__))
    resourceDir = scriptDir+"/../config/"
    filePath= resourceDir+yamlFile

    goal_points = loadCoordinates(filePath)
    print(goal_points)
    pointIdx = 0

    x = 0.0
    y = 0.0 
    theta = 0.0
    
    speed = Twist()

    goal = Point()
    goal.x = goal_points[pointIdx][0]
    goal.y = goal_points[pointIdx][1]

    print(goal)

    rospy.init_node("speed_controller")

    r = rospy.Rate(4)
    sub = rospy.Subscriber("/tf", TFMessage, newOdom)
    pub = rospy.Publisher("/LO01/cmd_vel", Twist, queue_size = 1)
    while not rospy.is_shutdown():
        inc_x = goal.x -x
        inc_y = goal.y -y

        angle_to_goal = atan2(inc_y, inc_x)
        actual_point = (x, y)

        if distance(actual_point, goal) > 0.25:
            if abs(angle_to_goal - theta) > 0.1:
                speed.linear.x = 0.0
                speed.angular.z = 0.3
            else:
                speed.linear.x = 0.1
                speed.angular.z = 0.0
            pub.publish(speed)
        elif pointIdx < len(goal_points):
            goal.x = goal_points[pointIdx][0]
            goal.y = goal_points[pointIdx][1]
            pointIdx += 1

        print(distance(actual_point, goal))
        print(" Goal:\n", '"', goal.x, goal.y, '"', "Idx:",pointIdx)
        
        
        
        r.sleep()  