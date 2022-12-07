#!/usr/bin/env python

import rospy
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, TransformStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from math import atan2, sqrt, pow
import tf
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
        
def initPosition(msg):
    print("Initializin starting position frame")
    print(msg.transforms[1].header.frame_id)

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

    marker_array = MarkerArray()
    
    idNum = 0
    for points in goal_points:
        marker_all = Marker()
        marker_all.header.frame_id = "LO01_odom"
        marker_all.type = 3
        marker_all.id = idNum
        marker_all.scale.x = 0.1
        marker_all.scale.y = 0.1
        marker_all.scale.z = 1.0

        marker_all.color.r = 1.0
        marker_all.color.g = 0.0
        marker_all.color.b = 0.0
        marker_all.color.a = 1.0

        marker_all.pose.position.x = points[0]
        marker_all.pose.position.y = points[1]
        marker_all.pose.position.z = 0

        marker_all.pose.orientation.x = 0.0
        marker_all.pose.orientation.y = 0.0
        marker_all.pose.orientation.z = 0.0
        marker_all.pose.orientation.w = 1.0

        marker_array.markers.append(marker_all)
        idNum += 1

    marker = Marker()
    marker.header.frame_id = "LO01_odom"
    marker.type = 3
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 1.0

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.pose.position.x = goal.x
    marker.pose.position.y = goal.y
    marker.pose.position.z = 0

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    rospy.init_node("speed_controller")

    r = rospy.Rate(4)

    sub = rospy.Subscriber("/tf", TFMessage, newOdom)
    pub = rospy.Publisher("/LO01/cmd_vel", Twist, queue_size = 1)
    pub_marker = rospy.Publisher("actual_goal_point", Marker, queue_size = 1)
    pub_markerArray = rospy.Publisher("all_goal_points", MarkerArray, queue_size = 4)


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
        pub_marker.publish(marker)
        pub_markerArray.publish(marker_array)
        
        r.sleep()  