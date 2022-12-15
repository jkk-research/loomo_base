#!/usr/bin/env python

import rospy
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, TransformStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from math import atan2, sqrt, pi
import tf
import os
import yaml


def distance(actual, goal):
    return sqrt((actual[0]-goal.x)**2+(actual[1]-goal.y)**2)

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

    origoPointInit = False
    origo_x = 0.0
    origo_y = 0.0
    origo_theta = 0.0

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
        marker_all.header.frame_id = "LO01_origo"
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
    marker.header.frame_id = "LO01_origo"
    marker.type = 3
    marker.scale.x = 0.11
    marker.scale.y = 0.11
    marker.scale.z = 1.01

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

    listener = tf.TransformListener()

    pub = rospy.Publisher("/LO01/cmd_vel", Twist, queue_size = 1)
    pub_marker = rospy.Publisher("actual_goal_point", Marker, queue_size = 1)
    pub_markerArray = rospy.Publisher("all_goal_points", MarkerArray, queue_size = 4)

    while not rospy.is_shutdown():

        try:
            (trans,rot) = listener.lookupTransform("LO01_origo", "LO01_base_link", rospy.Time(0))
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # print(trans)
        # print(rot)
        
        (roll, pitch, theta) = euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])

        x = trans[0]
        y = trans[1]

        inc_x = goal.x - x
        inc_y = goal.y - y

        angle_to_goal = atan2(inc_y, inc_x)
        actual_point = (x, y)

        if distance(actual_point, goal) > 0.25:
            if abs(angle_to_goal - theta) > 0.1:
                speed.linear.x = 0.0
                speed.angular.z = 0.3
            else:
                speed.linear.x = 0.1
                speed.angular.z = 0.0
            print("Angle diff:", abs(angle_to_goal - theta))
            pub.publish(speed)
        elif pointIdx < len(goal_points):
            goal.x = goal_points[pointIdx][0]
            goal.y = goal_points[pointIdx][1]
            marker.pose.position.x = goal.x
            marker.pose.position.y = goal.y
            pointIdx += 1

        print("Distance: ", distance(actual_point, goal))
        print(" Goal:\n", '"', goal.x, goal.y, '"', "Idx:",pointIdx)
        pub_markerArray.publish(marker_array)
        pub_marker.publish(marker)
        
        r.sleep()  