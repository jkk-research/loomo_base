#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import yaml
import os

def callback(msg):
    global waypoint
    waypoint.append([msg.pose.position.x, msg.pose.position.y])
    rospy.loginfo(f"{msg.pose.position.x} {msg.pose.position.y}")

def savePoints(points):
    yamlFile = "goal_points.yaml"
    scriptDir = os.path.dirname(os.path.realpath(__file__))
    resourceDir = scriptDir+"/../config/"
    filePath= resourceDir+yamlFile
    with open(filePath, mode="wt") as f:
        yaml.dump(points, f, Dumper=yaml.Dumper)

if __name__ == '__main__':
    rospy.init_node("waypoint_generator")
    waypoint = []
    sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    rospy.spin()
    savePoints(waypoint)