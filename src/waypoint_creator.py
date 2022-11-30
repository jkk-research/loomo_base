#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from math import pow, atan2, sqrt

def callback(msg):
    global waypoint
    waypoint.append([msg.pose.position.x, msg.pose.position.y])
    print(waypoint)

if __name__ == '__main__':
    rospy.init_node("waypoint_generator")
    waypoint = []
    sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    rospy.spin()