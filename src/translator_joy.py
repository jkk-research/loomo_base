#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Translator:
    def __init__(self):
        self.subA = rospy.Subscriber("joy", Joy, self.callbackJoy)
        self.pubA = rospy.Publisher("/LO01/cmd_vel", Twist, queue_size=1)
        #rospy.loginfo("init")
    
    def callbackJoy(self, message):
        #rospy.loginfo("new message: %.2f %.2f" % (message.axes[0], message.axes[1]))
        twistCmd = Twist()
        twistCmd.linear.x = message.axes[0]
        twistCmd.angular.z = message.axes[1]
        self.pubA.publish(twistCmd)

if __name__ == '__main__':
    rospy.init_node("translator_joystick")
    rospy.loginfo("Loomo translator_joystick started")
    t = Translator()
    rospy.spin()