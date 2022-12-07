#!/usr/bin/env python

import rospy
from tf2_msgs.msg import TFMessage
import tf


if __name__ == '__main__':

    initPoint = rospy.wait_for_message("/tf", TFMessage)

    initPoint = initPoint.transforms[0].transform

    newFrameMessage = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        newFrameMessage.sendTransform((-initPoint.translation.x, -initPoint.translation.y, -initPoint.translation.z),
                                        (0.0, 0.0, 0.0, 1.0),
                                        rospy.Time.now(),
                                        "LO01_odom",
                                        "LO01_origo")
        rate.sleep()