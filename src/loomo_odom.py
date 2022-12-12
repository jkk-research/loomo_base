#!/usr/bin/env python

import rospy
from tf2_msgs.msg import TFMessage
import tf2_ros
import geometry_msgs.msg

import argparse
import os
import yaml

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--prev', action='store_true', help='This argument loads the very last position of the loomo.')

    args = parser.parse_args()

    rospy.init_node('loomo_odom_tf')

    yamlFile = "last_odom_pos.yaml"
    scriptDir = os.path.dirname(os.path.realpath(__file__))
    resourceDir = scriptDir+"/../config/"
    filePath= resourceDir+yamlFile

    t = geometry_msgs.msg.TransformStamped()
    newFrameMessage = tf2_ros.TransformBroadcaster()
    t.header.frame_id = "LO01_odom"
    t.child_frame_id = "LO01_origo"

    if not args.prev:
        initPoint = rospy.wait_for_message("/tf", TFMessage)

        t.header.seq = initPoint.transforms[0].header.seq
        initPoint = initPoint.transforms[0].transform
        
        t.transform.translation.x = initPoint.translation.x
        t.transform.translation.y = initPoint.translation.y
        t.transform.translation.z = initPoint.translation.z

        t.transform.rotation.x = initPoint.rotation.x
        t.transform.rotation.y = initPoint.rotation.y
        t.transform.rotation.z = initPoint.rotation.z
        t.transform.rotation.w = initPoint.rotation.w

        savePosition = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
                        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]

        with open(filePath, mode="wt") as f:
            yaml.dump(savePosition, f, Dumper=yaml.Dumper)
    else:
        with open(filePath) as f:
            data = yaml.load(f, Loader=yaml.SafeLoader)
        
        t.transform.translation.x = data[0]
        t.transform.translation.y = data[1]
        t.transform.translation.z = data[2]

        t.transform.rotation.x = data[3]
        t.transform.rotation.y = data[4]
        t.transform.rotation.z = data[5]
        t.transform.rotation.w = data[6]

    rate = rospy.Rate(30.0)

    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        newFrameMessage.sendTransform(t)
        t.header.seq += 1
        rate.sleep()