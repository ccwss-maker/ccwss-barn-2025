#!/usr/bin/env python  
import rospy
import tf
from nav_msgs.msg import Odometry

def handle_odom(msg):
    br = tf.TransformBroadcaster()
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    br.sendTransform(
        (pos.x, pos.y, pos.z),
        (ori.x, ori.y, ori.z, ori.w),
        msg.header.stamp,
        msg.child_frame_id,   # usually "base_link"
        "odom"                
    )

if __name__ == '__main__':
    rospy.init_node('odom_tf_broadcaster')
    rospy.Subscriber("/odometry/filtered", Odometry, handle_odom)
    rospy.spin()
