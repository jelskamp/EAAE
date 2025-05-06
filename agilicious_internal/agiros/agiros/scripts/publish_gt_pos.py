#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry



def publish_gt_pos():
    rospy.init_node('publish_gt_pos', anonymous=True)
    pub = rospy.Publisher('/drone/gt_pose', PoseStamped, queue_size=10)
    # pub_odom = rospy.Publisher('/drone/odom', Odometry, queue_size=10)
    listener = tf.TransformListener()

    rate = rospy.Rate(500.0)  # 500 Hz
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/world', '/kingfisher/base_link', rospy.Time(0))
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'world'
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]
            pub.publish(pose)

            # odom = Odometry()
            # odom.header.stamp = rospy.Time.now()
            # odom.header.frame_id = 'world'
            # odom.child_frame_id = 'base_link'
            # odom.pose.pose.position.x = trans[0]
            # odom.pose.pose.position.y = trans[1]
            # odom.pose.pose.position.z = trans[2]
            # odom.pose.pose.orientation.x = rot[0]
            # odom.pose.pose.orientation.y = rot[1]
            # odom.pose.pose.orientation.z = rot[2]
            # odom.pose.pose.orientation.w = rot[3]
            # pub_odom.publish(odom)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_gt_pos()
    except rospy.ROSInterruptException:
        pass