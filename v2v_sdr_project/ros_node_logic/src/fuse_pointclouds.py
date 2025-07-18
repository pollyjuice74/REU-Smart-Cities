#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import message_filters

def callback(velodyne_msg, sdr_msg):
    # TODO: Convert both messages to PCL, concatenate, and publish
    rospy.loginfo("Fusing Velodyne and SDR point clouds")

rospy.init_node('fuse_pointclouds')

sub1 = message_filters.Subscriber('/velodyne_points', PointCloud2)
sub2 = message_filters.Subscriber('/sdr_points', PointCloud2)

ts = message_filters.ApproximateTimeSynchronizer([sub1, sub2], queue_size=10, slop=0.1)
ts.registerCallback(callback)

rospy.spin()
