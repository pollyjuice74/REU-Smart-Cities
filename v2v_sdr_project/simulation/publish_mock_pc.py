#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import random

def talker():
    pub = rospy.Publisher("/velodyne_points", PointCloud2, queue_size=10)
    rospy.init_node("fake_lidar", anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"

        points = [(random.uniform(-10,10), random.uniform(-10,10), random.uniform(-2,2), 100) for _ in range(500)]
        cloud = pc2.create_cloud(header,
                                 [PointField("x", 0, PointField.FLOAT32, 1),
                                  PointField("y", 4, PointField.FLOAT32, 1),
                                  PointField("z", 8, PointField.FLOAT32, 1),
                                  PointField("intensity", 12, PointField.FLOAT32, 1)],
                                 points)
        pub.publish(cloud)
        rate.sleep()

if __name__ == '__main__':
    talker()
