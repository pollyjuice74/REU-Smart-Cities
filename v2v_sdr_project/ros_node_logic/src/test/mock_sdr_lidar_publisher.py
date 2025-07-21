#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import numpy as np

def mock_sdr_lidar_publisher():
    rospy.init_node('mock_sdr_lidar_publisher')
    
    pub_sdr = rospy.Publisher('/sdr_lidar', PointCloud2, queue_size=1)
    pub_lidar = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=1)
    
    rate = rospy.Rate(1)

    fields = [
        PointField('x', 0,  PointField.FLOAT32, 1),
        PointField('y', 4,  PointField.FLOAT32, 1),
        PointField('z', 8,  PointField.FLOAT32, 1),
    ]

    while not rospy.is_shutdown():
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        # SDR PointCloud
        points_sdr = np.random.rand(100, 3)
        pc2_msg_sdr = pc2.create_cloud(header, fields, points_sdr)
        pub_sdr.publish(pc2_msg_sdr)

        # LIDAR PointCloud
        points_lidar = np.random.rand(100, 3) + 2.0  # Just to make them different
        pc2_msg_lidar = pc2.create_cloud(header, fields, points_lidar)
        pub_lidar.publish(pc2_msg_lidar)

        rate.sleep()

if __name__ == '__main__':
    mock_sdr_lidar_publisher()
