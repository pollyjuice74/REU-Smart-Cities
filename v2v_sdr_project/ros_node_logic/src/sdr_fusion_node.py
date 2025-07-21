#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # or 'Qt5Agg' if you have it

# Most of these are empty directories
# from src.visualization import ...
from fusion.fused_callback import fused_callback
from storage.ipfs_store import store_to_ipfs
# from src.rsu_logic import ...
# from src.cav_logic import ...

pointcloud_pub = None 
received_data_log = [] 

def main():
    # First visualize received data, via print, rviz, etc.
    # Then fuse that data with some algorithm, better if its a ros one
    # Lastly save fused data to internal ipfs storage
        # CAVs collect data (/velodyne_points) RSUs fuse & store data compressed (/sdr_lidar)
    rospy.init_node('sdr_fusion_node', anonymous=True)

    # Publisher for RViz point cloud
    pointcloud_pub = rospy.Publisher('/fused_pointcloud', PointCloud2, queue_size=10)

    sdr_sub = Subscriber('/sdr_lidar', PointCloud2)
    lidar_sub = Subscriber('/velodyne_points', PointCloud2)  # Replace topic/type as needed

    ats = ApproximateTimeSynchronizer([sdr_sub, lidar_sub], queue_size=10, slop=0.1, allow_headerless=True)
    ats.registerCallback(lambda sdr, lidar: fused_callback(sdr, lidar, pointcloud_pub, received_data_log))

    # Then save data

    rospy.loginfo("[ROS] SDR-LIDAR Fusion Node Running")
    rospy.spin()


if __name__ == '__main__':
    main()
