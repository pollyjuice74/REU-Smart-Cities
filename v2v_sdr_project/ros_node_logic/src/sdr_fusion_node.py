#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from functools import partial

# Most of these are empty directories
from fusion.fusion_callback import fusion_callback
from storage.ipfs_store import store_to_ipfs
# from src.rsu_logic import ...
# from src.cav_logic import ...

latest_local = None
latest_sdr = None
pointcloud_pub = None
received_data_log = [] 

def local_callback(msg):
    global latest_local
    rospy.loginfo("[ROS] Received local_lidar data")
    latest_local = msg

def sdr_callback(msg):
    global latest_sdr
    rospy.loginfo("[ROS] Received sdr_lidar data")
    latest_sdr = msg

def main():
    global pointcloud_pub
    rospy.init_node('sdr_fusion_node', anonymous=True)

    pointcloud_pub = rospy.Publisher('/fused_pointcloud', PointCloud2, queue_size=10)

    # Subscribers for individual logging (not strictly needed with message_filters, but good for debug)
    rospy.Subscriber('/local_lidar', PointCloud2, local_callback)
    rospy.Subscriber('/sdr_lidar', PointCloud2, sdr_callback)

    # Synced fusion subscriber
    sdr_sub = Subscriber('/sdr_lidar', PointCloud2)
    local_sub = Subscriber('/local_lidar', PointCloud2)

    ats = ApproximateTimeSynchronizer([sdr_sub, local_sub], queue_size=10, slop=0.5)
    ats.registerCallback(partial(fusion_callback, pub=pointcloud_pub, log=received_data_log))

    # Storage

    rospy.loginfo("[ROS] SDR Fusion Node Started.")
    rospy.spin()


if __name__ == '__main__':
    main()
