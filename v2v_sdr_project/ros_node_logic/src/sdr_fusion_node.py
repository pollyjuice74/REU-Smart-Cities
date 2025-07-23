#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from functools import partial

# Most of these are empty directories
from fusion.fusion_callback import fusion_callback
from src.rsu_logic.rsu_main import run_rsu_node
from src.cav_logic.cav_main import run_cav_node

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

def main(mode="RSU"):
    global pointcloud_pub
    rospy.init_node(f'{mode}_sdr_fusion_node', anonymous=True)

    # Interface topic for IPFS since all sdr topics would be fused with local here immediately
    pointcloud_pub = rospy.Publisher('/fused_pointcloud', PointCloud2, queue_size=10)

    # Subscribers for individual logging (not strictly needed with message_filters, but good for debug)
    rospy.Subscriber('/local_lidar', PointCloud2, local_callback)
    rospy.Subscriber('/sdr_lidar', PointCloud2, sdr_callback)

    # Synced fusion subscriber
    sdr_sub = Subscriber('/sdr_lidar', PointCloud2)
    local_sub = Subscriber('/local_lidar', PointCloud2)

    ats = ApproximateTimeSynchronizer([sdr_sub, local_sub], queue_size=10, slop=0.5)
    ats.registerCallback(partial(fusion_callback, pub=pointcloud_pub, log=received_data_log))

    # Handles IPFS database logic for either a centralized autonomous vehicle CAV or a road side unit RSU
    if mode == "CAV":
        run_cav_node()
    elif mode == "RSU":
        run_rsu_node()
    else:
        rospy.logwarn(f"[ROS] Unknown mode: {mode}. No memory logic initialized.")

    rospy.loginfo(f"[ROS] {mode} SDR Fusion Node Started.")
    rospy.spin()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Run SDR Fusion Node for RSU or CAV")
    parser.add_argument(
        "--mode",
        choices=["RSU", "CAV"],
        default="RSU",
        help="Select node mode: RSU or CAV (default: RSU)"
    )

    args = parser.parse_args()
    main(mode=args.mode)