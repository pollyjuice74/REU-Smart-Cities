#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np

received_data_log = []

def sdr_callback(data):
    array = np.array(data.data)
    rospy.loginfo(f"[ROS] Received SDR data: {len(array)} elements")
    
    # Optional: show first few values for inspection
    rospy.loginfo(f"First 5 values: {array[:5]}")
    
    # Store for later visualization or processing
    received_data_log.append(array)

def fused_callback(sdr_data, lidar_data):
    sdr_array = np.array(sdr_data.data)
    lidar_array = np.array(lidar_data.data)

    rospy.loginfo(f"[Fusion] SDR: {sdr_array[:3]}, LIDAR: {lidar_array[:3]}")

    # Example fusion: concatenate
    fused = np.concatenate((sdr_array, lidar_array))

    # Optional: do localization/mapping here
    received_data_log.append(fused)

def main():
    rospy.init_node('sdr_fusion_node', anonymous=True)

    sdr_sub = Subscriber('/sdr_lidar', Float64MultiArray)
    lidar_sub = Subscriber('/velodyne_points', Float64MultiArray)  # Replace topic/type as needed

    ats = ApproximateTimeSynchronizer([sdr_sub, lidar_sub], queue_size=10, slop=0.1, allow_headerless=True)
    ats.registerCallback(fused_callback)

    rospy.loginfo("[ROS] SDR-LIDAR Fusion Node Running")
    rospy.spin()

    # Save fused data
    if received_data_log:
        np.save("/tmp/fused_data.npy", np.array(received_data_log))
        rospy.loginfo(f"[ROS] Fused data saved to /tmp/fused_data.npy")

# def main():
#     rospy.init_node('sdr_fusion_node', anonymous=True)
#     rospy.Subscriber('/sdr_lidar', Float64MultiArray, sdr_callback)
    
#     rospy.loginfo("[ROS] SDR Fusion Node Started")
    
#     # Keep spinning until shutdown
#     rospy.spin()

#     # Optional: Save received data after shutdown
#     if received_data_log:
#         np.save("/tmp/sdr_received_data.npy", np.array(received_data_log))
#         rospy.loginfo(f"[ROS] Saved received data to /tmp/sdr_received_data.npy")

if __name__ == '__main__':
    main()
