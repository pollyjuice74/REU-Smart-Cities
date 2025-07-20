#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

received_data_log = []

def sdr_callback(data):
    array = np.array(data.data)
    rospy.loginfo(f"[ROS] Received SDR data: {len(array)} elements")
    
    # Optional: show first few values for inspection
    rospy.loginfo(f"First 5 values: {array[:5]}")
    
    # Store for later visualization or processing
    received_data_log.append(array)

def main():
    rospy.init_node('sdr_fusion_node', anonymous=True)
    rospy.Subscriber('/sdr_lidar', Float64MultiArray, sdr_callback)
    
    rospy.loginfo("[ROS] SDR Fusion Node Started")
    
    # Keep spinning until shutdown
    rospy.spin()

    # Optional: Save received data after shutdown
    if received_data_log:
        np.save("/tmp/sdr_received_data.npy", np.array(received_data_log))
        rospy.loginfo(f"[ROS] Saved received data to /tmp/sdr_received_data.npy")

if __name__ == '__main__':
    main()
