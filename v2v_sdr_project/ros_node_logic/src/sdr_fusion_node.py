#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # or 'Qt5Agg' if you have it

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
    lidar_array = np.array(lidar_data.data) # OS's own data from sensors (/velodyne_points)

    print("SDR data:", sdr_array)
    print("LIDAR for sdr,sensor data shape:", sdr_array.shape, lidar_array.shape)
    rospy.loginfo(f"[Fusion] SDR: {sdr_array[:3]}, LIDAR: {lidar_array[:3]}")

    # Example fusion: concatenate
    fused = np.concatenate((sdr_array, lidar_array))

    # Optional: do localization/mapping here
    received_data_log.append(fused)

        # Check if lidar_array is raw bytes or already a float array
    if lidar_array.dtype == np.float32:
        data = lidar_array
    else:
        data = np.array(lidar_array, dtype=np.float32)

    if len(data) % 4 == 0:
        points = np.reshape(data, (-1, 4))
        x, y = points[:, 0], points[:, 1]
        plt.clf()
        plt.scatter(x, y, s=1)
        timestamp = rospy.Time.now().to_sec()
        filepath = f"/tmp/lidar_plot_{timestamp:.0f}.png"
        plt.savefig(filepath)
        rospy.loginfo(f"[Plot] Saved LIDAR plot to {filepath}")
    else:
        rospy.logwarn(f"[Plot] LIDAR data shape invalid: {data.shape}")


def main():
    rospy.init_node('sdr_fusion_node', anonymous=True)
    # rospy.loginfo("[ROS] ")

    sdr_sub = Subscriber('/sdr_lidar', Float64MultiArray)
    lidar_sub = Subscriber('/velodyne_points', Float64MultiArray)  # Replace topic/type as needed

    ats = ApproximateTimeSynchronizer([sdr_sub, lidar_sub], queue_size=10, slop=0.1, allow_headerless=True)
    ats.registerCallback(fused_callback)
    plt.ion() # ???

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
