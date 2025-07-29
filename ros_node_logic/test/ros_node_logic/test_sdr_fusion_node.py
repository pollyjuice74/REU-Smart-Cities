#!/usr/bin/env python3
import os
import time
import threading
import numpy as np
import rospy

from std_msgs.msg import Float64MultiArray
from sdr_fusion_node import main as fusion_main  # Import the real node

# Dummy publisher to /sdr_lidar
def rand_pointcloud_publisher():
    pub = rospy.Publisher("/sdr_lidar", Float64MultiArray, queue_size=10)
    rospy.init_node("dummy_sdr_publisher", anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        dummy_data = np.random.rand(100, 3).flatten()
        msg = Float64MultiArray(data=dummy_data.tolist())
        pub.publish(msg)
        rate.sleep()


def run_test():
    # Thread 1: Dummy sensor publishers
    threading.Thread(target=publish_dummy_sdr, daemon=True).start()
    
    # Thread 2: Start the fusion node
    threading.Thread(target=fusion_main, daemon=True).start()

    # Wait and observe
    print("Test started. Waiting to receive and fuse data...")
    try:
        while not rospy.is_shutdown():
            time.sleep(1)
    except KeyboardInterrupt:
        print("Test interrupted.")

if __name__ == "__main__":
    run_test()
