#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray

def sdr_callback(data):
    rospy.loginfo(f"[ROS] Received SDR data: {len(data.data)} elements")
    # Do whatever you want: fuse with lidar, store to file, etc.

def main():
    rospy.init_node('sdr_fusion_node')
    rospy.Subscriber('/sdr_lidar', Float64MultiArray, sdr_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
