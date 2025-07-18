#!/usr/bin/env python3
import rospy
from your_pkg.msg import DsrcDetection

def sdr_rx_loop():
    pub = rospy.Publisher('/dsrc_signals', DsrcDetection, queue_size=10)
    rospy.init_node('sdr_receiver', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # This part reads output from MATLAB (could be file, socket, etc.)
        detection = DsrcDetection()
        detection.header.stamp = rospy.Time.now()
        detection.azimuth = 45.0   # example
        detection.range = 10.0
        detection.snr = 18.3
        detection.dsrc_type = "BSM"
        pub.publish(detection)
        rate.sleep()

if __name__ == '__main__':
    try:
        sdr_rx_loop()
    except rospy.ROSInterruptException:
        pass
