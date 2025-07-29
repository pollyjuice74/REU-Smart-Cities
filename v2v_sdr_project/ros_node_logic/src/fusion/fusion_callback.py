import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import numpy as np


def local_callback(msg):
    global latest_local
    rospy.loginfo("[ROS] Received local_lidar data")
    latest_local = msg

def sdr_callback(msg):
    global latest_sdr
    rospy.loginfo("[ROS] Received sdr_lidar data")
    latest_sdr = msg
    
# === FUSION CALLBACK ===
def fusion_callback(sdr_msg, local_msg, pub, log):
    rospy.loginfo("[Fusion] Received SDR and LOCAL PointClouds")

    # Read points, including RGB if available
    sdr_points = list(pc2.read_points(sdr_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))
    local_points = list(pc2.read_points(local_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))

    fused_points = np.vstack([sdr_points, local_points])

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'

    fields = [
        PointField('x', 0,  PointField.FLOAT32, 1),
        PointField('y', 4,  PointField.FLOAT32, 1),
        PointField('z', 8,  PointField.FLOAT32, 1),
        PointField('rgb', 12, PointField.FLOAT32, 1),
    ]
    msg = pc2.create_cloud(header, fields, fused_points)
    pub.publish(msg)

    log.append(fused_points)