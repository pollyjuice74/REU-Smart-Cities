import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import numpy as np

def fusion_callback(sdr_msg: PointCloud2, lidar_msg: PointCloud2, pub, log):
    rospy.loginfo("[Fusion] Received SDR PointCloud with %d points",
                  sdr_msg.width * sdr_msg.height)
    rospy.loginfo("[Fusion] Received LOCAL PointCloud with %d points",
                  lidar_msg.width * lidar_msg.height)
    
    # Convert to numpy arrays
    sdr_points = list(pc2.read_points(sdr_msg, field_names=("x", "y", "z"), skip_nans=True))
    lidar_points = list(pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True))

    # Combine
    fused_points = np.vstack([sdr_points, lidar_points])

    # Publish
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'

    fields = [
        PointField('x', 0,  PointField.FLOAT32, 1),
        PointField('y', 4,  PointField.FLOAT32, 1),
        PointField('z', 8,  PointField.FLOAT32, 1),
    ]
    fused_msg = pc2.create_cloud(header, fields, fused_points)
    pub.publish(fused_msg)

    # Optionally log
    log.append(fused_points)
