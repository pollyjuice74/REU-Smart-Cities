# PYTHONPATH=$(pwd)/ros_node_logic:$PYTHONPATH python3 ros_node_logic/src/lidar_fusion_node.py

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import std_msgs
from message_filters import ApproximateTimeSynchronizer, Subscriber
from functools import partial
import numpy as np
from threading import Thread

from fusion.fusion_callback import fusion_callback, local_callback, sdr_callback
from fusion.sphere_lidar_publisher import generate_half_colored_sphere, pack_rgb

# Most of these are empty directories
# from storage.ipfs_store import store_to_ipfs
# from src.rsu_logic import ...
# from src.cav_logic import ...


class LidarFusionNode:
    def __init__(self, mode="sphere"):
        rospy.init_node('sdr_fusion_node', anonymous=True)
        self.mode = mode
        self.received_data_log = []

        # Publisher for fused output
        self.pub_fused = rospy.Publisher('/fused_pointcloud', PointCloud2, queue_size=10)

        # --- MODIFY THIS ON THE OTHER NODE ---
        # Start publisher threads for mock input data
        Thread(target=self._publish_mock_lidar, args=('/local_lidar', 'top_half')).start()
        # Thread(target=self._publish_mock_lidar, args=('/sdr_lidar', 'bottom_half')).start()
        # -------------------------------------

        # Subscribers with approximate time sync
        local_sub = Subscriber('/local_lidar', PointCloud2)
        sdr_sub = Subscriber('/sdr_lidar', PointCloud2)

        ats = ApproximateTimeSynchronizer([sdr_sub, local_sub], queue_size=10, slop=0.5)
        ats.registerCallback(partial(fusion_callback, pub=self.pub_fused, log=self.received_data_log))

        rospy.loginfo("[ROS] SDR Fusion Node Fully Started.")

    def _publish_mock_lidar(self, topic="/local_lidar", split_mode="top_half"):
        pub = rospy.Publisher(topic, PointCloud2, queue_size=1)
        rate = rospy.Rate(1)  # 1 Hz
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1)
        ]

        while not rospy.is_shutdown():
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'

            if self.mode == "sphere":
                points = generate_half_colored_sphere(
                    radius=1.0,
                    resolution=30,
                    color1=(255, 0, 0),
                    color2=(0, 0, 255),
                    mode=split_mode
                )
            elif self.mode == "random_pointcloud":
                points = np.hstack([
                    np.random.uniform(-1, 1, size=(100, 3)),
                    np.full((100, 1), pack_rgb(128, 128, 128))
                ])

            msg = pc2.create_cloud(header, fields, points)
            pub.publish(msg)
            rate.sleep()

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    node = LidarFusionNode(mode="sphere")  # or mode="random"
    node.spin()