import rospy
import time
import ros_numpy
import numpy as np
import os

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from src.memory.cav_memory_adapter import CAVMemoryManager
from src.storage.ipfs_store import store_file

mem = CAVMemoryManager()
fusion_pub = None  # Global for use in callback


# Called when the SDR receives a message and fuses it to the topic
# This would be a live update from a:
    # CAV: Fuse for larger field of vision
    # RSU: Fuse to your local lidar and have a rough map of the area
def cav_callback(msg):
    # Convert PointCloud2 to numpy array
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    tmp_path = "/tmp/lidar.npy"
    np.save(tmp_path, arr)

    with open(tmp_path, "rb") as f:
        cloud_bytes = f.read()

    metadata = {
        'timestamp': int(time.time()),
        'location': 'CAV_1',
        'source_id': 'lidar_sensor_01',
        'id': str(time.time_ns())
    }

    # Update local memory
    mem.update_local_model(cloud_bytes)
    mem.save(cloud_bytes, metadata)

    # Optional: IPFS upload
    ipfs_hash = store_file(tmp_path)

    # Publish hash or metadata to RSU
    fusion_pub.publish(ipfs_hash)
    rospy.loginfo(f"[CAV] Published IPFS hash to RSU: {ipfs_hash}")

def run_cav_node():
    global fusion_pub
    fusion_pub = rospy.Publisher("/fused_pointcloud", String, queue_size=10)
    rospy.Subscriber("/local_lidar", PointCloud2, cav_callback)


if __name__ == "__main__":
    run_cav_node()
