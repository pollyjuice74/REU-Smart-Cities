import rospy
import os
import time

from std_msgs.msg import String
from src.memory.rsu_memory_adapter import RSUMemoryManager
from src.storage.ipfs_store import get_file

mem = RSUMemoryManager()

# Called when the SDR receives a message and fuses it to the topic
# This would be a live update from a:
    # CAV: Update from the road for internal map
    # RSU: Synchronize database? Connects node in a chain as a sort of path
def rsu_callback(msg):
    ipfs_hash = msg.data
    output_path = f"/tmp/received_{ipfs_hash}.npy"

    # Fetch from IPFS
    get_file(ipfs_hash, output_path)

    with open(output_path, "rb") as f:
        cloud_bytes = f.read()

    metadata = {
        'timestamp': int(time.time()),
        'location': 'RSU_1',
        'source_id': 'lidar_sensor_01',
        'id': ipfs_hash
    }

    # Update map and save
    mem.update_map(cloud_bytes)
    mem.save_pointcloud(cloud_bytes, metadata)

    rospy.loginfo(f"[RSU] Stored received cloud from {ipfs_hash}")

def run_rsu_node():
    rospy.Subscriber("/fused_pointcloud", String, rsu_callback)


if __name__ == "__main__":
    run_rsu_node()