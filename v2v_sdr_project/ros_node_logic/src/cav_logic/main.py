import time

from memory.cav_memory_adapter import CAVMemoryManager

mem = CAVMemoryManager()


def _get_local_lidar():
    """
    Replace with actual code to get local LiDAR point cloud data.
    For now, return dummy bytes and metadata.
    """
    import os
    dummy_data = os.urandom(1024)
    metadata = {
        'timestamp': int(time.time()),
        'location': 'CAV_1',
        'source_id': 'lidar_sensor_01'
    }
    return dummy_data, metadata

def _publish_to_rsu(cloud_bytes, metadata):
    """
    Send data to RSU or peers.
    Placeholder: print or send via ROS topic or socket
    """
    print(f"Publishing {len(cloud_bytes)} bytes to RSU with metadata {metadata}")

def run_cav_node():
    print("Starting CAV node...")
    while True:
        # Listen
        cloud_bytes, metadata = _get_local_lidar() # Listening to sensor LiDAR
        # TODO: Listen to RSU local area mapping data
        # Update
        mem.update_local_model(cloud_bytes) # Update local memory state, compressed data storage
        # Publish
        _publish_to_rsu(cloud_bytes, metadata) # Sending data to RSU