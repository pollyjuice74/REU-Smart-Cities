import time
from memory.rsu_memory_adapter import RSUMemoryManager

mem = RSUMemoryManager()


def _receive_point_cloud():
    """
    Replace with actual code to receive point cloud from ROS or socket
    For prototype, return dummy bytes and metadata
    """
    import os
    dummy_data = os.urandom(1024)  # random bytes as placeholder
    metadata = {
        'timestamp': int(time.time()),
        'location': 'RSU_1',
        'source_id': 'lidar_sensor_01'
    }
    return dummy_data, metadata

def _transmit_local_area_data():
    pass

def run_rsu_node():
    print("Starting RSU node...")
    while True:
        # Listen
        # RSU doesn't have LiDAR sensors...
        cloud_bytes, metadata = _receive_point_cloud() # Listen to transmitted LiDAR
        # Update
        mem.update_map(cloud_bytes) # Update internal spacial map
        mem.save_pointcloud(cloud_bytes, metadata) # Memory efficient save of local area
        # Publish
        _transmit_local_area_data() # TODO: Should transmit local area map data for CAV updating area state as well...