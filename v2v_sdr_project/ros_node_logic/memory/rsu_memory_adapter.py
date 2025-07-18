# memory/rsu_memory_adapter.py
import os
import json
from memory_manager import MemoryManager

class RSUMemoryManager(MemoryManager):
    def __init__(self, base_dir="./data/memory/rsu"):
        super().__init__("RSU", base_dir)

    def update_map(cloud):
        """
        Fuse point cloud into a global map / voxel grid.
        Placeholder function.
        """
        print("Fusing point cloud into RSU map...")

    def save_pointcloud(self, cloud_bytes, metadata):
        pkt = self.package(cloud_bytes, metadata)
        fname = os.path.join(self.base_dir, f"{metadata['id']}_{pkt['timestamp']}.json")
        with open(fname, "w") as f:
            json.dump(pkt, f)
        print(f"[RSU] Saved data to {fname}")

    def load(self, key):
        for f in os.listdir(self.base_dir):
            if key in f:
                with open(os.path.join(self.base_dir, f), "r") as file:
                    return json.load(file)
        print(f"[RSU] No memory found for key: {key}")
        return None
