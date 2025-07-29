# memory/cav_memory_adapter.py
from src.memory.memory_manager import MemoryManager
import os

class CAVMemoryManager(MemoryManager):
    def __init__(self, max_entries=10):
        super().__init__(storage_dir=os.path.join("/tmp/cav_memory", "CAV"))
        self.cache = []
        self.max_entries = max_entries
    
    def update_local_model(cloud_bytes):
        """
        Update CAV local short-term model based on point cloud.
        Placeholder.
        """
        print("Updating local CAV model with new point cloud data")

    def save(self, cloud_bytes, metadata):
        pkt = self.package(cloud_bytes, metadata)
        self.cache.append(pkt)
        if len(self.cache) > self.max_entries:
            self.cache.pop(0)
        print(f"[CAV] Cached point cloud. Cache size: {len(self.cache)}")

    def load(self, key=None):
        if not self.cache:
            return None
        if key:
            for pkt in self.cache:
                if pkt["metadata"]["id"] == key:
                    return pkt
        return self.cache[-1]  # latest if no key
