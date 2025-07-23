import os
import json
import time
import threading


class MemoryManager:
    def __init__(self, storage_dir='rsu_storage'):
        self.storage_dir = storage_dir
        os.makedirs(self.storage_dir, exist_ok=True)
        self.index_file = os.path.join(self.storage_dir, 'index.json')
        self._load_index()

        # Lock for thread-safe operations
        self.lock = threading.Lock()

    def _load_index(self):
        if os.path.exists(self.index_file):
            with open(self.index_file, 'r') as f:
                self.index = json.load(f)
        else:
            self.index = {}

    def _save_index(self):
        with open(self.index_file, 'w') as f:
            json.dump(self.index, f, indent=2)

    def save_pointcloud(self, pointcloud_data, metadata):
        """
        Save pointcloud data and update index with version/timestamp.
        metadata should include e.g. 'location', 'timestamp', 'source_id'
        """
        with self.lock:
            timestamp = metadata.get('timestamp', int(time.time()))
            key = f"{metadata.get('source_id','unknown')}_{timestamp}"
            filename = os.path.join(self.storage_dir, f"{key}.bin")

            # Save raw binary data
            with open(filename, 'wb') as f:
                f.write(pointcloud_data)

            # Update index with metadata and filename
            self.index[key] = {
                'file': filename,
                'metadata': metadata,
                'version': self.index.get(key, {}).get('version', 0) + 1,
                'timestamp': timestamp
            }
            self._save_index()
            print(f"Saved pointcloud with metadata {metadata}")

    def get_pointcloud(self, key):
        """Retrieve pointcloud binary data by key"""
        with self.lock:
            entry = self.index.get(key)
            if not entry:
                return None
            with open(entry['file'], 'rb') as f:
                return f.read()

    def list_pointclouds(self):
        """Return list of all stored keys with metadata"""
        with self.lock:
            return self.index

    def prune_old_data(self, max_age_seconds=3600):
        """Remove data older than max_age_seconds"""
        with self.lock:
            now = int(time.time())
            keys_to_remove = []
            for key, entry in self.index.items():
                if now - entry['timestamp'] > max_age_seconds:
                    keys_to_remove.append(key)

            for key in keys_to_remove:
                try:
                    os.remove(self.index[key]['file'])
                except FileNotFoundError:
                    pass
                del self.index[key]

            if keys_to_remove:
                self._save_index()

    # Placeholder: method to merge / reconcile data from peer nodes
    def sync_with_peer(self, peer_metadata_list):
        """
        peer_metadata_list: list of metadata dicts from peer nodes
        This function will compare versions/timestamps and request missing data.
        """
        pass