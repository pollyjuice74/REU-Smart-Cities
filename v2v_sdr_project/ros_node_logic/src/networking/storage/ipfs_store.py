import ipfsApi
import os

# Connect to the IPFS daemon running on localhost:5001
client = ipfsApi.Client('127.0.0.1', 5001)

def store_file(file_path):
    res = client.add(file_path)
    return res['Hash']

def get_file(ipfs_hash, output_path):
    # Downloads file and writes it to output_path
    data = client.cat(ipfs_hash)
    with open(output_path, 'wb') as f:
        f.write(data)

def add_json(obj):
    return client.add_json(obj)

def get_json(ipfs_hash):
    return client.get_json(ipfs_hash)

def list_peers():
    return client.swarm_peers()
